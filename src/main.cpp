#ifndef __OBJECTS_H__
#define __OBJECTS_H__
#include "Headers/Common.hpp"
#include "Headers/Utils.hpp"
#include "Headers/Objects.hpp"
#include "Headers/Publishers.hpp"
#include "Headers/PointClouds.hpp"
#include "Headers/Accumulator.hpp"
#include "Headers/Compensator.hpp"
#include "Headers/Localizator.hpp"
#include "Headers/Mapper.hpp"
#endif

Params Config;

void fill_config(ros::NodeHandle& nh);

/**
 * 【功能描述】：LIMO-Velo系统的主函数，实现实时LiDAR-IMU里程计和建图
 * @param argc：命令行参数个数
 * @param argv：命令行参数数组
 * @return 程序退出状态码，0表示正常退出
 */
int main(int argc, char** argv) {
    // 初始化ROS节点，节点名称为"limovelo"
    ros::init(argc, argv, "limovelo");
    ros::NodeHandle nh;

    // 从YAML配置文件中填充全局配置参数
    fill_config(nh);

    // 创建核心对象实例
    Publishers publish(nh);                           // 发布器，用于发布状态、点云、TF等信息
    Accumulator& accum = Accumulator::getInstance();  // 数据累加器单例，收集LiDAR和IMU数据
    Compensator comp = Compensator();                 // 运动补偿器，用于补偿点云运动畸变
    Mapper& map = Mapper::getInstance();              // 地图构建器单例，维护全局地图
    Localizator& loc = Localizator::getInstance();    // 定位器单例，估计机器人位姿

    // 设置ROS订阅器
    // 订阅LiDAR点云数据，队列大小1000，回调函数为Accumulator::receive_lidar
    ros::Subscriber lidar_sub = nh.subscribe(
        Config.points_topic, 1000,
        &Accumulator::receive_lidar, &accum
    );

    // 订阅IMU数据，队列大小1000，回调函数为Accumulator::receive_imu
    ros::Subscriber imu_sub = nh.subscribe(
        Config.imus_topic, 1000,
        &Accumulator::receive_imu, &accum
    );

    // 时间变量初始化
    double t1, t2;                                    // t1: 时间窗口起始时间, t2: 时间窗口结束时间
    t2 = DBL_MAX;                                     // 初始化t2为最大值，确保第一次运行时能正确设置

    // delta表示用于定位的视场大小（时间窗口大小）
    double delta = Config.Initialization.deltas.front();
    
    // 设置循环频率为5000Hz，保证高频率处理
    ros::Rate rate(5000);

    // 主处理循环
    while (ros::ok()) {
        
        // 当累加器收集到足够数据时开始处理
        while (accum.ready()) {
            
            /**
             * 步骤0：时间管理
             * 定义用于定位的时间区间[t1, t2]
             */
                
                // 实时模式：将t2设置为最新数据时间
                if (Config.real_time) t2 = accum.latest_time();
                // 非实时模式：t2 = 上一个t2 + delta，但不能超过最新数据时间
                else t2 = std::min(t2 + delta, accum.latest_time());
                
                // 根据累加器状态动态更新delta值
                delta = accum.update_delta(Config.Initialization, t2);

                // 设置t1，确保不重复定位相同点云数据
                t1 = std::max(t2 - delta, loc.last_time_updated);
                // 检查时间区间是否有足够的视场范围，不足则跳出
                if (t2 - t1 < delta - 1e-6) break;

            /**
             * 步骤1：定位处理
             * 通过IMU预测和LiDAR校正估计机器人位姿
             */

                // 使用IMU数据预测到时间t2的位姿
                loc.propagate_to(t2);

                // 对时间区间[t1, t2]内的点云进行运动补偿
                Points compensated = comp.compensate(t1, t2);
                // 对补偿后的点云进行下采样，减少计算量
                Points ds_compensated = comp.downsample(compensated);
                // 检查下采样后点云数量是否足够进行匹配
                if (ds_compensated.size() < Config.MAX_POINTS2MATCH) break; 

                // 使用下采样点云在地图中进行定位校正
                loc.correct(ds_compensated, t2);
                State Xt2 = loc.latest_state();           // 获取最新估计状态
                accum.add(Xt2, t2);                       // 将状态添加到累加器
                publish.state(Xt2, false);                // 发布机器人状态
                publish.tf(Xt2);                          // 发布TF变换

                // 将补偿后的点云转换到全局坐标系并发布
                Points global_compensated = Xt2 * Xt2.I_Rt_L() * compensated;
                Points global_ds_compensated = Xt2 * Xt2.I_Rt_L() * ds_compensated;
                publish.pointcloud(global_ds_compensated, true);  // 发布用于定位的点云

                // 如果启用外参打印，发布更新的外参
                if (Config.print_extrinsics) publish.extrinsics(Xt2);

            /**
             * 步骤2：地图构建
             * 根据配置选择在线或离线建图模式
             */

                // 在线建图模式：实时将点云添加到地图
                if (Config.mapping_online) {
                    map.add(global_ds_compensated, t2, true);
                    // 根据质量设置选择发布原始或下采样点云
                    if (Config.high_quality_publish) publish.pointcloud(global_compensated, false);                    
                    else publish.pointcloud(global_ds_compensated, false);                    
                }
                // 离线建图模式：在特定时间点进行建图
                else if (map.hasToMap(t2)) {
                    State Xt2 = loc.latest_state();
                    // 对完整旋转周期[t2 - FULL_ROTATION_TIME, t2]内的点云进行建图
                    Points full_compensated = comp.compensate(t2 - Config.full_rotation_time, t2);
                    Points global_full_compensated = Xt2 * Xt2.I_Rt_L() * full_compensated;
                    Points global_full_ds_compensated = comp.downsample(global_full_compensated);

                    map.add(global_full_ds_compensated, t2, true);
                    // 根据质量设置发布相应点云
                    if (Config.high_quality_publish) publish.pointcloud(global_full_compensated, false);
                    else publish.pointcloud(global_full_ds_compensated, false);
                }

            /**
             * 步骤3：数据清理
             * 删除过旧的数据以节省内存
             */

                // 清除过旧的LiDAR点云数据，保持内存使用合理
                accum.clear_lidar(t2 - Config.empty_lidar_time);

            // 处理完一帧数据后跳出内层循环（技巧性实现）
            break;
        }

        // 处理ROS回调函数
        ros::spinOnce();
        // 按设定频率休眠
        rate.sleep();
    }

    return 0;
}

void fill_config(ros::NodeHandle& nh) {
    // Read YAML parameters
    nh.param<bool>("mapping_online", Config.mapping_online, true);
    nh.param<bool>("real_time", Config.real_time, true);
    nh.param<bool>("estimate_extrinsics", Config.estimate_extrinsics, false);
    nh.param<bool>("print_extrinsics", Config.print_extrinsics, false);
    nh.param<int>("downsample_rate", Config.downsample_rate, 4);
    nh.param<float>("downsample_prec", Config.downsample_prec, 0.2);
    nh.param<bool>("high_quality_publish", Config.high_quality_publish, false);
    nh.param<int>("MAX_NUM_ITERS", Config.MAX_NUM_ITERS, 3);
    nh.param<std::vector<double>>("LIMITS", Config.LIMITS, std::vector<double> (23, 0.001));
    nh.param<int>("NUM_MATCH_POINTS", Config.NUM_MATCH_POINTS, 5);
    nh.param<int>("MAX_POINTS2MATCH", Config.MAX_POINTS2MATCH, 10);
    nh.param<double>("MAX_DIST_PLANE", Config.MAX_DIST_PLANE, 2.0);
    nh.param<float>("PLANES_THRESHOLD", Config.PLANES_THRESHOLD, 0.1f);
    nh.param<float>("PLANES_CHOOSE_CONSTANT", Config.PLANES_CHOOSE_CONSTANT, 9.0f);
    nh.param<std::string>("LiDAR_type", Config.LiDAR_type, "unknown");
    nh.param<double>("LiDAR_noise", Config.LiDAR_noise, 0.001);
    nh.param<double>("min_dist", Config.min_dist, 3.);
    nh.param<double>("imu_rate", Config.imu_rate, 400);
    nh.param<double>("degeneracy_threshold", Config.degeneracy_threshold, 5.d);
    nh.param<bool>("print_degeneracy_values", Config.print_degeneracy_values, false);
    nh.param<double>("full_rotation_time", Config.full_rotation_time, 0.1);
    nh.param<double>("empty_lidar_time", Config.empty_lidar_time, 20.);
    nh.param<double>("real_time_delay", Config.real_time_delay, 1.);
    nh.param<double>("covariance_gyroscope", Config.cov_gyro, 1e-4);
    nh.param<double>("covariance_acceleration", Config.cov_acc, 1e-2);
    nh.param<double>("covariance_bias_gyroscope", Config.cov_bias_gyro, 1e-5);
    nh.param<double>("covariance_bias_acceleration", Config.cov_bias_acc, 1e-4);
    nh.param<double>("wx_MULTIPLIER", Config.wx_MULTIPLIER, 1);
    nh.param<double>("wy_MULTIPLIER", Config.wy_MULTIPLIER, 1);
    nh.param<double>("wz_MULTIPLIER", Config.wz_MULTIPLIER, 1);
    nh.param<std::string>("points_topic", Config.points_topic, "/velodyne_points");
    nh.param<std::string>("imus_topic", Config.imus_topic, "/vectornav/IMU");
    nh.param<bool>("offset_beginning", Config.offset_beginning, false);
    nh.param<bool>("stamp_beginning", Config.stamp_beginning, false);
    nh.param<std::vector<double>>("/Initialization/times", Config.Initialization.times, {});
    nh.param<std::vector<double>>("/Initialization/deltas", Config.Initialization.deltas, {Config.full_rotation_time});
    nh.param<std::vector<float>>("initial_gravity", Config.initial_gravity, {0.0, 0.0, -9.807});
    nh.param<std::vector<float>>("I_Translation_L", Config.I_Translation_L, std::vector<float> (3, 0.));
    nh.param<std::vector<float>>("I_Rotation_L", Config.I_Rotation_L, std::vector<float> (9, 0.));
}