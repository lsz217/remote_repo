#include <chrono>
#include <memory>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <Eigen/Dense>
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/path.hpp"  // 添加路径类型

// WGS-84 参数
const double a = 6378137.0;
const double f = 1.0 / 298.257223563;
const double e2 = 2 * f - f * f;

// 角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 经纬度转 ECEF
void geodeticToECEF(double lat, double lon, double h, double& x, double& y, double& z) {
    double lat_rad = deg2rad(lat);
    double lon_rad = deg2rad(lon);
    double N = a / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
    x = (N + h) * std::cos(lat_rad) * std::cos(lon_rad);
    y = (N + h) * std::cos(lat_rad) * std::sin(lon_rad);
    z = ((1 - e2) * N + h) * std::sin(lat_rad);
}

// 构造 ECEF → ENU 的旋转矩阵
Eigen::Matrix3d getRotationMatrix(double lat0, double lon0) {
    double sin_lat = std::sin(deg2rad(lat0));
    double cos_lat = std::cos(deg2rad(lat0));
    double sin_lon = std::sin(deg2rad(lon0));
    double cos_lon = std::cos(deg2rad(lon0));
    Eigen::Matrix3d R;
    R << -sin_lon,            cos_lon,           0,
         -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
          cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat;
    return R;
}

class GpsConverterNode : public rclcpp::Node {
public:
    GpsConverterNode()
    : Node("gps_converter_node")
    {
        // 参数化原点坐标
        lat0_ = this->declare_parameter("lat0", 32.65367390);
        lon0_ = this->declare_parameter("lon0", 110.73012810);
        alt0_ = this->declare_parameter("alt0", 292.097);

        // GPS 订阅
        subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 100,  // 增加队列大小为100
            std::bind(&GpsConverterNode::gps_callback, this, std::placeholders::_1));

        // ENU 坐标发布
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps/pose", 10);

        // Path 轨迹发布
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/gps/path", 10);

        // 初始化路径消息
        path_msg_.header.frame_id = "enu";  // 设置坐标系

        // 定时器，用于降低路径发布频率
        // 创建一个 WallTimer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),  // 每1秒更新一次路径
            std::bind(&GpsConverterNode::publish_path, this)
        );
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // 使用你提供的原点坐标
        double lat0 = lat0_; // 原点纬度
        double lon0 = lon0_; // 原点经度
        double alt0 = alt0_;  // 原点海拔

        // 经纬度 → ECEF
        double x, y, z;
        geodeticToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);
        double x0, y0, z0;
        geodeticToECEF(lat0, lon0, alt0, x0, y0, z0);

        // 计算差异
        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        // 打印 ECEF 坐标差异
        RCLCPP_INFO(this->get_logger(), "ECEF Diff → dx=%.2f, dy=%.2f, dz=%.2f", dx, dy, dz);

        // 计算 ECEF → ENU 的旋转矩阵
        Eigen::Matrix3d R = getRotationMatrix(lat0, lon0);
        Eigen::Vector3d ecef_diff(dx, dy, dz);
        Eigen::Vector3d enu = R * ecef_diff;

        // 缩放 ENU 坐标（如果需要的话，可以调整这个缩放因子）
        double scale_factor = 100.0;  // 缩放因子
        enu[0] /= scale_factor;
        enu[1] /= scale_factor;
        enu[2] /= scale_factor;

        // 将此点添加到路径消息中
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();  // 使用this->now()直接获取当前时间戳
        pose_msg.header.frame_id = "enu";
        pose_msg.pose.position.x = enu[0];
        pose_msg.pose.position.y = enu[1];
        pose_msg.pose.position.z = enu[2];
        pose_msg.pose.orientation.w = 1.0;  // 设置四元数姿态

        // 保存路径点
        path_msg_.poses.push_back(pose_msg);
    }

    void publish_path()
    {
        // 发布完整路径
        path_msg_.header.stamp = this->now();
        path_publisher_->publish(path_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;  // 定时器
    nav_msgs::msg::Path path_msg_;  // 存储路径消息
    double lat0_, lon0_, alt0_;    // 原点坐标
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsConverterNode>());
    rclcpp::shutdown();
    return 0;
}





















