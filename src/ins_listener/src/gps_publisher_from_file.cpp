#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GPSDataPublisher : public rclcpp::Node
{
public:
    GPSDataPublisher()
    : Node("gps_data_publisher"), current_index_(0)
    {
        // 创建发布器，话题为 /gps/fix，消息类型为 NavSatFix
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

        // 创建定时器：每 100ms 发布一次数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GPSDataPublisher::publish_gps_data, this));

        // 读取数据文件
        read_gps_data_from_file("/home/lsz/下载/gps.txt");

        RCLCPP_INFO(this->get_logger(), "GPSDataPublisher node started");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::tuple<double, double, double>> gps_data_;
    size_t current_index_;

    void read_gps_data_from_file(const std::string &file_path)
    {
        std::ifstream file(file_path);
        std::string line;

        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        while (std::getline(file, line)) {
            double lat, lon, alt;
            if (sscanf(line.c_str(), "lat = %lf\tlon = %lf\talt = %lf", &lat, &lon, &alt) == 3) {
                gps_data_.emplace_back(lat, lon, alt);
                RCLCPP_INFO(this->get_logger(), "Read GPS → Lat: %.8f, Lon: %.8f, Alt: %.2f", lat, lon, alt);
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu GPS entries.", gps_data_.size());
    }

    void publish_gps_data()
    {
        if (gps_data_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No GPS data to publish!");
            return;
        }

        auto msg = sensor_msgs::msg::NavSatFix();
        auto &[lat, lon, alt] = gps_data_[current_index_];

        msg.latitude = lat;
        msg.longitude = lon;
        msg.altitude = alt;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "gps_link";  // 可根据需要修改

        RCLCPP_INFO(this->get_logger(), "Publishing → Lat: %.8f, Lon: %.8f, Alt: %.2f", lat, lon, alt);
        publisher_->publish(msg);

        current_index_ = (current_index_ + 1) % gps_data_.size();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSDataPublisher>());
    rclcpp::shutdown();
    return 0;
}



