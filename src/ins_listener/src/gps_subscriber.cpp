#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsSubscriber : public rclcpp::Node
{
public:
  GpsSubscriber()
  : Node("gps_subscriber")
  {
    // 订阅 /gps/fix 话题，队列长度 10
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10,
      std::bind(&GpsSubscriber::gps_callback, this, std::placeholders::_1));

    // 打印订阅器启动的日志
    RCLCPP_INFO(this->get_logger(), "GpsSubscriber node started, waiting for messages...");
  }

private:
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    double latitude  = msg->latitude;
    double longitude = msg->longitude;
    double altitude  = msg->altitude;

    // 打印接收到的数据
    RCLCPP_INFO(this->get_logger(),
      "Received GPS → Lat: %.8f, Lon: %.8f, Alt: %.2f",
      latitude, longitude, altitude);
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // 打印启动信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting GPS subscriber...");
  rclcpp::spin(std::make_shared<GpsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
