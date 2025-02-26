// 发布华氏度数据的节点
#include <memory>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"

class FahrenheitPublisher : public rclcpp::Node
{
public:
  FahrenheitPublisher() : Node("fahrenheit_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("fahrenheit", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&FahrenheitPublisher::publish_temperature, this));
  }


private:
  void publish_temperature()
  {
    auto message = std_msgs::msg::Float64();
    message.data = current_temperature_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f' Fahrenheit", message.data);
    publisher_->publish(message);
    current_temperature_ += 1.0; // 每次发布增加1度
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_temperature_ = 32.0; // 初始温度
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FahrenheitPublisher>());
  rclcpp::shutdown();
  return 0;
}