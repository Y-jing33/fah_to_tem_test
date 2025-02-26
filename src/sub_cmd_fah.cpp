// 订阅华氏度数据并发布摄氏度数据的节点
#include <memory>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class TemperatureConverter : public rclcpp::Node
{
public:
  TemperatureConverter() : Node("temperature_converter")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "fahrenheit", 10, std::bind(&TemperatureConverter::fahrenheit_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("celsius", 10);
  }

private:
  void fahrenheit_callback(const std_msgs::msg::Float64::SharedPtr msg) const
  {
    double celsius = (msg->data - 32.0) * 5.0 / 9.0;
    auto message = std_msgs::msg::Float64();
    message.data = celsius;
    RCLCPP_INFO(this->get_logger(), "Received: '%f' Fahrenheit, Converted: '%f' Celsius", msg->data, celsius);
    publisher_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperatureConverter>());
  rclcpp::shutdown();
  return 0;
}