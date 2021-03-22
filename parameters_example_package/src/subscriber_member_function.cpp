#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:

    MinimalSubscriber() : Node("minimal_subscriber")
    {
      // Declare some parameters so that we can fuzz them
      this->declare_parameter<std::string> ("string_parameter", "world");
      this->declare_parameter<int> ("int_parameter", -1);
      this->declare_parameter<long> ("bool_parameter", false);
      this->declare_parameter<double> ("double_parameter", 0.0);

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    #pragma clang diagnostic ignored "-Wunused-parameter"
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // empty to make fuzzing faster
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main (int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
