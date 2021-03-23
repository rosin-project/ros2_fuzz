// This is the eloquent version (the API has tiny diffrences starting foxy)
// https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/

#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

/** Message generation API **/

bool getFloat64 (double& d) 
{
    char * bytes = (char *) &d;

    for (size_t i = 0; i < sizeof (double); ++i) {
        int c = getchar ();
        if (c == EOF) return false;
        bytes[i] =  (char) c;
    }

    return true;
}


bool getInt8 (int8_t& out) 
{
  int c = getchar ();
  out = c;
  return (c != EOF);
}


bool getUInt8 (uint8_t& out) 
{
  return getInt8 ((int8_t&) out);
}


bool getInt64(int64_t& out) 
{
    char * bytes = (char *) &out;

    for (size_t i = 0; i < sizeof (int64_t); ++i) {
      int c = getchar ();
      if (c == EOF) return false;
      bytes[i] = (char) c;
    }

    return true;
}


bool getBool (bool& b) 
{
    int c = getchar();
    if (c == EOF)
        return false;
 
    b = (c % 2 == 0);
    return true;
}

bool getString(std::string& s, uint8_t size) {

    s = "";

    for (size_t i = 0; i < size; ++i) {
        int c = getchar ();
        if (c == EOF)
          return false;
        s += (char) c;
    }

    return true;
}


//
// 1. Server
//
void fuzz_server() {

  long a, b;
  auto node = rclcpp::Node::make_shared ("add_two_ints_client");

  auto client = 
    node->create_client<example_interfaces::srv::AddTwoInts> ("add_two_ints");

    while (!client->wait_for_service ()) {

      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

  while (getInt64(a) && getInt64(b)) {

    auto request = 
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    request->a = a;
    request->b = b;

    auto result = client->async_send_request (request);
    rclcpp::spin_until_future_complete (node, result);

  }
}


//
// 2. Subscriber
//
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      1ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      std::string content;
      if (!getString(content, 3)) {
        rclcpp::shutdown();
        exit(EXIT_SUCCESS);
      }

      auto message = std_msgs::msg::String();
      message.data = content;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish (message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


void fuzz_subscriber() {
  rclcpp::spin(std::make_shared<MinimalPublisher>());
}



/*!  
 * \brief Fuzz the parameter database facility (setting and getting)  in a
 * remote node.
 */
void fuzz_parameters () { std::shared_ptr<rclcpp::Node> node =
  rclcpp::Node::make_shared ("fuzz_param_api");

  int64_t i;
  uint8_t size;
  std::string str;
  bool j;
  double d;

  std::shared_ptr<rclcpp::AsyncParametersClient> client = 
    std::make_shared<rclcpp::AsyncParametersClient> (node, "/minimal_subscriber");

  while (! client->wait_for_service ()) {
    if (!rclcpp::ok()) return;
  }

  while (getInt64 (i) && getUInt8 (size) && getString (str, size) && getBool (j) && getFloat64 (d)) {

    rclcpp::Parameter param_i ("int_parameter", i);
    rclcpp::Parameter param_str ("string_parameter", str);
    rclcpp::Parameter param_j ("bool_parameter", j);
    rclcpp::Parameter param_d ("double_parameter", d);

    auto f1 = client->set_parameters (std::vector<rclcpp::Parameter> (1, param_i));
    rclcpp::spin_until_future_complete (node, f1);
    f1 = client->set_parameters (std::vector<rclcpp::Parameter> (1, param_str));
    rclcpp::spin_until_future_complete (node, f1);
    f1 = client->set_parameters (std::vector<rclcpp::Parameter> (1, param_j));
    rclcpp::spin_until_future_complete (node, f1);
    f1 = client->set_parameters (std::vector<rclcpp::Parameter> (1, param_d));
    rclcpp::spin_until_future_complete (node, f1);

    f1 = client->set_parameters ( { param_i, param_str, param_j, param_d } );
    rclcpp::spin_until_future_complete (node, f1);
    auto f2 = client->set_parameters_atomically ( { param_i, param_str, param_j, param_d } );
    rclcpp::spin_until_future_complete (node, f2);

    auto f3 = 
      client->get_parameters ( {"string_parameter", "int_parameter", "bool_parameter", "double_parameter"} );
    
    rclcpp::spin_until_future_complete (node, f3);

    auto result = f3.get();

    auto param = result.at(0);
    std::string str1 = param.as_string ();
    param.value_to_string ();

    param = result.at (1);
    int64_t i1 = param.as_int ();
    param.value_to_string ();

    param = result.at (2);
    bool j1 = param.as_bool ();
    param.value_to_string ();

    param = result.at (3);
    double d1 = param.as_double ();
    param.value_to_string ();

    assert (str == str1);
    assert (i == i1);
    assert (j == j1);
    assert ( d != d || d == d1);
  }

}



/*! 
 * \brief Fuzz the parameter database facility in this node (setting and
 * getting) 
 */
void fuzz_local_parameters () 
{ 
  std::shared_ptr<rclcpp::Node> node = 
    rclcpp::Node::make_shared ("fuzz_local_param_api");

  node->declare_parameter<std::string> ("string_parameter", "world");
  node->declare_parameter<int64_t> ("int_parameter", -1);
  node->declare_parameter<bool> ("bool_parameter", false);
  node->declare_parameter<double> ("double_parameter", 0.0);

  int64_t i;
  uint8_t size;
  std::string str = "";
  bool j;
  double d;

  while (getInt64 (i) && getUInt8 (size) && getString (str, size) && getBool (j) && getFloat64 (d)) {

    rclcpp::Parameter param_i ("int_parameter", i);
    rclcpp::Parameter param_str ("string_parameter", str);
    rclcpp::Parameter param_j ("bool_parameter", j);
    rclcpp::Parameter param_d ("double_parameter", d);

    node->set_parameter ( { param_i } );
    node->set_parameter ( { param_str } );
    node->set_parameter ( { param_j } );
    node->set_parameter ( { param_d } );

    node->set_parameters ( { param_i, param_str, param_j, param_d } );
    node->set_parameters_atomically ( { param_i, param_str, param_j, param_d } );

    auto result = 
       node->get_parameters ( {"string_parameter", "int_parameter", "bool_parameter", "double_parameter"} );

    auto param = result.at (0);
    std::string str1 = param.as_string ();
    param.value_to_string ();

    param = result.at (1);
    int64_t i1 = param.as_int ();
    param.value_to_string ();

    param = result.at (2);
    bool j1 = param.as_bool ();
    param.value_to_string ();

    param = result.at (3);
    double d1 = param.as_double ();
    param.value_to_string ();

    assert (str == str1);
    assert (i == i1);
    assert (j == j1);
    assert ( d != d || d == d1);
  }

}



int main (int argc, char **argv)
{
  uint8_t selector;
  rclcpp::init (argc, argv);

  if (getUInt8 (selector))  {
    switch (selector % 4) {

      case 0: 
        fuzz_server ();
        break;

      case 1:
        fuzz_subscriber ();
        break;

      case 2:
        fuzz_parameters ();

      case 3:
        fuzz_local_parameters ();
    }
  }
  
  rclcpp::shutdown ();

  return 0;
}
