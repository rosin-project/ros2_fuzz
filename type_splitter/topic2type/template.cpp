/*
 * This is an automatically generated file
 * Do not modify
 */

#include "rclcpp/rclcpp.hpp"
{{ IMPORTS }}
//#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("automatic_client");

    rclcpp::Client<{{ NODE_TYPE }}>::SharedPtr client =
        node->create_client<{{ NODE_TYPE }}>("{{ CLIENT_NAME }}");

    auto request = std::make_shared<{{ NODE_TYPE }}::Request>();

{{ REQUEST_CODE }}

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response!");
        // Well received!
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}
