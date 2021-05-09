/*
 * This is an automatically generated file. Do not modify.
 * 
 * This file contains the ros2_automatic_fuzzer implementation
 * for the `{{ FILE_NAME }}` server source file.
 */
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

{{ IMPORTS }}

{{ FUZZING_API }}

void fuzz_target(int argc, char const *const argv[])
{
    rclcpp::init(argc, argv);

    // Create node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("automatic_client");

    rclcpp::Client<{{ NODE_TYPE }}>::SharedPtr client =
        node->create_client<{{ NODE_TYPE }}>("{{ CLIENT_NAME }}");

    auto request = std::make_shared<{{ NODE_TYPE }}::Request>();

    /* START SINGLE ITERATION */

{{ REQUEST_CODE }}

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("automatic_fuzzing_logger"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        //RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "Sum: %ld", result.get()->sum);
        RCLCPP_INFO(rclcpp::get_logger("automatic_fuzzing_logger"), "Received response!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("automatic_fuzzing_logger"), "Failed to call service");
    }

    /* FINISH SINGLE ITERATION */

    rclcpp::shutdown();
}

static void kill_pid(const pid_t& pid)
{
    std::cout << "Time is up. Good job! Killing parent." << std::endl;
    kill(pid, SIGUSR1);
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}

static void treat_timeout_signal(int signum)
{
    if (signum == SIGUSR1) {
        std::cout << "It is time to finish!" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_SUCCESS);
    }
}

static void __attribute__((constructor)) __injector_init()
{
    pid_t parent_pid = getpid();
    pid_t pid = fork();

    if (pid < 0) {
        std::cerr << "Could not fork!" << std::endl;
        exit(-1);
    } else if (pid == 0) {
        int argc = 1;
        const char* argv[] = {"./my_fuzz_target", NULL};
        fuzz_target(argc, argv);

        // Kill the parent
        std::cout << "Killing the parent" << std::endl;
        kill_pid(parent_pid);
        exit(0);
    }
    // Parent's code
    signal(SIGUSR1, treat_timeout_signal);

    // Close standard input in the node
    close(0);

    // Continue normal system under test code
    std::cout << "Continuing normal code" << std::endl;
}