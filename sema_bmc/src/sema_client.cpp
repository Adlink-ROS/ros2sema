#include "rclcpp/rclcpp.hpp"

#include "sema_bmc/srv/sema_cmd.hpp"

using namespace std::chrono_literals;  // used by 1s

sema_bmc::srv::SemaCmd::Response::SharedPtr send_request(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<sema_bmc::srv::SemaCmd>::SharedPtr client,
    sema_bmc::srv::SemaCmd::Request::SharedPtr request)
{
    auto result = client->async_send_request(request);
    // Waiting
    if (rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return result.get();
    } else {
        return NULL;
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Node name
    auto node = rclcpp::Node::make_shared("sema_client");
    // Ceraete a client to query service name "sema_cmd"
    auto client = node->create_client<sema_bmc::srv::SemaCmd>("sema_cmd");

    // Build the request
    auto request = std::make_shared<sema_bmc::srv::SemaCmd::Request>();
    request->cmd = (argc >= 2)?argv[1]:"info";
    request->sub_cmd = (argc >= 3)?argv[2]:"cpu";
    request->param = (argc >= 4)?atoi(argv[3]):0;

    // Waiting until the service is up
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service...");
    }

    // Send the request
    auto result = send_request(node, client, request);
    if (result) {
        RCLCPP_INFO(node->get_logger(), "result=%d", result->result);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Catch interrupt and stop the program!");
    }

    rclcpp::shutdown();

    return 0;
}