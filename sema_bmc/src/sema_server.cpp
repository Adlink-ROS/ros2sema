#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sema_bmc/srv/sema_cmd.hpp"

namespace sema_server_node
{

class SemaServerNode : public rclcpp::Node
{
public:
  explicit SemaServerNode(const rclcpp::NodeOptions & options)
  : Node("sema_server", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto sema_cmd_service =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<sema_bmc::srv::SemaCmd::Request> request,
        std::shared_ptr<sema_bmc::srv::SemaCmd::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request: cmd=%s param=%d", request->cmd.c_str(), request->param);
        response->ret = 0;
        response->result = 0;
      };
    // Create a service that will use the callback function to handle requests. The service's name is example_service_name.
    srv_ = create_service<sema_bmc::srv::SemaCmd>("sema_cmd", sema_cmd_service);
  }

private:
  rclcpp::Service<sema_bmc::srv::SemaCmd>::SharedPtr srv_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(sema_server_node::SemaServerNode)