#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sema_bmc/srv/sema_cmd.hpp"
#ifndef _WIN32
#include "linux/EApiOs.h"
#else /* _WIN32 */
#include "winnt/EApiOs.h"
#endif /* _WIN32 */
#include "EApi.h"
#include "semaeapi.h"

namespace sema_server_node
{

class SemaServerNode : public rclcpp::Node
{
public:
  explicit SemaServerNode(const rclcpp::NodeOptions & options)
  : Node("sema_server", options)
  {
    uint32_t ret;
    char ipAddr[] = "127.0.0.1";
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    ret = SemaEApiLibInitialize(false, IP_V4, ipAddr, 0, (char*)"123", &sema_handler);
    if (ret != EAPI_STATUS_SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Can't initialize SEMA Lib. Make sure you are root. Error code: %X.", ret);
      return;
    }
    auto sema_cmd_service =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<sema_bmc::srv::SemaCmd::Request> request,
        std::shared_ptr<sema_bmc::srv::SemaCmd::Response> response) -> void
      {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "Incoming request: cmd=%s sub_cmd=%s param=%d", request->cmd.c_str(), request->sub_cmd.c_str(), request->param);
        if (request->cmd == "info") {
          uint32_t id;
          response->ret = -1;
          response->result = 0;
          if (request->sub_cmd == "cpu") {
            id = EAPI_ID_HWMON_CPU_TEMP;
          } else if (request->sub_cmd == "fan") {
            id = EAPI_ID_HWMON_FAN_CPU;
          } else if (request->sub_cmd == "voltage") {
            id = SEMA_EAPI_ID_HWMON_VOLTAGE_VIN;
          } else if (request->sub_cmd == "current") {
            id = SEMA_EAPI_ID_BOARD_MAIN_CURRENT;
          }
          response->ret = SemaEApiBoardGetValue(sema_handler, id, &response->result);
        } else if (request->cmd == "watchdog") {
          if (request->sub_cmd == "start") {
            response->ret = SemaEApiWDogStart(sema_handler, request->param, 0, 0);
          } else if (request->sub_cmd == "trigger") {
            response->ret = SemaEApiWDogTrigger(sema_handler);
          } else if (request->sub_cmd == "stop") {
            response->ret = SemaEApiWDogStop(sema_handler);
          }
        } else if (request->cmd == "fan") {
          if (request->sub_cmd == "mode") {
            response->ret = SemaEApiSmartFanSetMode(sema_handler, SEMA_EAPI_ID_FAN_CPU, (request->param)?2:1);
          }
        }
      };
    // Create a service that will use the callback function to handle requests. The service's name is example_service_name.
    RCLCPP_INFO(this->get_logger(), "Service starts to listening...");
    srv_ = create_service<sema_bmc::srv::SemaCmd>("sema_cmd", sema_cmd_service);
  }
  ~SemaServerNode()
  {
    SemaEApiLibUnInitialize(sema_handler);
  }

private:
  rclcpp::Service<sema_bmc::srv::SemaCmd>::SharedPtr srv_;
  uint32_t sema_handler;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(sema_server_node::SemaServerNode)