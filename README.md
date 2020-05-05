# ros2sema
A toolset for interfacing ADLINK SEMA library for ROS2 dashing.

# sema_bmc usage
* server side (**Need to be root!**)
  - `ros2 run sema_bmc_sema_server`
* client side
  - `ros2 run sema_bmc sema_client <cmd> <sub_cmd> <param>`
  - Support cmd and sub_cmd:
    * info: cpu, fan, voltage, current
    * watchdog: start, trigger, stop
    * fan: set
