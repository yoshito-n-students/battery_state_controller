#include "battery_state_controller/battery_state_controller.hpp"

#include<pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(battery_state_controller::BatteryStateController,
                       controller_interface::ControllerBase);