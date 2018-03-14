#include <limits>

#include <battery_state_interface/battery_state_interface.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/spinner.h>
#include <ros/time.h>
#include <sensor_msgs/BatteryState.h>

// example of a robot hardware having a battery interface
class FakeBatteryHW : public hardware_interface::RobotHW {
public:
  FakeBatteryHW() {
    // register batteries to the battery interface
    // so that controllers can access data from the batteries
    interface_.registerHandle(
        battery_state_interface::BatteryStateHandle("left_battery", &left_state_));
    interface_.registerHandle(
        battery_state_interface::BatteryStateHandle("right_battery", &right_state_));
    registerInterface(&interface_);
  }

  virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    initState(left_state_, "left_battery_slot");
    initState(right_state_, "right_battery_slot");
    return true;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // read dropped voltage
    left_state_.voltage = std::max(left_state_.voltage - period.toSec() / 60., 25.);
    right_state_.voltage = std::max(right_state_.voltage - period.toSec() / 100., 25.);
  }

private:
  void initState(sensor_msgs::BatteryState &state, const std::string &location) {
    state.voltage = 30.;
    state.current = std::numeric_limits< float >::quiet_NaN();
    state.charge = std::numeric_limits< float >::quiet_NaN();
    state.capacity = std::numeric_limits< float >::quiet_NaN();
    state.design_capacity = std::numeric_limits< float >::quiet_NaN();
    state.percentage = std::numeric_limits< float >::quiet_NaN();
    state.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    state.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    state.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    state.present = false;
    state.location = location;
  }

private:
  battery_state_interface::BatteryStateInterface interface_;
  sensor_msgs::BatteryState left_state_;
  sensor_msgs::BatteryState right_state_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_battery_hw");
  ros::NodeHandle nh;

  // bind hardware and controllers
  FakeBatteryHW battery_hw;
  battery_hw.init(nh, nh);
  controller_manager::ControllerManager controllers(&battery_hw);

  // run a bachground spinner for controller manager services
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // run general ros_control cycles ...
  ros::Rate rate(50.);
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    battery_hw.read(now, period);
    controllers.update(now, period);
    battery_hw.write(now, period);
    last = now;
    rate.sleep();
  }

  return 0;
}