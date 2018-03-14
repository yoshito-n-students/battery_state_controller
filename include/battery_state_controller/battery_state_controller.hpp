#ifndef BATTERY_STATE_CONTROLLER_BATTERY_STATE_CONTROLLER
#define BATTERY_STATE_CONTROLLER_BATTERY_STATE_CONTROLLER

#include <string>
#include <vector>

#include <battery_state_interface/battery_state_interface.hpp>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/BatteryState.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace battery_state_controller {

class BatteryStateController
    : public controller_interface::Controller< battery_state_interface::BatteryStateInterface > {
public:
  BatteryStateController() {}

  virtual bool init(battery_state_interface::BatteryStateInterface *hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &controller_nh) {
    // load params
    interval_ = ros::Duration(controller_nh.param("interval", 10.));

    // get battery handles registered by the hardware
    const std::vector< std::string > battery_names(hw->getNames());
    for (std::vector< std::string >::const_iterator battery_name = battery_names.begin();
         battery_name != battery_names.end(); ++battery_name) {
      batteries_.push_back(hw->getHandle(*battery_name));
      publishers_.push_back(boost::make_shared< Publisher >(root_nh, *battery_name, 1));
    }
    deadlines_.resize(battery_names.size(), ros::Time(0)); // zero time means no deadline

    // get the controller name just for console output
    controller_name_ = controller_nh.getNamespace().substr(root_nh.getNamespace().size());
    ROS_INFO_STREAM(controller_name_ << " found " << battery_names.size() << " batteries");

    return true;
  }

  virtual void starting(const ros::Time &time) {
    // set publish deadlines as the current time
    for (std::vector< ros::Time >::iterator deadline = deadlines_.begin();
         deadline != deadlines_.end(); ++deadline) {
      *deadline = time;
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period) {
    for (std::size_t i = 0; i < batteries_.size(); ++i) {
      // check if deadline has come
      if (deadlines_[i].isZero() || deadlines_[i] > time) {
        continue;
      }
      // check if can publish
      if (!publishers_[i]->trylock()) {
        ROS_WARN_STREAM(controller_name_ << " could not own the publisher associated with "
                                         << batteries_[i].getName()
                                         << ". Will retry in the next cycle.");
        continue;
      }
      // publish message
      publishers_[i]->msg_ = batteries_[i].getData();
      publishers_[i]->msg_.header.stamp = time;
      publishers_[i]->unlockAndPublish();
      // set next deadline
      deadlines_[i] += interval_;
    }
  }

  virtual void stopping(const ros::Time &time) {
    // unset publish deadlines
    for (std::vector< ros::Time >::iterator deadline = deadlines_.begin();
         deadline != deadlines_.end(); ++deadline) {
      *deadline = ros::Time(0);
    }
  }

private:
  typedef realtime_tools::RealtimePublisher< sensor_msgs::BatteryState > Publisher;

  std::vector< battery_state_interface::BatteryStateHandle > batteries_;
  std::vector< boost::shared_ptr< Publisher > > publishers_;
  std::vector< ros::Time > deadlines_;
  ros::Duration interval_;

  std::string controller_name_;
};

} // namespace battery_state_controller

#endif