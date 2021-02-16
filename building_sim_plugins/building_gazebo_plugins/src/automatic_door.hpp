/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef BUILDING_SIM_COMMON__AUTOMATIC_DOOR_GAZEBO_HPP
#define BUILDING_SIM_COMMON__AUTOMATIC_DOOR_GAZEBO_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <google/protobuf/descriptor.h>
#include <gazebo/transport/transport.hh>
#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>
#include <rmf_door_msgs/msg/supervisor_heartbeat.hpp>

using namespace building_sim_common;

namespace building_gazebo_plugins {
//==============================================================================
namespace automatic_door {

using namespace google::protobuf;
gazebo::physics::ModelPtr _model;

class AutomaticDoorPlugin : public gazebo::ModelPlugin
{
public:
  AutomaticDoorPlugin() {}

  ~AutomaticDoorPlugin()
  {
    if (_main_thread.joinable())
      _main_thread.join();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  std::shared_ptr<DoorCommon> _door_common = nullptr;
  void on_update();
  struct moving_state
  {
    enum current_state { OPEN, OPENING, CLOSED, CLOSING} _current_state;
    int _recent_velocity_sign[10]; // plus as true and minus as false
    int index;
    void set_velocity_sign(double velocity)
    {
      if (velocity > 1e-10)
        _recent_velocity_sign[index] = 1;
      else if (velocity < -1e-10)
        _recent_velocity_sign[index] = -1;
      else
        _recent_velocity_sign[index] = 0;

      index = ++index % 10;
    }
    int get_majority_sign()
    {
      int plus_sign = 0;
      int neutral_sign = 0;
      int minus_sign = 0;
      for (auto velocity_sign:_recent_velocity_sign)
      {
        if (velocity_sign == 1)
          plus_sign++;
        else if (velocity_sign == -1)
          minus_sign++;
        else
          neutral_sign++;
      }
      if (plus_sign >= minus_sign && plus_sign >= neutral_sign)
        return 1;
      else if (minus_sign >= plus_sign && minus_sign >= neutral_sign)
        return -1;
      else
        return 0;
    }
  } ms;

private:
  std::thread _main_thread;
  gazebo::event::ConnectionPtr _update_connection;
  std::unordered_map<std::string, gazebo::physics::JointPtr> _joints;
  bool _initialized = false;
};

class DoorWatcherNode : public rclcpp::Node
{
public:
  const std::string FinalDoorRequestTopicName = "door_requests";
  const std::string AdapterDoorRequestTopicName = "adapter_door_requests";
  const std::string DoorStateTopicName = "door_states";
  const std::string DoorSupervisorHeartbeatTopicName =
    "door_supervisor_heartbeat";
  using DoorMode = rmf_door_msgs::msg::DoorMode;
  using DoorRequest = rmf_door_msgs::msg::DoorRequest;
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
  using Heartbeat = rmf_door_msgs::msg::SupervisorHeartbeat;
  using HeartbeatPub = rclcpp::Publisher<Heartbeat>;
  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateSub = rclcpp::Subscription<DoorState>;
  DoorStateSub::SharedPtr _door_state_sub;
  HeartbeatPub::SharedPtr _door_heartbeat_pub;
  DoorRequest _previous_request;
  DoorState _state;
  std::vector<double> _previous_scan_1;
  std::vector<double> _previous_scan_2;
  DoorRequestPub::SharedPtr _door_request_pub;
  enum current_state { OPEN, MOVING, CLOSED} _current_state;
  enum last_state { LAST_OPEN, LAST_CLOSED} _last_state;
  double t_since_transition_state;
  AutomaticDoorPlugin* _parent;
  bool _last_fluctuation;

  DoorWatcherNode()
  : rclcpp::Node("door_watcher")
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    _door_request_pub = create_publisher<DoorRequest>(
      AdapterDoorRequestTopicName, default_qos);
    _timestep = 0;
    _timer = false;
  }
  ~DoorWatcherNode() {}
  static void init_and_make(AutomaticDoorPlugin* parent);
  static void cb1(ConstLaserScanStampedPtr& _msg);
  static void cb2(ConstLaserScanStampedPtr& _msg);
  static void process_scan_and_actuate(ConstLaserScanStampedPtr& _msg,
    std::vector<double>& _previous_scan);
  uint16_t _timestep;
  bool _timer;
};

typedef std::shared_ptr<DoorWatcherNode> _DWN_PTR;
_DWN_PTR dwn_ptr = nullptr;

}
} // namespace building_gazebo_plugins

#endif // BUILDING_SIM_COMMON__AUTOMATIC_DOOR_GAZEBO_HPP