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

#ifndef BUILDING_SIM_COMMON__AUTOMATIC_DOOR_COMMON_HPP
#define BUILDING_SIM_COMMON__AUTOMATIC_DOOR_COMMON_HPP

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
#include "automatic_door.hpp"

using namespace building_sim_common;

namespace building_gazebo_plugins {
//==============================================================================
namespace automatic_door {

class AutomaticDoorPlugin;

using DoorMode = rmf_door_msgs::msg::DoorMode;
using DoorState = rmf_door_msgs::msg::DoorState;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;

using namespace google::protobuf;

//==============================================================================
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
  using DoorState = rmf_door_msgs::msg::DoorState;
  using DoorStateSub = rclcpp::Subscription<DoorState>;
  DoorStateSub::SharedPtr _door_state_sub;
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

  DoorWatcherNode(std::string door_watcher_name)
  : rclcpp::Node(door_watcher_name)
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    _door_request_pub = create_publisher<DoorRequest>(
      AdapterDoorRequestTopicName, default_qos);
    _timestep = 0;
    _timer = false;
  }
  ~DoorWatcherNode() {}
  static std::shared_ptr<DoorWatcherNode> init_and_make(
    AutomaticDoorPlugin* parent);
  void cb1(ConstLaserScanStampedPtr& _msg);
  void cb2(ConstLaserScanStampedPtr& _msg);
  void process_scan_and_actuate(ConstLaserScanStampedPtr& _msg,
    std::vector<double>& _previous_scan);
  uint16_t _timestep;
  bool _timer;
};

} // namespace automatic_door
} // namespace building_gazebo_plugins

#endif // BUILDING_SIM_COMMON__AUTOMATIC_DOOR_COMMON_HPP
