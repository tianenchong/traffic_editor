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


#include "automatic_door_common.hpp"

using namespace building_sim_common;

namespace building_gazebo_plugins {
//==============================================================================
namespace automatic_door {

void DoorWatcherNode::cb1(ConstLaserScanStampedPtr& _msg)
{
  process_scan_and_actuate(_msg, _previous_scan_1);
}

void DoorWatcherNode::cb2(ConstLaserScanStampedPtr& _msg)
{
  process_scan_and_actuate(_msg, _previous_scan_2);
}

void DoorWatcherNode::process_scan_and_actuate(ConstLaserScanStampedPtr& _msg,
  std::vector<double>& _previous_scan)
{
  // Dump the message contents to stdout.
  //const std::string data;
  if (_msg->has_scan())
  {
    //std::cout << _msg->DebugString();
    const auto scan = _msg->scan();
    const auto* descriptor = scan.GetDescriptor();
    const auto* ranges_field = descriptor->FindFieldByName("ranges");
    assert(ranges_field != nullptr);
    assert(ranges_field->type() == FieldDescriptor::TYPE_DOUBLE);
    assert(ranges_field->label() == FieldDescriptor::LABEL_REPEATED);
    const auto* reflection = scan.GetReflection();
    auto count = scan.count();
    const auto* scan_msg =
      dynamic_cast<const Message*>(&scan);

    std::vector<double> current_scan = {};
    for (unsigned int i = 0; i < count; i++)
    {
      current_scan.push_back(std::move(reflection->GetRepeatedDouble(*scan_msg,
        ranges_field, i)));
    }


    if (_previous_scan.size() != 0)
    {
      DoorRequest request;
      request.door_name = _parent->_model->GetName();
      request.request_time = get_clock()->now();
      //request.requester_id = "door_watcher
      bool fluctuation = false;
      for (unsigned int i = 0; i < count; i++)
      {
        if (trunc(current_scan[i]*100) !=
          trunc((_previous_scan)[i]*100))
        {
          fluctuation = true; //changes in sensor signal indicates activities
          break;
        }
      }

      int sign = _parent->ms.get_majority_sign();
      if (fluctuation && !_last_fluctuation &&
        ((_state.current_mode.value == DoorMode::MODE_CLOSED &&
        sign == 0) ||
        (_state.current_mode.value == DoorMode::MODE_MOVING &&
        sign == 1))) // leading edge fluctuation and  close (sign 0 to double check closed to avoid timing mismatch) or closing
      {
        request.requested_mode.value = DoorMode::MODE_OPEN;
        _door_request_pub->publish(request);
      }
      else
      {
        if (_state.current_mode.value == DoorMode::MODE_OPEN)
        {
          if (fluctuation)
          {
            _timer = false;
            _timestep = 0;
          }
          else if (_timer)
          {
            _timestep++;
            if (_timestep >= 100)
            {
              request.requested_mode.value = DoorMode::MODE_CLOSED;
              _door_request_pub->publish(request);
              _timer = false;
              _timestep = 0;
            }
          }
          else
          {
            _timer = true;
          }
        }
      }
      _last_fluctuation = fluctuation;
    }
    _previous_scan = current_scan;
  }
}


std::shared_ptr<DoorWatcherNode> DoorWatcherNode::init_and_make(
  AutomaticDoorPlugin* parent)
{
  std::cout << "here....." << std::endl;
  parent->_dwn_ptr = std::make_shared<DoorWatcherNode>(
    parent->_model->GetName()+"_watcher_node");

  parent->_dwn_ptr->_parent = parent;
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  const auto& name = parent->_model->GetName();
  auto dwn_ptr = parent->_dwn_ptr.get();
  gazebo::transport::SubscriberPtr sub1 = node->Subscribe(
    " ~/"+name+"/"+name+"_sensor_1/link/hokuyo/scan",
    &DoorWatcherNode::cb1, dwn_ptr);
  gazebo::transport::SubscriberPtr sub2 = node->Subscribe(
    " ~/"+name+"/"+name+"_sensor_2/link/hokuyo/scan",
    &DoorWatcherNode::cb2, dwn_ptr);
  parent->_dwn_ptr->_door_state_sub =
    parent->_dwn_ptr->create_subscription<DoorState>(
    "door_states", rclcpp::SystemDefaultsQoS(),
    [&](DoorState::UniquePtr msg)
    {
      if (msg->door_name == name)
        parent->_dwn_ptr->_state = *msg;
    });
  parent->_dwn_ptr->_last_fluctuation = false;
  rclcpp::spin(parent->_dwn_ptr);
  rclcpp::shutdown();
}

} // namespace automatic_door
} // namespace building_gazebo_plugins
