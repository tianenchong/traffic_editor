#include "automatic_door.hpp"

using namespace building_sim_common;

namespace building_gazebo_plugins {
//==============================================================================
namespace automatic_door {

using namespace google::protobuf;
using DoorRequestPub = rclcpp::Publisher<DoorRequest>;


void AutomaticDoorPlugin::Load(gazebo::physics::ModelPtr model,
  sdf::ElementPtr sdf)
{
  auto _ros_node = gazebo_ros::Node::Get(sdf);
  _model = model;

  RCLCPP_INFO(
    _ros_node->get_logger(),
    "Loading AutomaticDoorPlugin for [%s]",
    _model->GetName().c_str());

  _door_common = DoorCommon::make(
    _model->GetName(),
    _ros_node,
    sdf);

  if (!_door_common)
    return;

  for (const auto& joint_name : _door_common->joint_names())
  {
    const auto joint = _model->GetJoint(joint_name);
    if (!joint)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        joint_name.c_str());
      return;
    }
    _joints.insert(std::make_pair(joint_name, joint));
  }

  _initialized = true;

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AutomaticDoorPlugin::on_update, this));

  RCLCPP_INFO(_ros_node->get_logger(),
    "Finished loading [%s]",
    _model->GetName().c_str());

  printf("Door Watcher created.\n");
  dwn_ptr = std::make_shared<DoorWatcherNode>();

  _main_thread = std::make_unique<std::thread>(
    &DoorWatcherNode::listener_main, this);

  _main_thread->detach();
}

void AutomaticDoorPlugin::on_update()
{
  if (!_initialized)
    return;

  const double t = _model->GetWorld()->SimTime().Double();

  // Create DoorUpdateRequest
  std::vector<DoorCommon::DoorUpdateRequest> requests;
  for (const auto& joint : _joints)
  {
    DoorCommon::DoorUpdateRequest request;
    request.joint_name = joint.first;
    request.position = joint.second->Position(0);
    request.velocity = joint.second->GetVelocity(0);
    //request.requester_id = "[door_watcher]";
    //std::cout << "[door_watcher] requested velocity "<<request.velocity<<
    //  std::endl;
    requests.push_back(request);
  }

  auto results = _door_common->update(t, requests);

  // Apply motions to the joints
  for (const auto& result : results)
  {
    const auto it = _joints.find(result.joint_name);
    assert(it != _joints.end());
    it->second->SetParam("vel", 0, result.velocity);
    it->second->SetParam("fmax", 0, result.fmax);
    //std::cout << "[door_watcher] result velocity is " << result.velocity  <<
    //  std::endl;
  }
}

void DoorWatcherNode::cb(ConstLaserScanStampedPtr& _msg)
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


    if (dwn_ptr->_previous_scan.size() != 0)
    {
      DoorRequest request;
      request.door_name = "coe_door";
      request.request_time = dwn_ptr->get_clock()->now();
      //request.requester_id = "door_watcher";
      bool fluctuation = false;
      for (unsigned int i = 0; i < count; i++)
      {
        if (trunc(current_scan[i]*1000) !=
          trunc((dwn_ptr->_previous_scan)[i]*1000))
        {
          fluctuation = true; //changes in sensor signal indicates activities
          //std::cout << "["<< i <<"] "<< current_scan[i] << " " <<
          //(dwn_ptr->_previous_scan)[i] << std::endl;
          break;
        }
      }

      //request.requested_mode.value =
      //  (fluctuation) ? DoorMode::MODE_OPEN : DoorMode::MODE_CLOSED;
      //if (dwn_ptr->_previous_request.requested_mode.value !=
      //  request.requested_mode.value) // different -> previously closed
      //{
      // std::cout << (fluctuation ? "got fluctuation" : "still") << ', ';
      // if (dwn_ptr->_previous_request.requested_mode.value ==
      //   DoorMode::MODE_CLOSED)
      //   std::cout << "mode is closed" << std::endl;
      // else if (dwn_ptr->_previous_request.requested_mode.value ==
      //   DoorMode::MODE_OPEN)
      //   std::cout << "mode is open" << std::endl;
      // else if (dwn_ptr->_previous_request.requested_mode.value ==
      //   DoorMode::MODE_MOVING)
      //   std::cout << "mode is moving" << std::endl;
      // else
      //   std::cout << "dwn_ptr is something else" << std::endl;


      if (fluctuation &&
        dwn_ptr->_previous_request.requested_mode.value !=
        DoorMode::MODE_OPEN)
      {
        //std::cout << "in fluctuation and previous request not mode_open" <<
        //  std::endl;
        request.requested_mode.value = DoorMode::MODE_OPEN;
        dwn_ptr->_door_request_pub->publish(request);
        //std::cout << "open door" << std::endl;
        dwn_ptr->_previous_request = request; // save open
      }
      else
      {
        //std::cout << "before previous request is mode_open" << std::endl;
        if (dwn_ptr->_previous_request.requested_mode.value ==
          DoorMode::MODE_OPEN)
        {
          //std::cout << "previous request is mode_open" << std::endl;
          AutomaticDoorPlugin* parent = dwn_ptr->_parent;
          std::shared_ptr<DoorCommon> door_common = parent->_door_common;
          //std::cout << "before all_doors_open" << std::endl;
          if (door_common->all_doors_open())
          {
            //std::cout << "all_doors_open" << std::endl;
            if (dwn_ptr->_timer)
            {
              dwn_ptr->_timestep++;
              if (dwn_ptr->_timestep >= 100)
              {
                request.requested_mode.value = DoorMode::MODE_CLOSED;
                dwn_ptr->_door_request_pub->publish(request);
                dwn_ptr->_timer = false;
                dwn_ptr->_timestep = 0;
                dwn_ptr->_previous_request = request; // save closed
              }
            }
            else
            {
              dwn_ptr->_timer = true;
            }
          }
        }
      }

      //rmf_door_msgs::msg::SupervisorHeartbeat msg;
      //dwn_ptr->_door_heartbeat_pub->publish(msg);
      //}

      // get current state
      /*
      if (std::abs(_current_velocity) < 1.0e-10)
      {
        if (dwn_ptr->_previous_request.requested_mode.value == DoorMode::
          MODE_OPEN)
          dwn_ptr->_current_state = OPEN;
        else
          dwn_ptr->_current_state = CLOSED;
      }
      else
      {
        dwn_ptr->_current_state = MOVING;
      }*/
      /*
      AutomaticDoorPlugin* parent = dwn_ptr->_parent;
      std::shared_ptr<DoorCommon> door_common = parent->_door_common;
      for (const auto& door : door_common->_doors)
        std::cout << "current_position: " << door.second.current_position <<
          std::endl;*/

    }
    //std::cout <<
    //  _model->GetWorld()->SimTime().Double() << '\t' << dwn_ptr->_state <<
    //  std::endl;
    dwn_ptr->_previous_scan = current_scan;
  }

}

void DoorWatcherNode::listener_main(AutomaticDoorPlugin* parent)
{
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  dwn_ptr->_parent = parent;

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe(
    "~/coe_door/coe_door_sensor_2/link/hokuyo/scan", DoorWatcherNode::cb);

  node->Subscribe("/door_states", [&](DoorRequest::UniquePtr msg)
    {
      if (msg->door_name == "coe_door")
      {
        dwn_ptr->_state = *msg;
        //std::cout << "[" << msg->door_name << "] "<< msg->requester_id << " mode: " << msg->requested_mode.value<< std::endl; //0: close, 2: open
      }
    });

  // _door_request_sub = node->Subscribe<DoorRequest>(
  //   "/door_requests", rclcpp::SystemDefaultsQoS(),
  //   [&](
  //     DoorRequest::UniquePtr msg)
  //   {
  //     if (msg->door_name == _state.door_name)
  //     {
  //       _request = *msg;
  //       //std::cout << "[" << msg->door_name << "] "<< msg->requester_id << " mode: " << msg->requested_mode.value<< std::endl; //0: close, 2: open
  //     }
  //   });

  while (true)
    gazebo::common::Time::MSleep(10);
}
}

GZ_REGISTER_MODEL_PLUGIN(automatic_door::AutomaticDoorPlugin)

} // namespace building_gazebo_plugins
