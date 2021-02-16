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

  RCLCPP_INFO(_ros_node->get_logger(),
    "Finished loading [%s]",
    _model->GetName().c_str());

  _main_thread = std::thread(&DoorWatcherNode::init_and_make, this);
  _main_thread.detach();

  printf("Door Watcher created.\n");
  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&AutomaticDoorPlugin::on_update, this));
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

    ms.set_velocity_sign(request.velocity);


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
  }
}

void DoorWatcherNode::init_and_make(AutomaticDoorPlugin* parent)
{
  dwn_ptr = std::make_shared<DoorWatcherNode>();
  dwn_ptr->_parent = parent;
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  const auto& name = _model->GetName();
  gazebo::transport::SubscriberPtr sub1 = node->Subscribe(
    "~/"+name+"/"+name+"_sensor_1/link/hokuyo/scan",
    DoorWatcherNode::cb1);
  gazebo::transport::SubscriberPtr sub2 = node->Subscribe(
    "~/"+name+"/"+name+"_sensor_2/link/hokuyo/scan",
    DoorWatcherNode::cb2);
  dwn_ptr->_door_state_sub = dwn_ptr->create_subscription<DoorState>(
    "door_states", rclcpp::SystemDefaultsQoS(),
    [&](DoorState::UniquePtr msg)
    {
      if (msg->door_name == name)
        dwn_ptr->_state = *msg;
    });
  dwn_ptr->_last_fluctuation = false;
  rclcpp::spin(dwn_ptr);
  rclcpp::shutdown();
}

void DoorWatcherNode::cb1(ConstLaserScanStampedPtr& _msg)
{
  process_scan_and_actuate(_msg, dwn_ptr->_previous_scan_1);
}

void DoorWatcherNode::cb2(ConstLaserScanStampedPtr& _msg)
{
  process_scan_and_actuate(_msg, dwn_ptr->_previous_scan_2);
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
      request.door_name = _model->GetName();
      request.request_time = dwn_ptr->get_clock()->now();
      //request.requester_id = "door_watcher";
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

      int sign = dwn_ptr->_parent->ms.get_majority_sign();
      if (fluctuation && !dwn_ptr->_last_fluctuation &&
        ((dwn_ptr->_state.current_mode.value == DoorMode::MODE_CLOSED &&
        sign == 0) ||
        (dwn_ptr->_state.current_mode.value == DoorMode::MODE_MOVING &&
        sign == 1))) // leading edge fluctuation and  close (sign 0 to double check closed to avoid timing mismatch) or closing
      {
        request.requested_mode.value = DoorMode::MODE_OPEN;
        dwn_ptr->_door_request_pub->publish(request);
        std::cout << "[door watcher] opening door ... " <<
        (fluctuation ? "GOT FLUCTUATION " : "NO FLUCTUATION ") <<
        ((dwn_ptr->_state.current_mode.value ==
          DoorMode::MODE_CLOSED) ? "CLOSED " : "MOVING ")<< sign << std::endl;
      }
      else
      {
        if (dwn_ptr->_state.current_mode.value == DoorMode::MODE_OPEN)
        {
          if (fluctuation)
          {
            dwn_ptr->_timer = false;
            dwn_ptr->_timestep = 0;
          }
          else if (dwn_ptr->_timer)
          {
            dwn_ptr->_timestep++;
            if (dwn_ptr->_timestep >= 100)
            {
              request.requested_mode.value = DoorMode::MODE_CLOSED;
              dwn_ptr->_door_request_pub->publish(request);
              std::cout << "[door watcher] closing door ... " << std::endl;
              dwn_ptr->_timer = false;
              dwn_ptr->_timestep = 0;
            }
          }
          else
          {
            dwn_ptr->_timer = true;
          }
        }
      }
      dwn_ptr->_last_fluctuation = fluctuation;
    }
    _previous_scan = current_scan;
  }
}

}

GZ_REGISTER_MODEL_PLUGIN(automatic_door::AutomaticDoorPlugin)

} // namespace building_gazebo_plugins
