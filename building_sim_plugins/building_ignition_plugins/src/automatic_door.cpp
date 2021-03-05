#include "automatic_door.hpp"

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_ignition_plugins {

//==============================================================================


void AutomaticDoorPlugin::create_entity_components(Entity entity,
  EntityComponentManager& ecm)
{
  if (!ecm.EntityHasComponentType(entity,
    components::JointPosition().TypeId()))
    ecm.CreateComponent(entity, components::JointPosition({0}));
  if (!ecm.EntityHasComponentType(entity,
    components::JointVelocity().TypeId()))
    ecm.CreateComponent(entity, components::JointVelocity({0}));
  if (!ecm.EntityHasComponentType(entity,
    components::JointVelocityCmd().TypeId()))
    ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
}


void AutomaticDoorPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager& /*_eventMgr*/)
{
  //_ros_node = gazebo_ros::Node::Get(sdf);
  // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
  // TODO proper rclcpp init (only once and pass args)
  auto model = Model(entity);
  char const** argv = NULL;
  auto door_ele = sdf->GetElementImpl("door");
  get_sdf_attribute_required<std::string>(door_ele, "name", _name);
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);
  std::string plugin_name("plugin_" + _name);
  ignwarn << "Initializing plugin with name " << plugin_name <<
    std::endl;
  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

  RCLCPP_INFO(_ros_node->get_logger(),
    "Loading AutomaticDoorPlugin for [%s]",
    _name.c_str());

  _door_common = DoorCommon::make(
    _name,
    _ros_node,
    sdf);

  if (!_door_common)
    return;

  for (const auto& joint_name : _door_common->joint_names())
  {
    const auto joint = model.JointByName(ecm, joint_name);
    if (!joint)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        joint_name.c_str());
      return;
    }
    create_entity_components(joint, ecm);
    _joints.insert({joint_name, joint});
  }

  _initialized = true;

  RCLCPP_INFO(_ros_node->get_logger(),
    "Finished loading [%s]",
    _name.c_str());

  _main_thread = std::thread(&DoorWatcherNode::init_and_make, this);
  _main_thread.detach();
}

void DoorWatcherNode::init_and_make(AutomaticDoorPlugin* parent)
{
  dwn_ptr = std::make_shared<DoorWatcherNode>();
  dwn_ptr->_parent = parent;
  std::string lidar1_scan_topic =
    "/world/sim_world/model/"+parent->_name+"/link/"+parent->_name+
    "_sensor_1::link/sensor/lidar_2d_v1/scan";
  if (!parent->_ign_node.Subscribe(
      lidar1_scan_topic,
      &DoorWatcherNode::cb1))
  {
    std::cerr <<
      "Error subscribing to topic [" << lidar1_scan_topic <<"]" << std::endl;
  }
  std::string lidar2_scan_topic =
    "/world/sim_world/model/"+parent->_name+"/link/"+parent->_name+
    "_sensor_2::link/sensor/lidar_2d_v1/scan";
  if (!parent->_ign_node.Subscribe(
      lidar2_scan_topic,
      &DoorWatcherNode::cb2))
  {
    std::cerr <<
      "Error subscribing to topic [" << lidar2_scan_topic <<"]" << std::endl;
  }
  dwn_ptr->_door_state_sub = dwn_ptr->create_subscription<DoorState>(
    "door_states", rclcpp::SystemDefaultsQoS(),
    [&](DoorState::UniquePtr msg)
    {
      if (msg->door_name == parent->_name)
        dwn_ptr->_state = *msg;
    });
  dwn_ptr->_last_fluctuation = false;
  rclcpp::spin(dwn_ptr);
  rclcpp::shutdown();
}

void DoorWatcherNode::cb1(const ignition::msgs::LaserScan& msg)
{
  process_scan_and_actuate(msg, dwn_ptr->_previous_scan_1);
}

void DoorWatcherNode::cb2(const ignition::msgs::LaserScan& msg)
{
  process_scan_and_actuate(msg, dwn_ptr->_previous_scan_2);
}

void DoorWatcherNode::process_scan_and_actuate(
  const ignition::msgs::LaserScan& msg,
  std::vector<double>& _previous_scan)
{
  const auto* descriptor = msg.descriptor();
  const auto* ranges_field = descriptor->FindFieldByName("ranges");
  assert(ranges_field != nullptr);
  assert(ranges_field->type() == FieldDescriptor::TYPE_DOUBLE);
  assert(ranges_field->label() == FieldDescriptor::LABEL_REPEATED);
  const auto* reflection = msg.GetReflection();
  auto count = msg.count();
  const auto* scan_msg =
    dynamic_cast<const Message*>(&msg);

  std::vector<double> current_scan = {};
  for (unsigned int i = 0; i < count; i++)
  {
    current_scan.push_back(std::move(reflection->GetRepeatedDouble(*scan_msg,
      ranges_field, i)));
  }

  if (_previous_scan.size() != 0)
  {
    DoorRequest request;
    request.door_name = dwn_ptr->_parent->_name;
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

void AutomaticDoorPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  // TODO parallel thread executor?
  rclcpp::spin_some(_ros_node);
  if (!_initialized)
    return;

  double t =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
    count()) * 1e-9;

  // Create DoorUpdateRequest
  std::vector<DoorCommon::DoorUpdateRequest> requests;
  for (const auto& joint : _joints)
  {
    DoorCommon::DoorUpdateRequest request;
    request.joint_name = joint.first;
    request.position = ecm.Component<components::JointPosition>(
      joint.second)->Data()[0];
    request.velocity = ecm.Component<components::JointVelocity>(
      joint.second)->Data()[0];
    requests.push_back(request);
  }

  auto results = _door_common->update(t, requests);

  // Apply motions to the joints
  for (const auto& result : results)
  {
    const auto it = _joints.find(result.joint_name);
    assert(it != _joints.end());
    auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
      it->second);
    vel_cmd->Data()[0] = result.velocity;
  }
}

IGNITION_ADD_PLUGIN(
  AutomaticDoorPlugin,
  System,
  AutomaticDoorPlugin::ISystemConfigure,
  AutomaticDoorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AutomaticDoorPlugin, "automatic_door")
} // namespace building_ignition_plugins
