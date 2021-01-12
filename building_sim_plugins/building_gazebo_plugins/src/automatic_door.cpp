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
class DoorWatcherNode;
typedef std::shared_ptr<DoorWatcherNode> _DWN_PTR;
_DWN_PTR dwn_ptr = nullptr;
gazebo::physics::ModelPtr _model;
double _current_velocity;
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
  std::vector<double> _previous_scan;
  DoorRequestPub::SharedPtr _door_request_pub;
  enum current_state { OPEN, MOVING, CLOSED} _current_state;
  enum last_requested_state { OPEN, CLOSED} _last_requested_state;
  enum transition_state { JUST_OPENED, JUST_CLOSED} _transition_state;
  double t_since_transition_state;

  DoorWatcherNode()
  : rclcpp::Node("door_watcher")
  {
    const auto default_qos = rclcpp::SystemDefaultsQoS();
    _door_request_pub = create_publisher<DoorRequest>(
      FinalDoorRequestTopicName, default_qos);
    _door_heartbeat_pub = create_publisher<Heartbeat>(
      DoorSupervisorHeartbeatTopicName, default_qos);
  }
  ~DoorWatcherNode() {}
  static void cb(ConstLaserScanStampedPtr& _msg)
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
        request.requester_id = "door_watcher";
        bool same = true;
        for (unsigned int i = 0; i < count; i++)
        {
          if (trunc(current_scan[i]*1000) !=
            trunc((dwn_ptr->_previous_scan)[i]*1000))
          {
            same = false; //changes in sensor signal indicates activities
            //std::cout << "["<< i <<"] "<< current_scan[i] << " " <<
            //(dwn_ptr->_previous_scan)[i] << std::endl;
            break;
          }
        }

        request.requested_mode.value =
          (same) ? DoorMode::MODE_CLOSED : DoorMode::
          MODE_OPEN;
        if (dwn_ptr->_previous_request.requested_mode.value !=
          request.requested_mode.value) // different -> previously closed
        {
          dwn_ptr->_door_request_pub->publish(request);
          //rmf_door_msgs::msg::SupervisorHeartbeat msg;
          //dwn_ptr->_door_heartbeat_pub->publish(msg);
        }
        dwn_ptr->_previous_request = request; // save open

        // get current state
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
        }

      }
      //std::cout <<
      //  _model->GetWorld()->SimTime().Double() << '\t' << dwn_ptr->_state <<
      //  std::endl;
      dwn_ptr->_previous_scan = current_scan;
    }

  }
  static void listener_main()
  {
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe(
      "~/coe_door/coe_door_sensor_2/link/hokuyo/scan", DoorWatcherNode::cb);

    while (true)
      gazebo::common::Time::MSleep(10);
  }

};

class AutomaticDoorPlugin : public gazebo::ModelPlugin
{
  using DoorRequestPub = rclcpp::Publisher<DoorRequest>;
private:
  gazebo::event::ConnectionPtr _update_connection;
  std::unordered_map<std::string, gazebo::physics::JointPtr> _joints;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

public:
  AutomaticDoorPlugin()
  {
    // Do nothing
  }

  ~AutomaticDoorPlugin()
  {
    if (_main_thread->joinable())
      _main_thread->join();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
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
      &DoorWatcherNode::listener_main);

    _main_thread->detach();
  }

private:
  std::unique_ptr<std::thread> _main_thread;

  void on_update()
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
      request.velocity = _current_velocity = joint.second->GetVelocity(0);
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

};
}

GZ_REGISTER_MODEL_PLUGIN(automatic_door::AutomaticDoorPlugin)

} // namespace building_gazebo_plugins
