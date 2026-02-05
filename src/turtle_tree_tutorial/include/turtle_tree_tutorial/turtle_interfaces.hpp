#include <behaviortree_ros2/bt_service_node.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_relative.hpp>

// see behaviortree.dev/docs/ros2_integration for more information

class SpawnTurtle : public BT::RosServiceNode<turtlesim::srv::Spawn> {
public:
  SpawnTurtle(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosServiceNode<turtlesim::srv::Spawn>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<float>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("theta")});
  }

  bool setRequest(Request::SharedPtr& request) override {
    getInput("x", request->x);
    getInput("y", request->y);
    getInput("theta", request->theta);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
    RCLCPP_INFO(logger(), "Spawned turtle with name: %s", response->name.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class MoveTurtleRelative : public BT::RosServiceNode<turtlesim::srv::TeleportRelative> {
public:
  MoveTurtleRelative(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosServiceNode<turtlesim::srv::TeleportRelative>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<float>("angular"), BT::InputPort<float>("linear")});
  }

  bool setRequest(Request::SharedPtr& request) override {
    getInput("angular", request->angular);
    getInput("linear", request->linear);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
    // RCLCPP_INFO(logger(), "Spawned turtle with name: %s", response->.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

