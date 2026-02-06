#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/srv/teleport_relative.hpp>

// see behaviortree.dev/docs/ros2_integration for more information on the BT::RosServiceNode wrapper

/**
 * BehaviorTree Action Node that calls the "reset" service (no inputs) which clears the screen and sets the turtle to 0,0
 */
class ResetTurtle : public BT::RosServiceNode<std_srvs::srv::Empty> {
public:
  ResetTurtle(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosServiceNode<std_srvs::srv::Empty>(name, conf, params) {}

  bool setRequest(Request::SharedPtr&) override { return true; }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr&) override { return BT::NodeStatus::SUCCESS; }
};

/**
 * BehaviorTree Action Node that calls the "spawn" service to add a turtle to the screen at an initial location
 * 
 * Input ports:
 *   - x: the target x position to teleport
 *   - y: the target y position to teleport
 *   - theta: the target theta angle to teleport (radians)
 * 
 * Output ports:
 *   - new_name: the name of the spawned turtle
 * 
 */
class SpawnTurtle : public BT::RosServiceNode<turtlesim::srv::Spawn> {
public:
  SpawnTurtle(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosServiceNode<turtlesim::srv::Spawn>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<float>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("theta"),
         BT::OutputPort<std::string>("new_name")});
  }

  bool setRequest(Request::SharedPtr& request) override {
    getInput("x", request->x);
    getInput("y", request->y);
    getInput("theta", request->theta);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override {
    RCLCPP_INFO(logger(), "Spawned turtle with name: %s", response->name.c_str());
    setOutput("new_name", response->name);
    return BT::NodeStatus::SUCCESS;
  }
};

/**
 * BehaviorTree Action Node that calls the "teleport absolute" service to set the turtle pose
 * 
 * Input ports:
 *   - x: the target x position to teleport
 *   - y: the target y position to teleport
 *   - theta: the target theta angle to teleport (radians)
 * 
 *   - service_name: if specified, can be used to change the target service (for example, to move a different turtle)
 * 
 */
class MoveTurtleAbsolute : public BT::RosServiceNode<turtlesim::srv::TeleportAbsolute> {
public:
  MoveTurtleAbsolute(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosServiceNode<turtlesim::srv::TeleportAbsolute>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<float>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("theta")});
  }

  bool setRequest(Request::SharedPtr& request) override {
    getInput("x", request->x);
    getInput("y", request->y);
    getInput("theta", request->theta);
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr&) override { return BT::NodeStatus::SUCCESS; }
};

/**
 * BehaviorTree Action Node that calls the "teleport relative" service to set the turtle pose based on its current pose
 * 
 * Input ports:
 *   - linear: the distance to move the turtle from its current position in the direction it is facing
 *   - angular: the amount to rotate the turtle from its current angle after moving it (radians)
 * 
 *   - service_name: if specified, can be used to change the target service (for example, to move a different turtle)
 */
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

  BT::NodeStatus onResponseReceived(const Response::SharedPtr&) override { return BT::NodeStatus::SUCCESS; }
};

/**
 * BehaviorTree Condition node based on a ROS publisher to send one twist message to the turtle.
 * The twist is the combined linear and angular velocity, where in this case we care about the relative
 * x velocity of the turtle and the angular velocity which is around the z axis.
 * 
 * Input ports:
 *   - speed: the desired forward velocity of the swim stroke
 *   - turn: the desired angular velocity (radians/second)
 * 
 *   - topic_name: if specified, can be used to change the target topic (for example, to move a different turtle)
 * 
 * Condition nodes only tick once and return either SUCCESS or FAILURE. The RosTopicPubNode wrapper
 * simply triggers a publication based on input ports.
 */
class CommandTurtleSwim : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
  CommandTurtleSwim(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<float>("speed"), BT::InputPort<float>("turn")});
  }

  bool setMessage(geometry_msgs::msg::Twist& twist) override {
    BT::Expected<float> x = this->getInput<float>("speed");
    BT::Expected<float> theta = this->getInput<float>("turn");

    twist.linear.x = x ? x.value() : 0;
    twist.angular.z = theta ? theta.value() : 0;
    return true;
  }
};

/**
 * BehaviorTree Condition node based on a ROS subscriber to check if the last pose is within range of a specific target pose.
 * 
 * Input ports:
 *   - x: the target x position
 *   - y: the target y position
 *   - theta: the target theta angle (radians)
 *   - linear_distance: the maximum distance between actual position and target position that would count as SUCCESS (in range)
 *   - angular_distance: the maximum distance between actual angle and target angle (in radians) that would count as SUCCESS (in range)
 * 
 *   - topic_name: if specified, can be used to change the target topic (for example, to check a different turtle)
 * 
 * Condition nodes only tick once and return either SUCCESS or FAILURE. The RosTopicSubNode wrapper
 * allows the node to query the last message on the topic and make a conditional check on the value.
 */
class CheckTurtlePose : public BT::RosTopicSubNode<turtlesim::msg::Pose> {
public:
  CheckTurtlePose(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
      : BT::RosTopicSubNode<turtlesim::msg::Pose>(name, conf, params) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<float>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("theta"),
         BT::InputPort<float>("linear_distance"), BT::InputPort<float>("angular_distance")});
  }

  BT::NodeStatus onTick(const std::shared_ptr<turtlesim::msg::Pose>& pose) override {
    if (!pose) {
      RCLCPP_WARN(logger(), "[%s] no pose subscription was received!", name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
        logger(), "[%s] turtle pose [x, y, theta]: [%.2f, %.2f, %.2f]", name().c_str(), pose->x, pose->y, pose->theta);

    BT::Expected<float> target_x = this->getInput<float>("x");
    BT::Expected<float> target_y = this->getInput<float>("y");
    BT::Expected<float> target_theta = this->getInput<float>("theta");

    auto linear_error = sqrt(pow(target_x.value() - pose->x, 2) + pow(target_y.value() - pose->y, 2));
    auto angular_error = target_theta.value() - pose->theta;

    RCLCPP_INFO(
        logger(), "[%s] turtle pose distance to target [linear, angular]: [%.2f, %.2f]", name().c_str(), linear_error,
        angular_error);

    bool in_range = true;
    if (linear_error > this->getInput<float>("linear_distance").value()) {
      RCLCPP_WARN(logger(), "[%s] turtle pose is out of linear range!", name().c_str());
      in_range = false;
    }
    if (angular_error > this->getInput<float>("angular_distance").value()) {
      RCLCPP_WARN(logger(), "[%s] turtle pose is out of angular range!", name().c_str());
      in_range = false;
    }

    return in_range ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};