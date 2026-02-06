#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#ifndef BEHAVIOR_TREE_DIR
#define BEHAVIOR_TREE_DIR "/home/ros2/.devcontainer/src/turtle_tree_tutorial/behavior_trees/"
#endif

#include "turtle_tree_tutorial/TalkerActionNode.hpp"
#include "turtle_tree_tutorial/delay_node.hpp"
#include "turtle_tree_tutorial/turtle_interfaces.hpp"

int main(int argc, char** argv) {
  std::string tree_file = "example.xml";

  if (argc > 1) {
    tree_file = argv[1];
    std::cout << "Using user-defined tree file " << tree_file << std::endl;
  } else {
    std::cout << "Using default tree file " << tree_file << std::endl;
  }

  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;

  // register standard tree nodes from defined classes
  factory.registerNodeType<TalkerActionNode>("Talker");
  factory.registerNodeType<BT::PatchedDelayNode>("PatchedDelay");

  // simple actions can be registered with any lambda function that takes a reference to a tree node and returns BT::NodeStatus
  factory.registerSimpleAction("HelloWorld", [&](BT::TreeNode&) {
    std::cout << "Hello world" << std::endl;
    return BT::NodeStatus::SUCCESS;
  });

  // use this node as a service and action client for behavior nodes
  auto node = std::make_shared<rclcpp::Node>("turtle_client");

  // register ROS nodes
  BT::RosNodeParams params;
  params.nh = node;
  params.default_port_value = "/spawn";
  factory.registerNodeType<SpawnTurtle>("SpawnTurtle", params);

  params.default_port_value = "/reset";
  factory.registerNodeType<ResetTurtle>("ResetTurtle", params);

  // the default name of the service can be overridden with the "service_name" input port to target a different turtle
  params.default_port_value = "/turtle1/teleport_relative";
  factory.registerNodeType<MoveTurtleRelative>("MoveTurtleRelative", params);

  // the default name of the service can be overridden with the "service_name" input port to target a different turtle
  params.default_port_value = "/turtle1/teleport_absolute";
  factory.registerNodeType<MoveTurtleAbsolute>("MoveTurtleAbsolute", params);

  params.default_port_value =
      "/turtle1/pose";// default name of the pose subscription topic, can be overridden with the "topic_name" input port
  factory.registerNodeType<CheckTurtlePose>("CheckTurtlePose", params);

  params.default_port_value =
      "/turtle1/cmd_vel";// default name of the command topic, can be overridden with the "topic_name" input port
  factory.registerNodeType<CommandTurtleSwim>("CommandTurtleSwim", params);

  // load a tree from XML file
  auto tree = factory.createTreeFromFile(std::string(BEHAVIOR_TREE_DIR) + tree_file);

  // run the tree
  tree.tickWhileRunning();

  return 0;
}
