#pragma once

#include "behaviortree_cpp/action_node.h"

class TalkerActionNode : public BT::SyncActionNode {
public:
  TalkerActionNode(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
