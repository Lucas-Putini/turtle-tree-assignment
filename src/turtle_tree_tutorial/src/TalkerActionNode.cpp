#include "turtle_tree_tutorial/TalkerActionNode.hpp"

TalkerActionNode::TalkerActionNode(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}

BT::PortsList TalkerActionNode::providedPorts() {
    return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus TalkerActionNode::tick() {
    BT::Expected<std::string> msg = this->getInput<std::string>("message");
    if (msg) {
        std::cout << "Talker node: " << msg.value() << std::endl;
    } else {
        throw BT::RuntimeError("Missing required input [message]: ", msg.error());
    }
    return BT::NodeStatus::SUCCESS;
}