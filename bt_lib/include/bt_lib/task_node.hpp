#pragma once

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "bt_lib/action/task.hpp"

class MyActionBTNode 
  : public nav2_behavior_tree::BtActionNode<bt_lib::action::Task>
{
public:
  MyActionBTNode(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BtActionNode(name, "my_task", config) {}  // <-- action name

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("command")
    };
  }

  void on_tick() override
  {
    getInput("command", goal_.command);
  }

  BT::NodeStatus on_success() override
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override
  {
    return BT::NodeStatus::FAILURE;
  }
};