#include "bt_lib/task_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<MyActionBTNode>("MyAction");
}