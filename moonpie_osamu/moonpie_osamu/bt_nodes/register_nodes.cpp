#include "behaviortree_cpp_v3/bt_factory.h"
// Include all node headers
#include "wait_for_start_command.cpp"
#include "find_aruco.cpp"
#include "dig.cpp"
#include "dump.cpp"
#include "wait_for_stop_command.cpp"
#include "scan_dump_zone.cpp"
#include "backward_navigate_to_pose.cpp"
#include "set_dump_goal.cpp"

void RegisterCustomNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<WaitForStartCommand>("WaitForStartCommand");
  factory.registerNodeType<FindAruco>("FindAruco");
  factory.registerNodeType<Dig>("Dig");
  factory.registerNodeType<Dump>("Dump");
  factory.registerNodeType<WaitForStopCommand>("WaitForStopCommand");
  factory.registerNodeType<ScanDumpZone>("ScanDumpZone");
  factory.registerNodeType<BackwardNavigateToPose>("BackwardNavigateToPose");
  factory.registerNodeType<SetDumpGoal>("SetDumpGoal");
} 