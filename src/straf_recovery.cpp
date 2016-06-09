#include "straf_recovery/straf_recovery.h"
#include <pluginlib/class_list_macros.h>

namespace straf_recovery {
StrafRecovery::StrafRecovery() {}

void StrafRecovery::runBehavior() {
  ROS_INFO("running the recovery behavior");
}

void StrafRecovery::initialize(std::string name, tf::TransformListener *tf,
                               costmap_2d::Costmap2DROS *global_costmap,
                               costmap_2d::Costmap2DROS *local_costmap) {
  ROS_INFO("initializing the recovery behavior");
}

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
