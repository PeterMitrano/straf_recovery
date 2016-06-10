#include "straf_recovery/straf_recovery.h"
#include <pluginlib/class_list_macros.h>

namespace straf_recovery {
StrafRecovery::StrafRecovery(): initialized_(false) {}

void StrafRecovery::initialize(std::string name, tf::TransformListener *tf,
                               costmap_2d::Costmap2DROS *global_costmap,
                               costmap_2d::Costmap2DROS *local_costmap) {
  if (initialized_){
    ROS_ERROR("thou shall not call initialize twice on this object. Doing nothing.\n");
    return;
  }

  initialized_ = true;

  name_ = name;
  tf_ = tf;
  global_costmap_ = global_costmap;
  local_costmap_ = local_costmap;
  local_costmap_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

  //get some parameters from the parameter server
  ros::NodeHandle private_nh("~/" + name_);
  ros::NodeHandle base_local_planner_nh("~/TrajectoryPlannerROS");

  //use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);
}

void StrafRecovery::runBehavior() {
  if (!initialized_){
    ROS_ERROR("thou shall not fail to call initialize! Doing nothing.\n");
    return;
  }

  if (local_costmap_ == NULL || global_costmap_ == NULL){
    ROS_ERROR("Thou shall not pass costmaps to the straf recovery which are nullptr. Doing nothing\n");
    return;
  }

  ROS_WARN("Straf recovery behavior started.\n");

  ros::NodeHandle n;
  ros::Rate r(frequency_);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  while (n.ok()) {
    local_costmap_->getRobotPose(global_pose);
  }

}

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
