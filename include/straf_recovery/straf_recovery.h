#pragma once

#include "straf_recovery/StrafRecoveryConfig.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace straf_recovery
{
class StrafRecovery : public nav_core::RecoveryBehavior
{
public:
  StrafRecovery();
  void initialize(std::string, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap,
                  costmap_2d::Costmap2DROS* local_costmap);

  /** \brief This recovery behavior will translate away from the nearest obstacle in the local costmap.
   * This is done by finding the nearest points, calculating a tangent, and then finding the normal to that tangent.
   * The robot moves along that tangent line until all of the following conditions are true:
   *  - The robot has moved at least the distance specified by min_distance (via parameter server)
   *  - The robot has moved no more than the distance specified by max_distance (via parameter server)
   *  - The robot is able to spin freely (IE, rotate recovery would work now)
   *  - The robot is no closer to any other obstacle than it was when it began translating
   */
  void runBehavior();

private:
  bool initialized_;
  bool enabled_;
  double frequency_;
  double maximum_translate_distance_;
  double minimum_translate_distance_;
  double go_to_goal_distance_threshold_;
  double xy_goal_tolerance_;
  double vel_;
  int timeout_;  // in seconds
  int cycles_; // track how many times we've run for detecting failure?
  base_local_planner::CostmapModel* local_costmap_model_;
  dynamic_reconfigure::Server<StrafRecoveryConfig>* dsrv_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2DROS* global_costmap_;
  geometry_msgs::PoseStamped last_goal_;
  ros::Publisher cycles_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher vel_pub_;
  ros::Subscriber goal_sub_;
  std::string name_;
  tf::TransformListener* tf_;

  /** @brief straf in the direction of a point, given in the odom frame */
  void strafInDiretionOfPose(tf::Stamped<tf::Pose> current_pose, tf::Vector3 direction_pose, bool away=true);

  /** @brief uses move_base_simple/goal */
  void goalCallback(const geometry_msgs::PoseStamped& msg);

  /** @brief callback for reconfigure */
  void reconfigureCB(StrafRecoveryConfig& config, uint32_t level);

};
}
