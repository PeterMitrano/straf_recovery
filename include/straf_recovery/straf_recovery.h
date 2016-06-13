#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>

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
  std::string name_;
  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2DROS* global_costmap_;
  base_local_planner::CostmapModel* local_costmap_model_;
  ros::Publisher obstacle_pub_;
  double frequency_;
  int timeout_; //in seconds
  double maximum_translate_distance_;
  double minimum_translate_distance_;
  double vel_;

  /** \return true if the robot can safely rotate
   * \param x needs to be in the map frame
   * \param y needs to be in the map frame
   * \param theta needs to be in the map frame
   */
  bool canRotateInPlace(double robot_map_x, double robot_map_y, double theta, tf::Stamped<tf::Pose> global_pose);

};

}
