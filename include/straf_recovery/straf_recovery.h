#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>

namespace straf_recovery {

class StrafRecovery : public nav_core::RecoveryBehavior {
 public:
  StrafRecovery();
  void initialize(std::string, tf::TransformListener *tf,
                  costmap_2d::Costmap2DROS *global_costmap,
                  costmap_2d::Costmap2DROS *local_costmap);
  void runBehavior();

 private:
  bool initialized_;
  std::string name_;
  tf::TransformListener* tf_;
  costmap_2d::Costmap2DROS* local_costmap_;
  costmap_2d::Costmap2DROS* global_costmap_;
  base_local_planner::CostmapModel* local_costmap_model_;
  double frequency_;
};

}
