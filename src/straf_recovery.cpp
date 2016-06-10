#include "straf_recovery/straf_recovery.h"
#include <cmath>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace straf_recovery
{

StrafRecovery::StrafRecovery() : initialized_(false)
{
}

void StrafRecovery::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *global_costmap,
                               costmap_2d::Costmap2DROS *local_costmap)
{
  if (initialized_)
  {
    ROS_ERROR("thou shall not call initialize twice on this object. Doing "
              "nothing.\n");
    return;
  }

  initialized_ = true;

  name_ = name;
  tf_ = tf;
  global_costmap_ = global_costmap;
  local_costmap_ = local_costmap;
  local_costmap_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

  // get some parameters from the parameter server
  ros::NodeHandle private_nh("~/" + name_);
  ros::NodeHandle base_local_planner_nh("~/TrajectoryPlannerROS");

  private_nh.param("minimum_translate_distance", minimum_translate_distance_, 1.0);
  private_nh.param("maximum_translate_distance", maximum_translate_distance_, 5.0);
  private_nh.param("min_vel", min_vel_, 0.0);
  private_nh.param("max_vel", max_vel_, 0.5);

  // use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);

  //for visualizing
  straf_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("straf_direction", 10);

  ROS_WARN("Initializing StrafRecovery");
  ROS_WARN("minimum_translate_distance %f", minimum_translate_distance_);
  ROS_WARN("maximum_translate_distance %f", maximum_translate_distance_);
  ROS_WARN("min_vel %f", min_vel_);
  ROS_WARN("max_vel %f", max_vel_);

  // use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);
}

void StrafRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("thou shall not fail to call initialize! Doing nothing.\n");
    return;
  }

  if (local_costmap_ == NULL || global_costmap_ == NULL)
  {
    ROS_ERROR("Thou shall not pass costmaps to the straf recovery which are nullptr. "
              "Doing nothing\n");
    return;
  }

  ROS_WARN("Straf recovery behavior started.\n");

  ros::NodeHandle n;
  ros::Rate r(frequency_);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = -M_PI;
  double start_offset = -angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  bool current_distance_translated = 0.0;

  // find the nearest obstacle
  double robot_x = global_pose.getOrigin().x();
  double robot_y = global_pose.getOrigin().y();

  unsigned int robot_map_x, robot_map_y;
  local_costmap_->getCostmap()->worldToMap(robot_x, robot_y, robot_map_x, robot_map_y);

  ROS_WARN("robot pos: %f,%f", robot_x, robot_y);

  ClosestObstacle initial_nearest_obstacle = nearestObstacle(local_costmap_, robot_map_x, robot_map_y);

  double obstacle_world_x;
  double obstacle_world_y;
  local_costmap_->getCostmap()->mapToWorld(initial_nearest_obstacle.x, initial_nearest_obstacle.y, obstacle_world_x, obstacle_world_y);

  ROS_WARN("obstacle pos: %f,%f", obstacle_world_x, obstacle_world_y);

  geometry_msgs::PoseStamped tmp;
  tmp.header.frame_id = global_pose.frame_id_;
  tmp.header.stamp = ros::Time::now();
  tmp.pose.position.x = robot_x;
  tmp.pose.position.y = robot_y;
  tmp.pose.orientation.x = 0;
  tmp.pose.orientation.y = initial_nearest_obstacle.dist;
  straf_pub_.publish(tmp);

  geometry_msgs::PoseStamped straf_dir;
  straf_dir.header.frame_id = global_pose.frame_id_;
  straf_dir.header.stamp = ros::Time::now();
  straf_dir.pose.position.x = obstacle_world_x;
  straf_dir.pose.position.y = obstacle_world_y;
  straf_dir.pose.orientation.x = 0;
  straf_dir.pose.orientation.y = initial_nearest_obstacle.dist;
  straf_pub_.publish(straf_dir);

  return;
  while (n.ok())
  {
    local_costmap_->getRobotPose(global_pose);

    double normalized_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    robot_x = global_pose.getOrigin().x();
    robot_y = global_pose.getOrigin().y();
    current_angle = angles::normalize_angle(normalized_angle + start_offset);

    // check if we can rotate
    double theta = 0.0;
    unsigned int map_x = 0.0, map_y = 0.0;

    bool can_rotate_in_place = canRotateInPlace(robot_map_x, robot_map_y, theta, global_pose);

    ClosestObstacle nearest_obstacle = nearestObstacle(local_costmap_, robot_map_x, robot_map_y);


    if (nearest_obstacle.dist < initial_nearest_obstacle.dist)
    {
      return;
    }

    // check if we've reached max distance
    if (current_distance_translated > maximum_translate_distance_)
    {
      ROS_WARN("Straf Recovery has met maximum translate distance");
      return;
    }

    if (current_distance_translated > minimum_translate_distance_)
    {
      ROS_WARN("Straf Recovery has met minimum translate distance");

      if (can_rotate_in_place)
      {
        ROS_WARN("Straf Recovery is able to rotate in place.");
        return;
      }
    }

    // go away from obstacles

    // (goal_x, goal_y) is in the direction we want to straf
    // subtraction of robot_map_x, robot_map_y translates to origin
    // then do dot product math, assuming x=1 and solving for y
    // then normalize
    double goal_x = 1.0;
    double goal_y  = -(goal_x * (nearest_obstacle.x - robot_map_x)) / nearest_obstacle.y - robot_map_y;
    double vel = sqrt(goal_x * goal_x + goal_y * goal_y);
    goal_x /= vel;
    goal_y /= vel;

    vel = std::min(std::max(vel, min_vel_), max_vel_);

    ROS_WARN("vel: %f", vel);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel * goal_x;
    cmd_vel.linear.y = vel * goal_y;
    cmd_vel.linear.z = 0.0;

    //vel_pub.publish(cmd_vel);

    r.sleep();
  }
}

bool StrafRecovery::canRotateInPlace(double robot_map_x, double robot_map_y, double theta, tf::Stamped<tf::Pose> global_pose)
{
  while (theta < 2 * M_PI)
  {
    double current_theta = tf::getYaw(global_pose.getRotation()) + theta;

    // make sure that the point is legal
    double footprint_cost =
        local_costmap_model_->footprintCost(robot_map_x, robot_map_y, current_theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    if (footprint_cost < 0.0)
    {
      return false;
    }
  }

  return true;
}

ClosestObstacle StrafRecovery::nearestObstacle(costmap_2d::Costmap2DROS *local_costmap, double robot_map_x, double robot_map_y)
{
  unsigned int min_x = INT_MAX;
  unsigned int min_y = INT_MAX;
  double minimum_distance = DBL_MAX;
  double resolution = local_costmap->getCostmap()->getResolution();
  unsigned int cell_x_idx, cell_y_idx;

  costmap_2d::Costmap2D *costmap = local_costmap->getCostmap();

  for (cell_x_idx = 0; cell_x_idx < costmap->getSizeInCellsX(); cell_x_idx++)
  {
    for (cell_y_idx = 0; cell_y_idx < costmap->getSizeInCellsY(); cell_y_idx++)
    {
      double cost_idx = costmap->getCost(cell_x_idx, cell_y_idx);
      double dx = cell_x_idx - robot_map_x;
      double dy = cell_y_idx - robot_map_y;

      double dist_idx = sqrt(dx * dx + dy * dy);

      // if we found an obstacle, check and set if it's the new closest
      if (cost_idx >= costmap_2d::LETHAL_OBSTACLE)
      {

        ROS_WARN("LETHAL AT: %d,%d", cell_x_idx, cell_y_idx);

        if (dist_idx < minimum_distance) {
          minimum_distance = dist_idx;
          min_x = cell_x_idx;
          min_y = cell_y_idx;
        }
      }
    }
  }


  return ClosestObstacle(min_x, min_y, minimum_distance);
}

ClosestObstacle::ClosestObstacle(int x, int y, double dist): x(x), y(y), dist(dist) {}

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
