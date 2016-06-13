#include "straf_recovery/straf_recovery.h"
#include "obstacle_finder/obstacle_finder.h"
#include <cmath>
#include <math.h>
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
              "nothing.");
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
  private_nh.param("straf_vel", vel_, 0.1);
  private_nh.param("timeout", timeout_, 5);

  // use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);

  //for visualizing
  obstacle_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("obstacle_direction", 10);

  ROS_INFO("minimum_translate_distance %f", minimum_translate_distance_);
  ROS_INFO("maximum_translate_distance %f", maximum_translate_distance_);
  ROS_INFO("vel %f", vel_);

  // use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);
}

void StrafRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("thou shall not fail to call initialize! Doing nothing.");
    return;
  }

  if (local_costmap_ == NULL || global_costmap_ == NULL)
  {
    ROS_ERROR("Thou shall not pass costmaps to the straf recovery which are nullptr. "
              "Doing nothing");
    return;
  }

  ROS_WARN("Straf recovery behavior started.");

  ros::NodeHandle n;
  ros::Rate r(frequency_);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> initial_global_pose;
  local_costmap_->getRobotPose(initial_global_pose);

  double current_angle = -M_PI; // rotate from -M_PI to M_PI
  double start_offset = -angles::normalize_angle(tf::getYaw(initial_global_pose.getRotation()));
  double current_distance_translated = 0.0;

  // find the nearest obstacle
  double robot_odom_x = initial_global_pose.getOrigin().x();
  double robot_odom_y = initial_global_pose.getOrigin().y();

  obstacle_finder::ObstacleFinder finder(local_costmap_, robot_odom_x, robot_odom_y);
  obstacle_finder::Obstacle initial_nearest_obstacle = finder.nearestObstacle();

  ros::Time end = ros::Time::now() + ros::Duration(timeout_);
  while (n.ok() && ros::Time::now() < end)
  {
    tf::Stamped<tf::Pose> global_pose;
    local_costmap_->getRobotPose(global_pose);
    double robot_odom_x = global_pose.getOrigin().x();
    double robot_odom_y = global_pose.getOrigin().y();

    current_distance_translated = (global_pose.getOrigin() - initial_global_pose.getOrigin()).length();

    double normalized_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    current_angle = angles::normalize_angle(normalized_angle + start_offset);

    //// check if we can rotate
    double theta = 0.0;

    bool can_rotate_in_place = canRotateInPlace(robot_odom_x, robot_odom_y, theta, global_pose);

    // The obstacle finder has a pointer to the local costmap, so that will keep working.
    // We just need to use the opdated x and y
    obstacle_finder::Obstacle nearest_obstacle = finder.nearestObstacle(robot_odom_x, robot_odom_y);

    // check if we've reached max distance
    if (current_distance_translated > maximum_translate_distance_)
    {
      ROS_WARN("Straf Recovery has met maximum translate distance");
      return;
    }

    if (current_distance_translated > minimum_translate_distance_)
    {
      if (nearest_obstacle.dist < initial_nearest_obstacle.dist)
      {
        ROS_WARN("Straf Recovery got too close to something else. Stopping.");
        return;
      }
      else if (can_rotate_in_place)
      {
        ROS_WARN("Straf Recovery is able to rotate in place. Stopping.");
        return;
      }
    }

    tf::Vector3 obstacle_pose(nearest_obstacle.x, nearest_obstacle.y, global_pose.getOrigin().z());
    tf::Vector3 diff = global_pose.getOrigin() - obstacle_pose;
    double yaw_in_odom_frame = atan2(diff.y() , diff.x());

    tf::Quaternion straf_direction = tf::createQuaternionFromYaw(yaw_in_odom_frame);

    geometry_msgs::PoseStamped obstacle_msg;

    obstacle_msg.header.frame_id = global_pose.frame_id_;
    obstacle_msg.header.stamp = ros::Time::now();
    obstacle_msg.pose.position.x = nearest_obstacle.x;
    obstacle_msg.pose.position.y = nearest_obstacle.y;
    obstacle_msg.pose.orientation.x = straf_direction.x();
    obstacle_msg.pose.orientation.y = straf_direction.y();
    obstacle_msg.pose.orientation.z = straf_direction.z();
    obstacle_msg.pose.orientation.w = straf_direction.w();

    obstacle_pub_.publish(obstacle_msg);

    geometry_msgs::PoseStamped straf_msg;
    tf_->waitForTransform("/base_link", global_pose.frame_id_, ros::Time::now(), ros::Duration(3.0));
    tf_->transformPose("/base_link", obstacle_msg, straf_msg);

    //angle in the base_link frame
    double straf_angle = tf::getYaw(straf_msg.pose.orientation);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel_ * cos(straf_angle);
    cmd_vel.linear.y = vel_ * sin(straf_angle);
    cmd_vel.linear.z = 0.0;

    vel_pub.publish(cmd_vel);

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

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
