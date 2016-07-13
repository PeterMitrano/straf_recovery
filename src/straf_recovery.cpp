#include "straf_recovery/straf_recovery.h"
#include "obstacle_finder/obstacle_finder.h"

#include <angles/angles.h>
#include <cmath>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

namespace straf_recovery
{
StrafRecovery::StrafRecovery() : initialized_(false), cycles_(0)
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
  ros::NodeHandle nh;
  ros::NodeHandle base_local_planner_nh("~/TrajectoryPlannerROS");

  private_nh.param("go_to_goal_distance_threshold", go_to_goal_distance_threshold_, 1.0);
  private_nh.param("minimum_translate_distance", minimum_translate_distance_, 0.5);
  private_nh.param("maximum_translate_distance", maximum_translate_distance_, 5.0);
  private_nh.param("straf_vel", vel_, 0.1);
  private_nh.param("timeout", timeout_, 10);

  // use the same control frequency and xy goal tolerance as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);
  base_local_planner_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);

  // for visualizing
  obstacle_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("obstacle_direction", 10);

  // for failure detection
  cycles_pub_ = private_nh.advertise<std_msgs::Int32>("cycles", 10);

  ROS_INFO("minimum_translate_distance %f", minimum_translate_distance_);
  ROS_INFO("maximum_translate_distance %f", maximum_translate_distance_);
  ROS_INFO("vel %f", vel_);

  // use the same control frequency as the base local planner
  base_local_planner_nh.param("frequency", frequency_, 20.0);

  // knowing current goal means we can cheat and straf to the goal
  goal_sub_ = nh.subscribe("move_base_simple/goal", 10, &StrafRecovery::goalCallback, this);
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

  ROS_WARN("Entering straf recovery behavior.");

  ros::NodeHandle n;
  ros::Rate r(frequency_);
  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> initial_local_pose;
  local_costmap_->getRobotPose(initial_local_pose);

  tf::Stamped<tf::Pose> initial_global_pose;
  global_costmap_->getRobotPose(initial_global_pose);

  double current_distance_translated = 0.0;

  // find the nearest obstacle
  double robot_odom_x = initial_local_pose.getOrigin().x();
  double robot_odom_y = initial_local_pose.getOrigin().y();

  obstacle_finder::ObstacleFinder finder(local_costmap_, robot_odom_x, robot_odom_y);

  ros::Time end = ros::Time::now() + ros::Duration(timeout_);
  while (n.ok() && ros::Time::now() < end)
  {
    tf::Stamped<tf::Pose> global_pose;
    global_costmap_->getRobotPose(global_pose);

    tf::Stamped<tf::Pose> local_pose;
    local_costmap_->getRobotPose(local_pose);

    double robot_odom_x = local_pose.getOrigin().x();
    double robot_odom_y = local_pose.getOrigin().y();

    current_distance_translated = (global_pose.getOrigin() - initial_global_pose.getOrigin()).length();

    tf::Stamped<tf::Pose> last_goal_pose;
    tf::poseStampedMsgToTF(last_goal_, last_goal_pose);

    double distance_to_goal = (last_goal_pose.getOrigin() - local_pose.getOrigin()).length();

    if (distance_to_goal < go_to_goal_distance_threshold_) {
      tf::Pose goal_pose;
      tf::poseMsgToTF(last_goal_.pose, goal_pose);
      strafInDiretionOfPose(local_pose, goal_pose.getOrigin());

      if (distance_to_goal < xy_goal_tolerance_){
        return;
      }
    }
    else {

      // The obstacle finder has a pointer to the local costmap, so that will keep working.
      // We just need to use the opdated x and y
      obstacle_finder::Obstacle nearest_obstacle = finder.nearestObstacle(robot_odom_x, robot_odom_y);

      // check if we've reached max distance
      if (current_distance_translated > maximum_translate_distance_)
      {
        ROS_WARN("Straf Recovery has met maximum translate distance");
        return;
      }

      // check if we've reade the minimum distance
      if (current_distance_translated > minimum_translate_distance_)
      {
        return;
      }

      tf::Vector3 obstacle_pose(nearest_obstacle.x, nearest_obstacle.y, local_pose.getOrigin().z());
      strafInDiretionOfPose(local_pose, obstacle_pose);
    }
    r.sleep();
  }

  cycles_++;
  std_msgs::Int32 msg;
  msg.data = cycles_;
  cycles_pub_.publish(msg);
}

void StrafRecovery::strafInDiretionOfPose(tf::Stamped<tf::Pose> current_pose, tf::Vector3 direction_pose){
  tf::Vector3 diff = current_pose.getOrigin() - direction_pose;
  double yaw_in_odom_frame = atan2(diff.y(), diff.x());

  tf::Quaternion straf_direction = tf::createQuaternionFromYaw(yaw_in_odom_frame);

  geometry_msgs::PoseStamped obstacle_msg;

  obstacle_msg.header.frame_id = current_pose.frame_id_;
  obstacle_msg.header.stamp = ros::Time::now();
  obstacle_msg.pose.position.x = direction_pose.getX();
  obstacle_msg.pose.position.y = current_pose.getOrigin().x();
  obstacle_msg.pose.orientation.x = straf_direction.x();
  obstacle_msg.pose.orientation.y = straf_direction.y();
  obstacle_msg.pose.orientation.z = straf_direction.z();
  obstacle_msg.pose.orientation.w = straf_direction.w();

  obstacle_pub_.publish(obstacle_msg);

  geometry_msgs::PoseStamped straf_msg;
  tf_->waitForTransform("/base_link", current_pose.frame_id_, ros::Time::now(), ros::Duration(3.0));
  tf_->transformPose("/base_link", obstacle_msg, straf_msg);

  // angle in the base_link frame
  double straf_angle = tf::getYaw(straf_msg.pose.orientation);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vel_ * cos(straf_angle);
  cmd_vel.linear.y = vel_ * sin(straf_angle);
  cmd_vel.linear.z = 0.0;

  vel_pub_.publish(cmd_vel);

}

void StrafRecovery::goalCallback(const geometry_msgs::PoseStamped& msg)
{
  last_goal_ = msg;
}

void StrafRecovery::reconfigureCB(StrafRecoveryConfig& config, uint32_t level)
{
  timeout_ = config.timeout;
  minimum_translate_distance_ = config.minimum_translate_distance;
  maximum_translate_distance_ = config.maximum_translate_distance;
  xy_goal_tolerance_ = config.xy_goal_tolerance;
  go_to_goal_distance_threshold_ = config.go_to_goal_distance_threshold;
  vel_ = config.straf_vel;
  frequency_ = config.frequency;
}

}

PLUGINLIB_EXPORT_CLASS(straf_recovery::StrafRecovery, nav_core::RecoveryBehavior)
