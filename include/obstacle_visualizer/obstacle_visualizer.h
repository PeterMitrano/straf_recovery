#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

namespace obstacle_visualizer {

class ObstacleVisualizer {
  public:
    ObstacleVisualizer();

    void costmapCallback(const nav_msgs::OccupancyGrid& msg);

  private:
    ros::Subscriber costmap_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Publisher nearest_obstacle_pub_;

};
}
