#include "obstacle_visualizer/obstacle_visualizer.h"
#include "obstacle_finder/obstacle_finder.h"

namespace obstacle_visualizer
{
ObstacleVisualizer::ObstacleVisualizer()
{
  ros::NodeHandle nh;
  std::string costmap_topic;
  ros::param::param<std::string>("costmap_topic", costmap_topic, "/move_base/local_costmap/costmap");
  costmap_sub_ = nh.subscribe(costmap_topic, 10, &ObstacleVisualizer::costmapCallback, this);
  nearest_obstacle_pub_ = nh.advertise<geometry_msgs::Pose>("nearest_obstacle", 10, false);
}

void ObstacleVisualizer::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_visualizer");

  obstacle_visualizer::ObstacleVisualizer obstacle_visaulizer;
  ros::spin();

  return EXIT_SUCCESS;
}
