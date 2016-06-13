#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace obstacle_finder {

class Obstacle
{
public:
  double x, y;
  double dist;

  Obstacle(double x, double y, double dist);

  Obstacle();
};

class ObstacleFinder {

  public:
    ObstacleFinder(costmap_2d::Costmap2DROS* costmap, double robot_odom_x, double robot_odom_y);

    /** \return calculates the distance to and location of the nearest cell with LETHAL cost.
     * Returned units are costmap cells, and in odom frame.
     * \param costmap the costmap to find obstacle one
     * \param robot_odom_x needs to be in the odom frame, in units of cells
     * \param robot_odom_y y needs to be in the odom frame, in units of cells
     */
    Obstacle nearestObstacle(costmap_2d::Costmap2DROS* new_costmap, double robot_odom_x, double robot_odom_y);

    Obstacle nearestObstacle(double robot_odom_x, double robot_odom_y);

    Obstacle nearestObstacle();

  private:
    costmap_2d::Costmap2DROS* costmap_;
    double robot_odom_x_;
    double robot_odom_y_;
};

}
