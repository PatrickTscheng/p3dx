#ifndef MAPTHREAD_H
#define MAPTHREAD_H

#include <QThread>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace global_planner {

class AStar;
class MoveBase;
class MapThread : public QThread
{
private:
  AStar* astar;

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Publisher pub_vel;
public:
  MapThread();
  ~MapThread();
  void run();
  bool makePlan();
private:
  void initMap();
  void updateMap(const nav_msgs::OccupancyGridConstPtr map);
  void calc_costMap();
  bool isPathBlocked();
  void updateAll();
  void pubZeroVel();
  void draw_costMap(int row, int col, int n, int value);
};
}


#endif // MAPTHREAD_H
