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
public:
  bool isMapInit;
  bool isPathInit;

private:
  ros::NodeHandle nh;
  ros::Subscriber map_sub;
  ros::Publisher pub_vel;
public:
  MapThread();
  ~MapThread();
  void run();
  void makePlan();
private:
  void initMap();
  void updateMap(const nav_msgs::OccupancyGridConstPtr map);
  void calc_costMap();
  bool isPathBlocked();
  void updateAll();
  void pubZeroVel();
};
}


#endif // MAPTHREAD_H
