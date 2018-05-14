#include "mapthread.h"
#include<geometry_msgs/Twist.h>
#include "AStar.h"
#include "param.h"


namespace global_planner {
/**
std::vector<std::vector<int> > MapThread::gridMap;
std::vector<std::pair<int, int> > MapThread::path;
int MapThread::ROW;
int MapThread::COL;

double MapThread::mapResolution;
*/
MapThread::MapThread()
{
  isMapInit = false;
  isPathInit = false;
  this->astar = new AStar;
  map_sub = nh.subscribe("/map", 1, &MapThread::updateMap, this);
  pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  initMap();
}

MapThread::~MapThread()
{
  delete astar;
}

void MapThread::initMap()
{
  while (!isMapInit) {
    ros::spinOnce();
  }
  ROS_INFO("map is initialized ...");
}

void MapThread::run()
{
  ros::Rate loop_rate(10);
  while (!isMapInit) {
    ROS_INFO("Waiting for Initialzation of Map ...");
    ros::spinOnce();
    loop_rate.sleep();
  }
  makePlan();
  isPathInit = true;
  updateAll();
}

void MapThread::updateAll()
{
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    calc_costMap();

    if (isPathBlocked()) {
      makePlan();
    }
    loop_rate.sleep();
  }
}

void MapThread::makePlan()
{
  mutex.lock();   // lock, because path is also in movebase.cpp used
  std::cout << "in makePlan(), has clock \n";
  pubZeroVel();   // stop roboter while recreate the path
  this->astar->aStarSearch(gridMap, roboter_pos, goal, path);
  mutex.unlock();
  std::cout << "in makePlan(), release clock \n";
}

bool MapThread::isPathBlocked()
{
  //std::cout << "in isPathBlocked() \n";
  for (int i = 0; i < path.size(); i++) {
    if (gridMap[path[i].first][path[i].second] != 0 &&
        gridMap[path[i].first][path[i].second] != -1) {
    //  std::cout << "in isPathBlocked()  return true\n";
      return true;
    }
  }
  //std::cout << "in isPathBlocked() return false\n";
  return false;
}

void MapThread::calc_costMap()
{
  int costMapRange = 10;
  //std::cout << "int calc_costMap() \n";
  for (int i = costMapRange; i < ROW-costMapRange; i++) {
    for (int j = costMapRange; j < COL-costMapRange; j++) {
      if (gridMap[i][j] == 100) {   // Block
        for (int n = -costMapRange; n <= costMapRange; n++) {
          for (int m = -costMapRange; m <= costMapRange; m++) {
            if (gridMap[i+n][j+m] != 100 && gridMap[i+n][j+m] != 110) {
              gridMap[i+n][j+m] = 110;    // 110 indicate costmap
            }
          }
        }
      }
    }
  }
  //std::cout << "out of calc_costMap() \n";
}

void MapThread::updateMap(const nav_msgs::OccupancyGridConstPtr map)
{
  //std::cout << "here in updateMap() \n";
  ROW = map->info.height;
  COL = map->info.width;
  mapResolution = map->info.resolution;
  // Init gridMap, if first get map
  if (!isMapInit) {
    gridMap.resize(ROW);
    for (int i = 0; i < ROW; i++) {
      gridMap[i].resize(COL);
    }
    isMapInit = true;
  }

  int curCell = 0;
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      gridMap[i][j] = map->data[curCell];
      curCell++;
    }
  }
}

void MapThread::pubZeroVel()
{
  geometry_msgs::Twist twist;
  twist.angular.z = 0;
  twist.linear.x = 0;
  pub_vel.publish(twist);
  ros::spinOnce();
}

}   // namespace global_planner

