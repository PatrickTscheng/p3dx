#include "global_planner_qt/mapthread.h"
#include<geometry_msgs/Twist.h>
#include "global_planner_qt/AStar.h"
#include "global_planner_qt/param.h"


namespace global_planner {

MapThread::MapThread()
{
  this->astar = new AStar;
  map_sub = nh.subscribe("/map", 1, &MapThread::updateMap, this);
  pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  std::cout << "mapthread \n";
//  initMap();
}

MapThread::~MapThread()
{
  delete astar;
}

void MapThread::initMap()
{
    ROS_INFO("Waiting for Initialization of Map ...");
  while (!is_map_init) {
    ros::spinOnce();
  }
  ROS_INFO("map is initialized ...");
}

void MapThread::run()
{
  initMap();
  while (!is_roboter_pos_init) {
	  sleep(1);
  }
//  if (makePlan()) {
//                ROS_INFO("new path is found");
//              }
  updateAll();
}

void MapThread::updateAll()
{
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    //calc_costMap();
    while (!is_job_finished ) {
//    	std::cout << "job not finished \n";
          ros::spinOnce();
          //calc_costMap();
          if ( request_new_plan || isPathBlocked()) {
            if (makePlan()) {
              ROS_INFO("new path is found");
              request_new_plan = false;
            }
          }
          loop_rate.sleep();
        }
    loop_rate.sleep();
  }
}

bool MapThread::makePlan()
{
  int count = 1;
  int row = roboter_pos.first;
  int col = roboter_pos.second;
  std::pair<int, int> start;
  while (true) {
    if (gridMap[row + count][col] == 0 || gridMap[row + count][col] == 120) {
      start.first = row + count;
      start.second = col;
      break;
    }
    if (gridMap[row - count][col] == 0 || gridMap[row - count][col] == 120) {
      start.first = row - count;
      start.second = col;
      break;
    }
    if (gridMap[row][col + count] == 0 || gridMap[row][col + count] == 120) {
      start.first = row;
      start.second = col + count;
      break;
    }
    if (gridMap[row][col - count] == 0 || gridMap[row][col - count] == 120) {
      start.first = row;
      start.second = col - count;
      break;
    }
    count++;
  }
  mutex.lock();   // lock, because path is also in movebase.cpp used
  std::cout << "in makePlan(), has clock \n";
  pubZeroVel();   // stop roboter while recreate the path
  bool res = this->astar->aStarSearch(gridMap, start, curr_goal.goal_pos, path);
  mutex.unlock();
  std::cout << "in makePlan(), release clock \n";
  return res;
}

bool MapThread::isPathBlocked()
{
  //std::cout << "in isPathBlocked() \n";
	int count = path.size() - 1;
	while (count > curr_goal.distance_precision) {
		if (gridMap[path[count].first][path[count].second] == 100 ||
			gridMap[path[count].first][path[count].second] == 110) {
			//  std::cout << "in isPathBlocked()  return true\n";
			return true;
		 }
    count--;
	}
    //std::cout << "in isPathBlocked() return false\n";
    return false;
}

void MapThread::calc_costMap()
{
	  int redRange = 25;
	  int orangeRange = 15;
	  int Upper = orangeRange * 5 + 120;
	  int roboter_row = roboter_pos.first;
	  int roboter_col = roboter_pos.second;
	  for (int row = roboter_row - costMap_area; row < roboter_row + costMap_area; row++) {
	    for (int col = roboter_col - costMap_area; col < roboter_col + costMap_area; col++) {
	      if (gridMap[row][col] >= 50 && gridMap[row][col] <= 100) {   // Block
	        for (int n = -redRange; n <= redRange; n++) {
	          for (int m = -redRange; m <= redRange; m++) {
	            if (gridMap[row+n][col+m] != 100 && gridMap[row+n][col+m] != 110) {
	              gridMap[row+n][col+m] = 110;    // 110 indicate red area
	            }
	          }
	        }

	        for (int i = 1; i <= orangeRange; i++) {
	          draw_costMap(row, col, i + redRange, Upper - i * 5);
	        }

	        /**
	        for (int n = -(redRange+orangeRange); n <= (redRange+orangeRange); n++) {
	          for (int m = -(redRange+orangeRange); m <= (redRange+orangeRange); m++) {
	            if (gridMap[row+n][col+m] != 100 && gridMap[row+n][col+m] != 110 && gridMap[row+n][col+m] != 120) {
	              gridMap[row+n][col+m] = 120;    // 120 indicate orange area
	            }
	          }
	        }
	        */
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
  //std::cout << "row: " << ROW << ", col: " << COL << " \n";
  mapResolution = map->info.resolution;
  //std::cout << "map resolution: " << mapResolution << " \n";
  // Init gridMap, if first get map
  if (!is_map_init) {
    gridMap.resize(ROW);
    for (int i = 0; i < ROW; i++) {
      gridMap[i].resize(COL);
      std::fill(gridMap[i].begin(), gridMap[i].end(), -1);
    }
    is_map_init = true;
  }

  if (is_roboter_pos_init) {
      int roboter_row = roboter_pos.first;
      int roboter_col = roboter_pos.second;
      for (int row = roboter_row - costMap_area; row < roboter_row + costMap_area; row++) {
        for (int col = roboter_col - costMap_area; col < roboter_col + costMap_area; col++) {
          int curCell = COL * row + col;
          //if (gridMap[row][col] == 0 || gridMap[row][col] == -1) {
            gridMap[row][col] = map->data[curCell];
          //}
        }
      }
      calc_costMap();
    } else {
      int curCell = 0;
      for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
          gridMap[i][j] = map->data[curCell];
          curCell++;
        }
      }
    }

//  for (int i = ROW-1; i >= 0; i--) {
//      for (int j = 0; j < COL; j++) {
//        gridMap[i][j] = map->data[curCell];
//        curCell++;
//      }
//    }
}

void MapThread::pubZeroVel()
{
  geometry_msgs::Twist twist;
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
  pub_vel.publish(twist);
  ros::spinOnce();
}
void MapThread::draw_costMap(int row, int col, int n, int value)
{
  for (int i = col - n; i <= col + n; i++) {
    if (gridMap[row-n][i] != 100 && gridMap[row-n][i] != 110) {
      if (gridMap[row-n][i] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[row-n][i]) {
          gridMap[row-n][i] = value;
        }
      } else {
        gridMap[row-n][i] = value;    // 0 or -1
      }
    }
    if (gridMap[row+n][i] != 100 && gridMap[row+n][i] != 110) {
      if (gridMap[row+n][i] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[row+n][i]) {
          gridMap[row+n][i] = value;
        }
      } else {
        gridMap[row+n][i] = value;    // 0 or -1
      }
    }
  }
  for (int i = row - n; i <= row + n; i++) {
    if (gridMap[i][col-n] != 100 && gridMap[i][col-n] != 110) {
      if (gridMap[i][col-n] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[i][col-n]) {
          gridMap[i][col-n] = value;
        }
      } else {
        gridMap[i][col-n] = value;    // 0 or -1
      }
    }
    if (gridMap[i][col+n] != 100 && gridMap[i][col+n] != 110) {
      if (gridMap[i][col+n] >= 120) {   // 120 is lowest value for orange area
        if (value > gridMap[i][col+n]) {
          gridMap[i][col+n] = value;
        }
      } else {
        gridMap[i][col+n] = value;    // 0 or -1
      }
    }
  }

}

}   // namespace global_planner

