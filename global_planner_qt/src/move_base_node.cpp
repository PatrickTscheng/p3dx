#include "global_planner_qt/movebase.h"
#include "global_planner_qt/MoveBaseThread.h"
#include "global_planner_qt/mapthread.h"
#include <QApplication>
#include "global_planner_qt/main_window.hpp" //../include/global_planner_qt/
#include <unistd.h>
#include "global_planner_qt/param.h"

QMutex mutex;     // global mutex
QWaitCondition condition;
std::vector<std::vector<int> > gridMap;
std::vector<std::pair<int, int> > path;
std::vector<std::vector<int> > roboter_local_field;
std::pair<int, int> roboter_pos;
std::queue<goal_t> goals;
goal_t curr_goal;

bool stopSim = true;
bool is_dest_reachable = true;
bool is_roboter_pos_init = false;
bool is_job_finished = false;
bool request_new_plan = true;
bool is_map_init = false;
bool is_path_init = false;
bool is_reach_goal = false;

int ROW;
int COL;
double mapResolution;
const int costMap_area = 200; //local

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  // Init ros node
  ros::init(argc, argv, "move_base_qt");

  QApplication app(argc, argv);

  global_planner::MapThread map_thread;
  global_planner::MoveBaseThread movebase_thread;

  map_thread.start();
//  while (!map_thread.isPathInit) {
//    sleep(1);
//  }
  movebase_thread.start();
  global_planner::MainWindow window;
  window.show();
  return app.exec();
}


