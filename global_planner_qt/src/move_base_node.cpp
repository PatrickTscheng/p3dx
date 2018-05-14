#include "movebase.h"
#include "gui_thread.h"
#include "mapthread.h"
#include <QApplication>
#include "main_window.hpp" //../include/global_planner_qt/
#include <unistd.h>
#include "param.h"

QMutex mutex;     // global mutex
QWaitCondition condition;
bool quitSim = true;
bool is_roboter_pos_init = false;
std::vector<std::vector<int> > gridMap;
std::vector<std::pair<int, int> > path;
std::vector<std::vector<int> > roboter_local_field;
std::pair<int, int> roboter_pos;
std::pair<int, int> goal;
int ROW;
int COL;
double mapResolution;

int main(int argc, char** argv) {
	ROS_INFO("Start ...");

  // Init ros node
  ros::init(argc, argv, "move_base_qt");

  QApplication app(argc, argv);

  global_planner::MapThread map_thread;
  global_planner::GUI_Thread gui_thread;
  map_thread.start();
  while (!map_thread.isPathInit) {
    sleep(1);
  }

  gui_thread.start();
  global_planner::MainWindow window;
  window.show();


  return app.exec();
}


