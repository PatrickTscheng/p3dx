#include "global_planner_qt/MoveBaseThread.h"
#include "global_planner_qt/movebase.h"

namespace global_planner {

MoveBaseThread::MoveBaseThread()
{
  move_base = new MoveBase;
}

MoveBaseThread::~MoveBaseThread(){
  move_base->pubZeroVel();
  delete move_base;
}

void MoveBaseThread::run()
{
  move_base->start();
}

}   // namespace global_planner
