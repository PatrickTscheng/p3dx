#include "gui_thread.h"
#include "movebase.h"

namespace global_planner {
GUI_Thread::GUI_Thread()
{
  move_base = new MoveBase;
}

GUI_Thread::~GUI_Thread(){
  delete move_base;
}

void GUI_Thread::run()
{
  move_base->excutePlan();
}

}   // namespace global_planner
