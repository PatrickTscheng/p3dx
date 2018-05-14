#ifndef GUI_THREAD_H
#define GUI_THREAD_H

#include <QThread>
namespace global_planner {

class MoveBase;
class GUI_Thread : public QThread
{
private:
  MoveBase* move_base;
public:
  explicit GUI_Thread();
  ~GUI_Thread();

  void run();
};
}   // namespace global_planner
#endif // GUI_THREAD_H
