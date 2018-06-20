#ifndef MoveBaseTHREAD_H
#define MoveBaseTHREAD_H

#include <QThread>
namespace global_planner {

class MoveBase;
class MoveBaseThread : public QThread
{
private:
  MoveBase* move_base;
public:
  explicit MoveBaseThread();
  ~MoveBaseThread();

  void run();
};
}   // namespace global_planner
#endif // GUI_THREAD_H
