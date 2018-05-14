#ifndef PARAM_H
#define PARAM_H
#include <QMutex>
#include <QWaitCondition>
extern QMutex mutex;
extern QWaitCondition condition;
extern bool quitSim;
extern std::vector<std::vector<int> > gridMap;
extern std::vector<std::pair<int, int> > path;
extern std::vector<std::vector<int> > roboter_local_field;
extern std::pair<int, int> roboter_pos;
extern bool is_roboter_pos_init;
extern std::pair<int, int> goal;
extern int ROW;
extern int COL;
extern double mapResolution;
#endif // PARAM_H
