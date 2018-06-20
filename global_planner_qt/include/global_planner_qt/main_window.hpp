/**
 * @file /include/global_planner_qt/main_window.hpp
 *
 * @brief Qt based gui for global_planner_qt.
 *
 * @date November 2010
 **/
#ifndef global_planner_qt_MAIN_WINDOW_H
#define global_planner_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include <QObject>
//#include "ros/ros.h"

/*****************************************************************************
** Namespace
*****************************************************************************/
class QProcess;
namespace global_planner {
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
  MainWindow(QWidget *parent = 0);
  ~MainWindow();
private Q_SLOTS:
//  ros::Publisher pub_cmd;
//  ros::NodeHandle nh;
  void stop_simulation();
  void start_simulation();
  void open_MapViewer();
//  void pubZeroVel();

private:
  Ui::MainWindow *ui;
};

}  // namespace global_planner_qt

#endif // global_planner_qt_MAIN_WINDOW_H
