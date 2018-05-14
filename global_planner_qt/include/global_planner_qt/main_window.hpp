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

/*****************************************************************************
** Namespace
*****************************************************************************/
class QScrollArea;
class QLabel;
class QTimer;

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
  QScrollArea *scrollArea;
  QLabel* label;
  QImage image;
  QTimer* timer;

  std::pair<int, int> roboter_pos_gui;

public:
  MainWindow(QWidget *parent = 0);
	~MainWindow();
  void drawMap();

private:
  void drawPath();
public Q_SLOTS:
  void update_roboter_pos();
  void quit_simulation();
  void start_simulation();
private:
  Ui::MainWindow *ui;
};

}  // namespace global_planner_qt

#endif // global_planner_qt_MAIN_WINDOW_H
