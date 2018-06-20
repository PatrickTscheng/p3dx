/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../include/global_planner_qt/main_window.hpp"
#include "../include/global_planner_qt/mapviewer.hpp"
#include "global_planner_qt/param.h"
//#include "movebase.h"
//#include "ui_main_window.h"
//#include <QScrollArea>
#include <QLabel>
#include <QTimer>
//#include <QMessageBox>
#include <QFileDialog>
//#include<geometry_msgs/Twist.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace global_planner {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QWidget *parent)  : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  //ui->create_assignment->show();
  connect(ui->stop, SIGNAL(clicked(bool)), this, SLOT(stop_simulation()));
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  connect(ui->map_viewer, SIGNAL(clicked(bool)), this, SLOT(open_MapViewer()));

  //pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::stop_simulation()
{
  stopSim = true;
 // pubZeroVel();
}

void MainWindow::start_simulation()
{
  stopSim = false;
  condition.wakeAll();
}

void MainWindow::open_MapViewer()
{
  MapViewer* mapViewer = new MapViewer(this);
  mapViewer->setAttribute(Qt::WA_DeleteOnClose);
  mapViewer->show();
}

//void MainWindow::pubZeroVel() {
//	geometry_msgs::Twist twist;
//	twist.linear.x = 0;
//	twist.linear.y = 0;
//    twist.linear.z = 0;
//    twist.angular.x = 0;
//    twist.angular.y = 0;
//    twist.angular.z = 0;
//    pub_cmd.publish(twist);
//    ros::spinOnce();
//}

}  // namespace global_planner

