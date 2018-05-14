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
#include "main_window.hpp"
#include "ui_main_window.h"
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QRgb>
#include <QTimer>
#include "param.h"

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
  /**
  label = new QLabel(this);
  label->setBackgroundRole(QPalette::Base);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);

  scrollArea = new QScrollArea;
  scrollArea->setBackgroundRole(QPalette::Dark);
  scrollArea->setWidget(label);
  scrollArea->setWidgetResizable(true);

  this->setCentralWidget(scrollArea);
  this->setWindowTitle("Map Viewer");
  this->resize(QSize(500, 500));
*/
  connect(ui->quit, SIGNAL(clicked(bool)), this, SLOT(quit_simulation()));
  connect(ui->start, SIGNAL(clicked(bool)), this, SLOT(start_simulation()));
  timer = new QTimer;
  timer->setInterval(10);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_roboter_pos()));
  timer->start();
}

MainWindow::~MainWindow() {
  //delete label;
  //delete scrollArea;
  delete ui;
}

void MainWindow::drawMap()
{
  image = QImage(QSize(ROW, COL), QImage::Format_ARGB32);
  QRgb value;
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (gridMap[i][j] == 0) {
        value = qRgb(255, 255, 255);  // white free
      } else if (gridMap[i][j] == -1) {
        value = qRgb(128, 128, 128);  // gray unknow
      } else if (gridMap[i][j] == 110) {
        value = qRgb(255, 0, 0);      // rot for costmap
      } else {
        value = qRgb(0, 0, 0);        // black block
      }
      image.setPixel(QPoint(j, i), value);
    }
  }
  // Init roboter position on map
  this->roboter_pos_gui.first = roboter_pos.first;
  this->roboter_pos_gui.second = roboter_pos.second;
  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  int rows = roboter_local_field.size();
  int cols = roboter_local_field[0].size();
  for (int i = 0; i <= rows; i++) {
    for (int j = 0; j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos_gui.second+i, roboter_pos_gui.first+j), value);
    }
  }
}

void MainWindow::drawPath()
{
  QRgb value;
  for (int i = 0; i < path.size(); i++) {
    value = qRgb(0, 255, 0);
    int row = path[i].first;
    int col = path[i].second;
    image.setPixel(QPoint(col, row), value);
  }
}

void MainWindow::update_roboter_pos()
{
  drawMap();    // update map
  drawPath();   // update path
  QRgb value = qRgb(255, 255, 255);   // white for delete
  // delete old position of roboter
  int rows = roboter_local_field.size();
  int cols = roboter_local_field[0].size();
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos_gui.second+i, roboter_pos_gui.first+j), value);
    }
  }
  // Record roboter position
  this->roboter_pos_gui.first = roboter_pos.first;
  this->roboter_pos_gui.second = roboter_pos.second;

  // Draw new position of roboter
  value = qRgb(0, 0, 255);   // blue for roboter
  for (int i = - (rows / 2); i <= rows; i++) {
    for (int j = -(cols / 2); j <= cols; j++) {
      image.setPixel(QPoint(roboter_pos_gui.second+i, roboter_pos_gui.first+j), value);
    }
  }
  ui->label->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::quit_simulation()
{
  quitSim = true;
}

void MainWindow::start_simulation()
{
  quitSim = false;
  condition.wakeAll();
}
}  // namespace global_planner

