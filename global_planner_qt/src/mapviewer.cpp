#include "../include/global_planner_qt/mapviewer.hpp"
#include <QLabel>
#include <QScrollArea>
#include <QImage>
#include <QTimer>
#include <QScrollBar>
#include <iostream>
#include <QWheelEvent>
#include "global_planner_qt/param.h"

namespace global_planner {

MapViewer::MapViewer(QWidget *parent) : QMainWindow(parent)
{
  label = new QLabel;
  label->setBackgroundRole(QPalette::Base);
  label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  label->setScaledContents(true);

  scrollArea = new QScrollArea;
  scrollArea->setBackgroundRole(QPalette::Dark);
  scrollArea->setWidget(label);
  scrollArea->setWidgetResizable(true);

  scaleFactor = 1.0;
  this->setCentralWidget(scrollArea);
  timer = new QTimer;
  timer->setInterval(10);

  this->resize(QSize(400, 400));
  this->setWindowTitle("MapViewer");

  connect(timer, SIGNAL(timeout()), this, SLOT(update_window()));
  timer->start();

  init_map();
}

MapViewer::~MapViewer()
{
  delete label;
  delete scrollArea;
  delete timer;
}

void MapViewer::init_map()
{
  // map and roboter position are already initialized
  image = QImage(QSize(ROW, COL), QImage::Format_ARGB32);
  QRgb value;
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (gridMap[i][j] == 0 ) {
        value = qRgb(255, 255, 255);  // white free
      } else if (gridMap[i][j] == -1) {
        value = qRgb(128, 128, 128);  // gray unknow
      } else if (gridMap[i][j] == 110) {
        value = qRgb(255, 0, 0);      // rot for costmap
      } else if (gridMap[i][j] >= 120) {
        value = qRgb(255, gridMap[i][j], 0);
      } else {
        value = qRgb(0, 0, 0);        // black block
      }
      image.setPixel(QPoint(j, i), value);
    }
  }
  label->setPixmap(QPixmap::fromImage(image));
}

void MapViewer::drawMap()
{
  //image = QImage(QSize(ROW, COL), QImage::Format_ARGB32);
  QRgb value;
  int roboter_row = roboter_pos.first;
  int roboter_col = roboter_pos.second;
  for (int row = roboter_row - costMap_area; row < roboter_row + costMap_area; row++) {
    for (int col = roboter_col - costMap_area; col < roboter_col + costMap_area; col++) {
      if (gridMap[row][col] == 0 ) {
        value = qRgb(255, 255, 255);  // white free
      } else if (gridMap[row][col] == -1) {
        value = qRgb(128, 128, 128);  // gray unknow
      } else if (gridMap[row][col] == 110) {
        value = qRgb(255, 0, 0);      // rot for costmap
      } else if (gridMap[row][col] >= 120) {
		value = qRgb(255, gridMap[row][col], 0);
	  } else {
        value = qRgb(0, 0, 0);        // black block
      }
      image.setPixel(QPoint(col, row), value);
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

void MapViewer::drawPath()
{
  QRgb value;
  for (int i = 0; i < path.size(); i++) {
    value = qRgb(0, 255, 0);
    int row = path[i].first;
    int col = path[i].second;
    image.setPixel(QPoint(col, row), value);
  }
}

void MapViewer::draw_roboter_pos()
{
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
}

void MapViewer::update_window()
{
  drawMap();    // update map
  drawPath();   // update path
  draw_roboter_pos();   // update roboter position

  label->setPixmap(QPixmap::fromImage(image));
}

void MapViewer::wheelEvent(QWheelEvent *wheelEvent)
{
  std::cout << "wheel event " << wheelEvent->delta() << std::endl;

  double factor = wheelEvent->delta() > 0 ? 0.8 : 1.25;
  scaleFactor *= factor;
  scrollArea->setWidgetResizable(false);
  label->resize(scaleFactor * label->pixmap()->size());
  adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
  adjustScrollBar(scrollArea->verticalScrollBar(), factor);
}

void MapViewer::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
  scrollBar->setValue(int(factor * scrollBar->value()
                               + ((factor - 1) * scrollBar->pageStep()/2)));
}

}   // namespace



