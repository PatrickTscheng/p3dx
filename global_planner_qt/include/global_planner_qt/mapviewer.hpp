#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QtGui/QMainWindow>
#include <QWidget>

class QImage;
class QTimer;
class QScrollArea;
class QLabel;
class QScrollBar;

namespace global_planner {

class MapViewer : public QMainWindow
{
  Q_OBJECT
private:
  QScrollArea *scrollArea;
  QLabel* label;
  QImage image;
  QTimer* timer;
  std::pair<int, int> roboter_pos_gui;
  double scaleFactor;
public:
  MapViewer(QWidget *parent = 0);
  ~MapViewer();
private:
  void drawPath();
  void drawMap();
  void draw_roboter_pos();
  void init_map();

  void adjustScrollBar(QScrollBar *scrollBar, double factor);
  void wheelEvent(QWheelEvent* wheelEvent);

private Q_SLOTS:
  void update_window();
};

}
#endif // MAPVIEWER_H
