#ifndef MOVEBASE_H
#define MOVEBASE_H
#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose2D.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud.h>
#define DEG2RAD (M_PI / 180.0)
#define SONAR_INDEX_MAX 7
#define DURATION 3.0
// coordination <x, y>
typedef std::pair<double, double> CoPair;
namespace global_planner {
class MoveBase
{
  enum MoveDirection {
//    BACKWARD = -1,
    FORWARD = 1
  };
  enum RotateDirection {
    CLOCKWISE = -1,
    ANTICLOCKWISE = 1
  };
  enum SPEED_MPS {
    SLOW = 1,
    FAST = 3
  };
  enum STEPS {
    SHORT = 12,
    LONG = 20
  };
private:
  ros::NodeHandle nh;
  geometry_msgs::Pose2D current_pose;
  ros::Publisher pub_movement;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_laser;
  ros::Subscriber sub_sonar;
  MoveDirection moveDirection;
  RotateDirection rotateDirection;
  SPEED_MPS speed;
  STEPS steps;
  bool isMoveable;
  ros::Time begin;

public:
  const static double MIN_SCAN_ANGLE_RAD = 30.0 / 180.0 * M_PI;
  const static float MIN_PROXIMITY_RANGE_M = 0.5;

public:
  MoveBase();
  void start();
  void pubZeroVel();
private:
  void init();
  void excutePlan();
  void calc_coordinate_matrix(double x, double y, std::pair<int, int> &step);
  void calc_coordiante_map(std::pair<int, int> &step, CoPair &coordinate);
  void odom_callback(const nav_msgs::OdometryConstPtr& msg);
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void sonar_callback(const sensor_msgs::PointCloud::ConstPtr& sonar);
  void find_nearst_pos();
  int calc_distance(std::pair<int, int> point1, std::pair<int, int> point2);
  float calc_movement();
  bool reach_goal(geometry_msgs::Twist &twist);
  void rotate(geometry_msgs::Twist &twist, double theta);
  void move(geometry_msgs::Twist &twist);
  void rotate_to_goal(geometry_msgs::Twist &twist);
  void obstacledetect(double closestRange);
  void pubZeroVel(int time);

  template<typename T>
  int signur(T num) {
    if (num < 0) return -1;
    else return 1;
  }

  template<typename T>
  T abs(T x) {
    return signur(x) * x;
  }
};
} // namespace global_planner
#endif // MOVEBASE_H
