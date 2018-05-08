/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef PIONEER_WANDER_H_
#define PIONEER_WANDER_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <math.h>
#include <limits.h>

#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.2
#define ANGULAR_VELOCITY 0.7

#define GET_DIRECTION 0
#define DRIVE_FORWARD 1
#define RIGHT_TURN    2
#define LEFT_TURN     3
#define SONAR_INDEX_MAX 7

class PioneerWander
{
 public:
  PioneerWander();
  ~PioneerWander();
  bool init();
  bool controlLoop();
  bool controlLoopLaser();
  bool controlLoopSonar();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Parameters
//  bool is_debug_;


  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber sonar_sub;

  bool sonar_control;
  double front_distance_limit_;
  double side_distance_limit_;
  double sonar_front_distance_limit_;
  double sonar_side_distance_limit_;

  double direction_vector_[3] = {0.0, 0.0, 0.0};
  double sonar_direction_vector_[3] = {0.0, 0.0, 0.0};


  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
  void sonarMsgCallBack(const sensor_msgs::PointCloud::ConstPtr& sonar);
};
#endif // PIONEER_WANDER_H_
