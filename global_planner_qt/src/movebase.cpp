#include "global_planner_qt/movebase.h"
#include "global_planner_qt/param.h"

namespace global_planner {
MoveBase::MoveBase()
{
  sub_odom = nh.subscribe("/odom", 1, &MoveBase::odom_callback, this);
//  sub_laser = nh.subscribe("/scan", 1, &MoveBase::scan_callback, this);
  sub_sonar = nh.subscribe("/sonar", 10, &MoveBase::sonar_callback, this);
  pub_movement = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  moveDirection = FORWARD;
  rotateDirection = CLOCKWISE;
  speed = SLOW;
  steps = SHORT;
  begin = ros::Time::now();

  isMoveable = true;
}

void MoveBase::init() {
  // Waiting bis roboter_pos position initialized
  std::cout << "Waiting for robot position initialization \n";
  while (!is_roboter_pos_init) {
    ros::spinOnce();
  }

  std::cout << "Roboter position is initialized ...\n";
  // Set Goal manuel
  //CoPair GOAL = std::make_pair(1.6, 1.6);
  //goal = std::make_pair(160, 62);
  calc_coordinate_matrix(5.0, 0, curr_goal.goal_pos);
  curr_goal.yaw = 0;
  curr_goal.distance_precision = 20;
  // Init roboter local field
  int roboter_length =  8;
  roboter_local_field.resize(roboter_length);
  for (int i = 0; i < roboter_local_field.size(); i++) {
    roboter_local_field[i].resize(roboter_length);
  }
}

void MoveBase::start() {
    while (!is_map_init) {
        sleep(1);
    }
    init();
    excutePlan();
}

void MoveBase::excutePlan() {
  geometry_msgs::Twist twist;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
	while (request_new_plan) {
      sleep(1);
    }
    while (!stopSim && !is_job_finished ) {

    	 if (!reach_goal(twist) && isMoveable) {
			  mutex.lock(); // lock, because path can be changed in MapThread
			  find_nearst_pos();
			  rotate(twist, 0.5);
			  move(twist);
			  ros::spinOnce();
			  mutex.unlock();
			  loop_rate.sleep();
		  } else if (!isMoveable) {
			pubZeroVel();
            find_nearst_pos();
			rotate(twist, 0.02);
			request_new_plan = true;
			sleep(1);
		  }
    }
    pubZeroVel();
	 if (stopSim) {
	   pubZeroVel();
	   // Wait for starting ...
	   mutex.lock();
	   condition.wait( & mutex);
	   mutex.unlock();
	 }
  }
}

void MoveBase::odom_callback(const nav_msgs::OdometryConstPtr& msg) {
  // current linear position
  current_pose.x = msg->pose.pose.position.x;
  current_pose.y = msg->pose.pose.position.y;

  // current rotation
  tf::Quaternion quat(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
  tf::Matrix3x3 matrix(quat);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  current_pose.theta = yaw;

  // initialize roboter_pos position as current position
  calc_coordinate_matrix(current_pose.x, current_pose.y, roboter_pos);
  is_roboter_pos_init = true;
}

void MoveBase::scan_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
       // Find the closest range between the defined minimum and maximum angles
       int minIndex = (int) ceil((-23*DEG2RAD - msg->angle_min) / msg->angle_increment);
       int maxIndex = (int) floor((23*DEG2RAD - msg->angle_min) / msg->angle_increment);

       float closest_range;
           closest_range = msg -> ranges[minIndex];
           for (int i = 0; i < minIndex; i++) {
               if (msg -> ranges[i] < closest_range) {
                   closest_range = msg -> ranges[i];
               }

               if (msg -> ranges[maxIndex - i - 2] < closest_range) {
                   closest_range = msg -> ranges[maxIndex - i - 2];
               }
           }


       //ROS_INFO_STREAM("Closest Range: "  << closest_range);

       return obstacledetect(closest_range);

   }




void MoveBase::sonar_callback(const sensor_msgs::PointCloud::ConstPtr& sonar)
{
	std::vector<float> distances;
	for(int i = 0; i <= SONAR_INDEX_MAX; ++i) {
		distances.push_back(sqrt(sonar->points[i].x*sonar->points[i].x + sonar->points[i].y * sonar->points[i].y));
	}

		//find the closest distance between [1]-[7]
		float closestRange = distances[2];

		for(int i = 1; i < SONAR_INDEX_MAX ; i++ ) {
			if(distances[i] < closestRange) {
				closestRange = distances[i];
			}
		}

		return obstacledetect(closestRange);

}

void MoveBase::obstacledetect(double closestRange) {
	if (closestRange < MIN_PROXIMITY_RANGE_M)
		       {
		         ROS_INFO("hearing block ...");
	//	         pubZeroVel();
		         speed = SLOW;
		         steps = SHORT;

		         if(isMoveable) {
		        	 double elapsed_time = ros::Time::now().toSec() - begin.toSec();
	//	        	 std::cout << "past time" << elapsed_time << std::endl;
		        	 if (elapsed_time > DURATION) {
		        		 std::cout << "past time" << elapsed_time << std::endl;
		        		 pubZeroVel(5);
		        		 isMoveable = false;
		        		 begin = ros::Time::now();
		        	 }
		        	 return;
		         }

		       }
		       speed = FAST;
		       steps = LONG;
		       isMoveable = true;
}

void MoveBase::rotate(geometry_msgs::Twist &twist, double theta)
{
  ros::Rate loop_rate(10);

  while (calc_movement() > theta) {
	  twist.linear.x = 0;
	  twist.angular.z = calc_movement() * rotateDirection;
	  pub_movement.publish(twist);
	  ros::spinOnce();
	  loop_rate.sleep();
	}
}


void MoveBase::move(geometry_msgs::Twist &twist)
{
  twist.angular.z = 0.6 * calc_movement() * rotateDirection;
  twist.linear.x = 0.1 * speed * moveDirection;
  pub_movement.publish(twist);
}


void MoveBase::rotate_to_goal(geometry_msgs::Twist & twist) {
    ros::Rate loop_rate(10);
    double goal_yaw = curr_goal.yaw;
    double roboter_direction = current_pose.theta;
    if (goal_yaw < 0) {
        goal_yaw += 2*M_PI;
    }
    if (roboter_direction < 0) {
        roboter_direction += 2*M_PI;
    }
    while (abs(goal_yaw - roboter_direction) > 0.5) {
        twist.linear.x = 0;
        twist.angular.z = 0.5 * (goal_yaw - roboter_direction);
        pub_movement.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
        roboter_direction = current_pose.theta;
        if (roboter_direction < 0) {
            roboter_direction += 2*M_PI;
        }
    }
    pubZeroVel();
    loop_rate.sleep();
}

bool MoveBase::reach_goal(geometry_msgs::Twist & twist) {
    if (path.size() < curr_goal.distance_precision) {
        rotate_to_goal(twist);
        if (goals.empty() && !request_new_plan) {
            is_job_finished = true;
            //stopSim = true;
            //request_new_plan = false;
        }
        else if (!goals.empty()) {

            ROS_ERROR("%d", curr_goal.task);
//            if (curr_goal.task == GRAB) {
//                Grab grab;
//                grab.grab_resource();
//                ROS_ERROR("DELETE GRAB");
//            }
//
//            else if (curr_goal.task == DROP) {
//                Drop drop;
//                drop.drop_resource();
//                ROS_ERROR("DELETE DROP");
//            }
//            curr_goal = goals.front();//goals[goals.size() - 1];
//            goals.pop();
            request_new_plan = true;
            while (request_new_plan) {
                sleep(0.1);
            }

        }
        return true;
    }
    return false;
}
void MoveBase::pubZeroVel() {
	pubZeroVel(0);
}

void MoveBase::pubZeroVel(int time) {
	geometry_msgs::Twist twist;
	twist.linear.x = 0;
	twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    pub_movement.publish(twist);
    ros::Duration(time).sleep();
    ros::spinOnce();
}

void MoveBase::find_nearst_pos() {
  int count = 5;
	while (calc_distance(path[path.size() - 1], roboter_pos) > calc_distance(path[path.size() - 5], roboter_pos)) {
	  path.pop_back();
	  path.pop_back();;
	  path.pop_back();
	  path.pop_back();
	}

}

int MoveBase::calc_distance(std::pair<int, int> point1, std::pair<int, int> point2) {
  return signur(point1.first - point2.first) * (point1.first - point2.first) +
	 signur(point1.second - point2.second) * (point1.second - point2.second);
}

void MoveBase::calc_coordiante_map(std::pair<int, int> &pixel, CoPair &coordinate) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  double x = ((double) (pixel.second - origin_x)) * mapResolution;
  double y = ((double) (pixel.first - origin_y)) * mapResolution;
  coordinate.first = x;
  coordinate.second = y;
}

void MoveBase::calc_coordinate_matrix(double x, double y, std::pair<int, int> &pixel) {
  int origin_x = COL / 2;
  int origin_y = ROW / 2;

  int i = origin_x + (int) (x / mapResolution);
  int j = origin_y + (int) (y / mapResolution);

  pixel.second = i;
  pixel.first = j;
}

float MoveBase::calc_movement()
{
  int theta_x = path[path.size()-steps].second - roboter_pos.second;
  int theta_y = path[path.size()-steps].first - roboter_pos.first;
  float target_angle = atan2((double) theta_y, (double) theta_x);
  float robot_direction = current_pose.theta;
  float PI = M_PI;
  moveDirection = FORWARD;

  if (target_angle >= 0 && target_angle <  PI) {
	  if (robot_direction > target_angle - PI && robot_direction <= target_angle) {
		  rotateDirection = ANTICLOCKWISE;
		  return this->abs(target_angle - robot_direction);
	  } else {
		  rotateDirection = CLOCKWISE;
		  if (robot_direction < 0) {
			  return this->abs(2*PI + robot_direction - target_angle);
		  } else {
			  return this->abs(robot_direction - target_angle);
		  }
	  }
  } else {
	  if (robot_direction >= target_angle && robot_direction < PI + target_angle) {
		  rotateDirection = CLOCKWISE;
		  return this->abs(robot_direction - target_angle);
	  } else {
		  rotateDirection = ANTICLOCKWISE;
		  if (robot_direction < 0) {
			  return this->abs(target_angle - robot_direction);
		  } else {
			  return this->abs(2*PI - robot_direction + target_angle);
		  }
	  }
  }

}

//float MoveBase::calc_movement()
//{
//  int theta_x = path[path.size()-steps].second - roboter_pos.second;
//  int theta_y = path[path.size()-steps].first - roboter_pos.first;
//  float angle = atan2((double) theta_y, (double) theta_x);
//  float theta = current_pose.theta;
//  float PI = 3.1416;
//  //ROS_ERROR("angle = %f, theta = %f", angle, theta);
//  if (angle >= 0 && angle <  PI/ 2) {
//    if ( angle - PI/2 < theta && theta <= angle) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle < theta && angle + PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle - PI < theta && angle - PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle + PI);   //--
//    } else {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = BACKWARD;
//      if (theta > 0) { return PI - theta + angle; }
//      else { return abs(theta - angle + PI); }  //--
//    }
//  } else if ((angle >= PI / 2) && angle < PI) {
//    if (angle - PI/2 < theta && theta <= angle) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle - PI < theta && angle - PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle + PI);   //--
//    } else if (angle - 3*PI/2 < theta && angle - PI > theta) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle + PI);   //--
//    } else {
//      rotateDirection = CLOCKWISE;
//      moveDirection = FORWARD;
//      if (theta < 0) { return 2*PI + theta - -angle; }
//      else { return theta - angle; }
//    }
//  } else if (angle >= -(PI / 2) && angle < 0) {
//    if (angle < theta && angle + PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle - PI/2 < theta && theta < angle) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle + PI/2 < theta && angle + PI >= theta) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle - PI);   //--
//    } else {
//      rotateDirection = CLOCKWISE;
//      moveDirection = BACKWARD;
//      if (theta < 0) { return PI + theta - angle; }
//      else { return theta - angle - PI; }
//    }
//  } else {
//    if (angle < theta && angle + PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = FORWARD;
//      return this->abs(theta - angle);
//    } else if (angle + PI/2 < theta && angle + PI >= theta) {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle - PI);   //--
//    } else if (angle + PI < theta && angle + 3*PI/2 >= theta) {
//      rotateDirection = CLOCKWISE;
//      moveDirection = BACKWARD;
//      return this->abs(theta - angle - PI);
//    } else {
//      rotateDirection = ANTICLOCKWISE;
//      moveDirection = FORWARD;
//      if (theta > 0) { return 2*PI - theta + angle; }
//      else { return abs(theta - angle);}
//    }
//  }
//}

} // namespace global_planner
