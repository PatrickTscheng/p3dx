
#include "pioneer_wander/pioneer_wander.h"


PioneerWander::PioneerWander()
{
  //Init Pioneer wander modes
  ROS_INFO("Pioneer3DX wander mode Init");
  ROS_ASSERT(init());
}

PioneerWander::~PioneerWander()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

bool PioneerWander::init()
{
	// initialize ROS parameter
	std::string cmd_vel_topic_name = "RosAria/cmd_vel";
    front_distance_limit_ = 0.9;
    side_distance_limit_  = 0.65;
    sonar_front_distance_limit_ = 0.6;
    sonar_side_distance_limit_  = 0.4;
    sonar_control = true;


    // initialize publishers
    cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

    // initialize subscribers
    laser_scan_sub_  = nh_.subscribe("/scan", 10, &PioneerWander::laserScanMsgCallBack, this);
    sonar_sub = nh_.subscribe("RosAria/sonar", 10, &PioneerWander::sonarMsgCallBack, this);

    return true;
}

double ClosestRangeLaser(const sensor_msgs::LaserScan::ConstPtr& msg, uint16_t index)
{
	double closestRangeLaser = msg->range_max;

	for (int currIndex = index - 20; currIndex <= index + 20; currIndex++) {
		if(std::isnan(msg->ranges[currIndex]))
			{
			continue;
			}
		else if (msg->ranges[currIndex] < closestRangeLaser) {

			closestRangeLaser = msg->ranges[currIndex];
		}
	}
	return closestRangeLaser;
}

void PioneerWander::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	uint16_t centerIndex = ceil((-scan->angle_min) / scan->angle_increment);
	uint16_t rightIndex = ceil((-23*DEG2RAD - scan->angle_min) / scan->angle_increment);
	uint16_t leftIndex = floor((23*DEG2RAD - scan->angle_min) / scan->angle_increment);

	//ROS_INFO("%d %d %d",centerIndex, leftIndex, rightIndex);

	uint16_t scan_angle[3] = {centerIndex, leftIndex, rightIndex};

	for (int num = 0; num < 3; num++)
	{
		direction_vector_[num] = ClosestRangeLaser(scan, scan_angle[num]);
	}
}

void PioneerWander::sonarMsgCallBack(const sensor_msgs::PointCloud::ConstPtr& sonar)
{
	std::vector<float> distances;
		for(auto itr = sonar->points.begin(); itr != sonar->points.end(); ++ itr) {
			distances.push_back(sqrt(itr->x*itr->x + itr->y * itr->y));
		}


		//find the closest distance between [1]-[7]
		float closestRange = distances[2];

		for(int i = 2; i < SONAR_INDEX_MAX -1 ; i++ ) {
			if(distances[i] < closestRange) {
				closestRange = distances[i];
			}
		}

		if (closestRange <= sonar_front_distance_limit_ || direction_vector_[1] == 0) {
			sonar_control = true;
		} else {sonar_control = false;}

		sonar_direction_vector_[0] = distances[3];
		sonar_direction_vector_[1] = distances[2];
		sonar_direction_vector_[2] = distances[5];

		if(sonar_direction_vector_[0] > distances[4]){
			sonar_direction_vector_[0] = distances[4];
		}

		if(sonar_direction_vector_[1] > distances[1]){
			sonar_direction_vector_[1] = distances[1];
		}

		if(sonar_direction_vector_[2] > distances[6]){
			sonar_direction_vector_[2] = distances[6];
		}
}

void PioneerWander::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}


/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool PioneerWander::controlLoopLaser()
{
  static uint8_t state_num = 0;

  ROS_INFO("%f %f %f",direction_vector_[0],direction_vector_[1],direction_vector_[2]);
  switch(state_num)
  {
    case GET_DIRECTION:
      if (direction_vector_[CENTER] > front_distance_limit_)
      {
        state_num = DRIVE_FORWARD;
        ROS_INFO("%f",direction_vector_[0]);
      }

      if (direction_vector_[CENTER] < front_distance_limit_ || direction_vector_[LEFT] < side_distance_limit_)
      {
        state_num = RIGHT_TURN;
      }
      else if (direction_vector_[RIGHT] < side_distance_limit_)
      {
        state_num = LEFT_TURN;
      }
      break;

    case DRIVE_FORWARD:
      //ROS_INFO("Forward");
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      state_num = GET_DIRECTION;
      break;

    case RIGHT_TURN:
          updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
          state_num = GET_DIRECTION;
      break;

    case LEFT_TURN:
          updatecommandVelocity(0.0, ANGULAR_VELOCITY);
          state_num = GET_DIRECTION;
      break;

    default:
      state_num = GET_DIRECTION;
      break;
  }

  return true;
}

bool PioneerWander::controlLoopSonar()
{
	if(!sonar_control) {
		this->controlLoopLaser();
		return false;
	}

	static uint8_t sonar_state_num = 0;

	  //ROS_INFO("%f %f %f",sonar_direction_vector_[0],sonar_direction_vector_[1],sonar_direction_vector_[2]);
	  switch(sonar_state_num)
	  {
	    case GET_DIRECTION:
	      if (sonar_direction_vector_[CENTER] > sonar_front_distance_limit_)
	      {
	    	sonar_state_num = DRIVE_FORWARD;
	        //ROS_INFO("%f",sonar_direction_vector_[0]);
	      }

	      if (sonar_direction_vector_[CENTER] < sonar_front_distance_limit_ || sonar_direction_vector_[LEFT] < sonar_side_distance_limit_)
	      {
	    	  sonar_state_num = RIGHT_TURN;
	      }
	      else if (sonar_direction_vector_[RIGHT] < sonar_side_distance_limit_)
	      {
	    	  sonar_state_num = LEFT_TURN;
	      }
	      break;

	    case DRIVE_FORWARD:
	      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
	      sonar_state_num = GET_DIRECTION;
	      break;

	    case RIGHT_TURN:
	          updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
	          sonar_state_num = GET_DIRECTION;
	      break;

	    case LEFT_TURN:
	          updatecommandVelocity(0.0, ANGULAR_VELOCITY);
	          sonar_state_num = GET_DIRECTION;
	      break;

	    default:
	    	sonar_state_num = GET_DIRECTION;
	      break;
	  }

	return true;
}

bool PioneerWander::controlLoop()
{
   this->controlLoopSonar();
   return true;
}
/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pioneer_wander");
  PioneerWander pioneer_wander;

  ros::Duration(5).sleep();

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
	pioneer_wander.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
