#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_

#define I2C_BUS 1
#define I2C_ADDRESS 0x40
#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 600
#define SERVO_PULSE_RANGE 4096

#define FW_NUMBER 6

#include <ros/ros.h>
#include <cmg_msgs/SpeedList.h>
#include <cmg_msgs/Speed.h>

#include "PCA9685.h"

class Controller {
	public:
		Controller();

	private:
		double ref_speed;
		double min_speed, max_speed;
		int zero, init_sig;
		double init_time;

		ros::NodeHandle node;
		PCA9685 *controller;

		std::vector<int> ids;
		std::vector<int> cmds;
		double joint_lower_limit, joint_upper_limit, d009a_limit_coef, d150a_limit_coef;
		ros::Subscriber sub_fwset;

		void chatterFW (const cmg_msgs::SpeedListConstPtr &state);

};

#endif /* CONTROLLER_SUB_HPP_ */
