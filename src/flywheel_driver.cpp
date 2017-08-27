#include "flywheel_driver.hpp"


Controller::Controller()
	: node("~") {
	int freq;

	node.param("flywheel_speed_a", fit_a, 2.11e-7);
	node.param("flywheel_speed_b", fit_b, 1.33e-3);
	node.param("flywheel_speed_c", fit_c, 258.3);
	node.param("flywheel_zero_signal", zero, 245);
	node.param("pwm_freq", freq, 60);
	node.param("flywheel_min_speed", min_speed, 5000.);
	node.param("flywheel_max_speed", max_speed, 14000.);

	controller = new PCA9685(I2C_BUS, I2C_ADDRESS);
	controller->setPWMFreq(freq);
	sub_fwset = node.subscribe("flywheel_driver/speeds", 100, &Controller::chatterFW, this);
	ROS_INFO("Flywheel controller is ready...");
	ROS_INFO("A:%f B:%f C:%f", fit_a, fit_b, fit_c);
}

void Controller::chatterFW (const cmg_msgs::FlywheelSpeedsConstPtr &state){
	for (const cmg_msgs::FlywheelSpeed& speed : state->speeds) {
		float rs = speed.speed;
		uint8_t id = speed.id;
		int req_speed = zero;
		if (rs >= min_speed) {
			if (rs > max_speed) { rs = max_speed; }
			req_speed = (int)(fit_a*rs*rs + fit_b*rs + fit_c);
		}
		if (id < FW_NUMBER) { 
			ROS_INFO("CMD Flywheel %d to speed %f (%d)",  id, rs, req_speed);
			controller->setPWM(id + 1, 0,  req_speed);
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "flywheel_driver");
	Controller c;
	ros::spin();
}
