#include "flywheel_driver.hpp"


Controller::Controller()
	: node("~") {
	int freq;

	node.param("flywheel_zero_signal", zero, 245);
	node.param("flywheel_init_signal", init_sig, 215);
	node.param("flywheel_init_time", init_time, 0.5);
	node.param("pwm_freq", freq, 60);
	node.param("flywheel_min_speed", min_speed, 5000.);
	node.param("flywheel_max_speed", max_speed, 14000.);
	node.getParam("flywheel_ids", ids);
	if (ids.size() < 6) {
		ROS_WARN("flywheel_ids is too small! Check config!");	
	}
	node.getParam("flywheel_calibrations", cmds);
	if (cmds.size() < 6) {
		ROS_WARN("flywheel_calibrations is too small! Check config!");	
	}
	node.param("flywheel_calibrated_speed", ref_speed, 6000.);

	controller = new PCA9685(I2C_BUS, I2C_ADDRESS);
	controller->setPWMFreq(freq);
	sub_fwset = node.subscribe("/fw/cmd", 100, &Controller::chatterFW, this);
	ROS_INFO("Flywheel controller is ready...");
	ROS_INFO("Calibrated for %f rpm", ref_speed);

	
	for (int i : ids) {
		controller->setPWM(i + 1, 0,  zero);
	}
	ros::Duration(init_time).sleep();
	for (int i : ids) {
		controller->setPWM(i + 1, 0,  init_sig);
	}
	ros::Duration(init_time).sleep();
	for (int i : ids) {
		controller->setPWM(i + 1, 0,  zero);
	}
}

void Controller::chatterFW (const cmg_msgs::SpeedListConstPtr &state){
	for (const cmg_msgs::Speed& speed : state->speeds) {
		float rs = speed.speed;
		uint8_t id = ids[speed.id];
		int req_speed = zero;
		if (rs >= min_speed) {
			if (rs > max_speed) { rs = max_speed; }
			req_speed = (int)(rs*cmds[speed.id]/ref_speed);
		}
		if (speed.id < FW_NUMBER) { 
			ROS_INFO("CMD Flywheel on pin %d to speed %f (%d)",  id, rs, req_speed);
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
