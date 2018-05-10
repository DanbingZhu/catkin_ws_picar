#include <unistd.h>
#include "sound_source_follower/PCA9685.h"
#include "wiringPi.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "hark_msgs/HarkSource.h"

class servoTuner {
private:
	PCA9685 * pwmContrl_;
public:
	servoTuner(){
		pwmContrl_ = new PCA9685(1, 0x40);
  	pwmContrl_->setPWMFreq(50);
  	pwmContrl_->setPWM(1, 375);
	}

	~servoTuner(){
		delete pwmContrl_;
	}

	void callback(const std_msgs::Int32::ConstPtr& msg){
				pwmContrl_->setPWM(1, 405 + msg->data);
				ROS_INFO("Current PWM value is: %d", 405 + msg->data);
	}

};


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "servo_tuner");
  ros::NodeHandle n;

	servoTuner tuner;
  ros::Subscriber sub = n.subscribe("servo_tune", 1000, &servoTuner::callback, &tuner);

  ros::spin();

  return 0;
}
