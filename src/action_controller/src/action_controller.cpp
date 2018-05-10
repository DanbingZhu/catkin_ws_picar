#include <unistd.h>
#include "action_controller/PCA9685.h"
#include "wiringPi.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#define NEUTRAL_POSITION 405

class actionController {
private:
	PCA9685 * pwmContrl_;	
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	std_msgs::String cmd_;
	
public:
	actionController() {
		pub_ = n_.advertise<std_msgs::String>("buzzer_cmd", 1000);
		sub_ = n_.subscribe("filtered_bearing", 1000, &actionController::actionCallback, this);

		pwmContrl_ = new PCA9685(1, 0x40);
  	pwmContrl_->setPWMFreq(50);
  	pwmContrl_->setPWM(1, NEUTRAL_POSITION);

  	wiringPiSetup();
  	pinMode(0, OUTPUT);
  	pinMode(1, OUTPUT);
  	pinMode(2, OUTPUT);
  	pinMode(3, OUTPUT);
	}

	~actionController(){
		delete pwmContrl_;
	}

	void actionCallback(const std_msgs::Int32::ConstPtr& msg) {
		if(msg->data == 255){
			
			digitalWrite(0, LOW);
			digitalWrite(1, LOW);
			digitalWrite(2, LOW);
			digitalWrite(3, LOW);

			cmd_.data = "sound";
			pub_.publish(cmd_);		

		} else {
			cmd_.data = "mute";
			pub_.publish(cmd_);

			pwmContrl_->setPWM(1, (NEUTRAL_POSITION - 2 * msg->data));
			
			digitalWrite(0, HIGH);
			digitalWrite(1, LOW);
			digitalWrite(2, HIGH);
			digitalWrite(3, LOW);

			pwmContrl_->setPWM(5, 1200 + 10 * msg->data);
			pwmContrl_->setPWM(6, 1200 + 10 * msg->data);
			
		}
	}	

};


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "action_controller");
 
	actionController controller;

  ros::spin();

  return 0;

}
