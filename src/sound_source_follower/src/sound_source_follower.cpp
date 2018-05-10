#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "hark_msgs/HarkSource.h"

#define HISTORY_LENGTH 20

class soundSourceFollower {
private:
	std::vector<int> history_;
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	std_msgs::Int32 pubVal_;
public:
	soundSourceFollower() : history_(HISTORY_LENGTH, 0){
		pub_ = n_.advertise<std_msgs::Int32>("filtered_bearing", 1000);
		sub_ = n_.subscribe("hark_source", 1000, &soundSourceFollower::harkCallback, this);
	}

	~soundSourceFollower(){
		
	}

	void harkCallback(const hark_msgs::HarkSource::ConstPtr& msg){
  	ROS_INFO("Exists source num: %d\n", msg->exist_src_num);
  	if(msg->exist_src_num < 1){
    	ROS_INFO("Existing number of source less than 1\n");
			pubVal_.data = 255;
			pub_.publish(pubVal_);
  	} else {
			int minid = 10000;
			int minidelem = 0;
			int historyMedian = 0;
			for(int num = 0; num < msg->exist_src_num; num++){
				if(msg->src[num].id > minid) continue;
				minidelem = num;
				minid = msg->src[num].id;
			}
	
			ROS_INFO("Received [ID, Azimuth] [%d, %f]\n",  msg->src[minidelem].id, msg->src[minidelem].azimuth);
			historyMedian = temporalMedianFilter(int(msg->src[minidelem].azimuth));		
			std::string content = "[ ";
			for(int i = 0; i < HISTORY_LENGTH; i++){
				content = content + std::to_string(history_[i]) + ", "; 
			}
			content = content + "]";
			ROS_INFO("%s", content.c_str());
			ROS_INFO("History Median: [%d]\n", historyMedian);

			// publish the filtered bearing info to corresponding topic
			pubVal_.data = historyMedian;
			pub_.publish(pubVal_);

		}
	}

	int temporalMedianFilter(int newReading){
		for(int i = HISTORY_LENGTH - 1; i > 0; i--){
			history_[i] = history_[i - 1];
		}

		history_[0] = newReading;

		std::vector<int> temp = history_;
		std::sort(temp.begin(), temp.end());

		if(HISTORY_LENGTH % 2 == 0){
			return ((temp[HISTORY_LENGTH/2] + temp[HISTORY_LENGTH/2 + 1]) / 2);
		} else {
			return temp[HISTORY_LENGTH/2 + 1];
		}

	}

};


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "sound_source_follower");

	soundSourceFollower follower;

  ros::spin();

  return 0;

}
