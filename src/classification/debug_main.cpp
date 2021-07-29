#include <ros/ros.h>
#include <struct_slam/classification.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"struct_slam_cc");
	
    classificationClass cc; //pan_tilt_control_instance
	// while(ros::ok())
	// {
	// 	ROS_INFO("subscribeSensorDataCamera");
	// 	cc.subscribeSensorDataCamera();
	// 	if(!cc.isCameraMapData()){
	// 		ROS_INFO("No camera map data");
	// 		continue;
	// 	}
	// 	ROS_INFO("classificationDBSCAN");
	// 	cc.classificationDBSCAN();
	// 	if(!cc.isClassificationData()){
	// 		ROS_INFO("No Classification data");
	// 	}
	// 	ROS_INFO("publishClassificationData");
	// 	cc.publishClassificationData();
	// 	ROS_INFO("showCluster");
	// 	cc.showCluster();
	// 	ROS_INFO("clearMessages");
	// 	cc.clearMessages();
		
	// }
    ros::spin();
	return 0;
}