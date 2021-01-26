//パブリッシュプログラミング
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

int main(int argc,char** argv){

    ros::init(argc, argv, "basic_simple_talker1");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    //ROSのPublisherを作成 ROSトピック名:chatter ROSトピック型:std_msgs::String
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter",10);
    std::string msg_chatter = "hello world";
    pnh.getParam("content",msg_chatter);
 
    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::String msg;
        msg.data=msg_chatter;
        ROS_INFO("%s",msg.data.c_str());
        chatter_pub.publish(msg);//ここでmsgをパブリッシュしてる

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

