//パブリッシュプログラミング
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc,char** argv){

    ros::init(argc, argv, "basic_simple_talker");
    ros::NodeHandle nh;

    //ROSのPublisherを作成
    //ROSトピック名:chatter 
    //ROSトピック型:std_msgs::String
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter",10);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        std_msgs::String msg;
        msg.data="hello world!";
        ROS_INFO("publish: %s",msg.data.c_str());
        chatter_pub.publish(msg);//ここでmsgをパブリッシュしてる

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

