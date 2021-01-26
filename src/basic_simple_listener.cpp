//サブスクライブプログラミング
#include<ros/ros.h>
#include<std_msgs/String.h>

void chatterCallback(const std_msgs::String& msg)//ここでmsgを受け取ってる受信部分
{
    ROS_INFO("subscribe: %s",msg.data.c_str());
}

int main(int argc, char** argv){
    ros::init(argc,argv,"basic_simple_listener");
    ros::NodeHandle nh;
    //ここはsubの設定部分 chatterという名前のトピックを受け取る
    ros::Subscriber sub = nh.subscribe("chatter",10,chatterCallback);

    ros::spin();

    return 0;
}
