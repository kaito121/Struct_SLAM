#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
cv::Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv::imshow("image", image);
    cv::waitKey(1);
}
int main(int argc, char** argv) {
    ros::init (argc, argv, "img_subscriber");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/img_publisher/image", 10, imageCallback);
    ros::spin();
    return 0;
}