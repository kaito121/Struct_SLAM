//多重インクルード防止
#ifndef INCLUDE_DETECT_PERSON_HOG
#define INCLUDE_DETECT_PERSON_HOG
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>


class detectPersonHOG {
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		//ros::CallbackQueue queue;
        cv_bridge::CvImagePtr bridgeImage;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        //使用データ
        std::vector<cv::Rect> found;
        cv::Mat gray_image;
        cv::Mat image;

    /* data */
    public:
        detectPersonHOG(/* args */);
        ~detectPersonHOG();
        void detect(); //人間を検出
        bool subscribeSensorData();//データ受信
        void sensor_callback(const sensor_msgs::ImageConstPtr& msg); //データ受信
        void publishData(); //データ送信
        cv::HOGDescriptor hog;
};

#endif

