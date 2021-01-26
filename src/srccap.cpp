//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//nameteigi
std::string win_src2 = "src2";//nameteigi
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";
std::string file_src = "test1.png";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180


//コールバック関数

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage,RGBimage2;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;
    }

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;
    }

    RGBimage2 = cv::imread("test1.png",1); // 画像読み込み
    

    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入



//ここに処理項目
	cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(win_src2, cv::WINDOW_AUTOSIZE);
  
	cv::Mat img_dst;
    cv::Mat img_dst1;
    cv::Mat img_dst2;
    cv::Mat img_dst3;
    cv::Mat img_dst4;
    cv::Mat img_dst5;
    cv::Mat img_dst6;
    cv::Mat img_dst7;
    cv::Mat img_dst8;
    cv::Mat img_dst9;


	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow(win_src, RGBimage);
    cv::imshow(win_src2, RGBimage2);
	cv::imwrite(file_src, RGBimage);
	cv::waitKey(1);



   //ros::spinにジャンプする
}

//メイン関数


int main(int argc,char **argv){
	ros::init(argc,argv,"opencv_main");
    	
	ros::NodeHandle nhSub;
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));
    	
	ros::spin();
	
	return 0;
}
