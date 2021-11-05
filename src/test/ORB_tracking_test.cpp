//特徴点検出のテストプログラム
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>//処理の時間を出力する
#include <sys/time.h>


#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_point = "point";

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.15f;

cv::Mat img_dst,image_curr,image_prev;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
int kaisu;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    printf("処理の時間=%f\n", realsec);
  }

//サンプリング時間取得(ROS)
  ros::WallTime wall_now = ros::WallTime::now();
  ros::WallDuration wall_duration = wall_now - wall_begin;
  ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
  wall_systemtime = wall_duration - wall_prev;
  //ROS_INFO("systemtime:%u.%09u", wall_systemtime.sec, wall_systemtime.nsec);
  std::cout << "wall_systemtime=" <<wall_systemtime<< std::endl;//サンプリング時間

    //変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
//
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;
    }
    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
//  
  	camera_info=*cam_info;//CameraInfo受け取り
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	  img_dst = image.clone();
	  image_curr = image.clone();
    //img_dst = cv::Scalar(255,255,255);
    //cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);
    Mat frame, frame1, imReg, h;

    if(kaisu!=0){
      // Convert images to grayscale
      Mat im1Gray, im2Gray;
      cvtColor(image_curr, im1Gray, CV_BGR2GRAY);
      cvtColor(image_prev, im2Gray, CV_BGR2GRAY);

      // Variables to store keypoints and descriptors
      std::vector<KeyPoint> keypoints1, keypoints2;
      Mat descriptors1, descriptors2;

      // Detect ORB features and compute descriptors.
      Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
      orb->detectAndCompute(im1Gray, Mat(), keypoints1, descriptors1);
      orb->detectAndCompute(im2Gray, Mat(), keypoints2, descriptors2);

      // Match features.
      std::vector<DMatch> matches;
      Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
      matcher->match(descriptors1, descriptors2, matches, Mat());

      // Sort matches by score
      std::sort(matches.begin(), matches.end());

      // Remove not so good matches(Lowe ratio test)
      const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
      matches.erase(matches.begin()+numGoodMatches, matches.end());


      // Draw top matches
      Mat imMatches;
      drawMatches(image_curr, keypoints1, image_prev, keypoints2, matches, imMatches);
      imshow("matches.jpg", imMatches);


      // Extract location of good matches
      std::vector<Point2f> points1, points2;

      for( size_t i = 0; i < matches.size(); i++ )
      {
        points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
        points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
      }

      std::vector<Point2f> obj_corners(4);
      std::vector<Point2f> scene_corners(4);

      obj_corners[0] = cvPoint(24, 44);
      obj_corners[1] = cvPoint( 196, 44);
      obj_corners[2] = Point2f( 196, 210 );
      obj_corners[3] = Point2f( 24, 210 );

      // Find homography
      h = findHomography( points1, points2, RANSAC );

      // Use homography to warp image
      perspectiveTransform(obj_corners, scene_corners, h);

      line(img_dst, scene_corners[0], scene_corners[1], (255, 0, 0), 2, LINE_8, 0);
      line(img_dst, scene_corners[1], scene_corners[2], (255, 0, 0), 2, LINE_8, 0);
      line(img_dst, scene_corners[2], scene_corners[3], (255, 0, 0), 2, LINE_8, 0);
      line(img_dst, scene_corners[3], scene_corners[0], (255, 0, 0), 2, LINE_8, 0);


	
    }
    cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）

    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);
	kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする
}



//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));
  
	ros::spin();//トピック更新待機
			
	return 0;
}