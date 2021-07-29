#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <math.h>
#include <highgui.h>

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
int best = 30;

int main(int argc, char** argv)
{  

  //カメラパラメータの読み込みとレンズ歪の除去
  cv::Mat img1; //入力画像1
  cv::Mat img2; //入力画像2
  cv::Mat K,img_match;
  cv::Mat distCoeffs;
  cv::FileStorage fs("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/realsense_para.xml", CV_STORAGE_READ);
  fs["intrinsicMat"] >> K;
  fs["distCoeffs"] >> distCoeffs;
    ROS_INFO("callBack0");//printと秒数表示

  cv::undistort(cv::imread("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/IMG_3273.JPG"), img1, K, distCoeffs);
  cv::undistort(cv::imread("/home/fuji/catkin_ws/src/Struct_SLAM/src/OptF/IMG_3274.JPG"), img2, K, distCoeffs);
    ROS_INFO("callBac1k");//printと秒数表示

  // (1) 特徴点抽出(ORB)
	cv::Ptr<cv::ORB> detector = cv::ORB::create();// FeatureDetectorオブジェクトの生成
	cv::Ptr<cv::ORB> extractor = cv::ORB::create();// DescriptionExtractorオブジェクトの生成
	cv::BFMatcher matcher(cv::NORM_HAMMING); // DescriptorMatcherオブジェクトの生成
  std::vector<cv::KeyPoint> kpts1, kpts2;// 特徴点情報を格納するための変数(KeyPoint)
    ROS_INFO("callBack2");//printと秒数表示
  
  detector->detect(img1, kpts1);// 特徴点抽出の実行
  detector->detect(img2, kpts2);

   cv::Mat desc1, desc2;// 画像の特徴情報を格納するための変数

  extractor->compute(img1, kpts1, desc1);// 特徴記述の計算を実行
  extractor->compute(img2, kpts2, desc2);

  std::vector<cv::DMatch> matches;// 特徴点のマッチング情報を格納する変数
  matcher.match(desc1, desc2, matches);// 特徴点マッチングの実行
	ROS_INFO("maxtutinngu");//printと秒数表示

  std::cout << "best = " << best << std::endl;
	std::cout << "match size = " << matches.size() << std::endl;
	if (matches.size() < best) {
		std::cout << "few matchpoints" << std::endl;
	}
	
	// 上位best個を採用
	std::nth_element(begin(matches), begin(matches) + best - 1, end(matches));
	matches.erase(begin(matches) + best, end(matches));
	std::cout << "matchs size = " << matches.size() << std::endl;
	
	// 特徴点の対応を表示(img_src1,KeyPoint1,img_src2,KeyPoint2,match,img_dst)
	cv::drawMatches(img1, kpts1, img2, kpts2, matches, img_match);


    std::vector<cv::Point2d> p1;
    std::vector<cv::Point2d> p2;
    //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
    for (size_t i = 0; i < matches.size(); ++i)
    {
      cv::Mat ip(3, 1, CV_64FC1);
      cv::Point2d p;

      ip.at<double>(0) = kpts1[matches[i].queryIdx].pt.x;
      ip.at<double>(1) = kpts1[matches[i].queryIdx].pt.y;
      ip.at<double>(2) = 1.0;
      ip = K.inv()*ip;
      p.x = ip.at<double>(0);
      p.y = ip.at<double>(1);
      p1.push_back( p );

      ip.at<double>(0) = kpts2[matches[i].trainIdx].pt.x;
      ip.at<double>(1) = kpts2[matches[i].trainIdx].pt.y;
      ip.at<double>(2) = 1.0;
      ip = K.inv()*ip;
      p.x = ip.at<double>(0);
      p.y = ip.at<double>(1);
      p2.push_back( p );
    }

    cv::Mat essentialMat = cv::findEssentialMat(p1, p2);
    std::cout << "Essential Matrix\n" << essentialMat << std::endl;

    cv::Mat r, t;
    cv::recoverPose(essentialMat, p1, p2, r, t);
    std::cout << "R:\n" << r << std::endl;
    std::cout << "t:\n" << t << std::endl;
  
  return 0;
}
