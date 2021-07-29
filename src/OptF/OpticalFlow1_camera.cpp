//20210527  
//画像座標の変化具合を知るためにオプティカルフローを使う
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>//reconfig用
#include <mutex>
//#include <OpenCV1/wakuhairetu.h>//自作メッセージ用ヘッダ
#include <algorithm>//並び替え用
#include <math.h>
#include <stdlib.h>//絶対値用関数
#include <highgui.h>
#include <visualization_msgs/Marker.h>//ラインマーカー用
#include <cmath>
#include <struct_slam/MaskImageData.h>//パッケージ名要変更（自分で作ったデータを取り込んで）
#include <struct_slam/ImageMatchingData.h>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <Eigen/Dense>//Eigen用
#include <Eigen/Core>//cvMat->Eigen変換用
#include <opencv2/core/core.hpp>//cvMat->Eigen変換用
#include <opencv2/core/eigen.hpp>//cvMat->Eigen変換用


#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>



ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher marker_pub;
ros::Publisher marker_pub_W;
// ros::Publisher image_pub;
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_dst2 = "dst2";
std::string win_depth = "depth";
std::string win_line = "line";


using namespace std;
using namespace cv;
using namespace Eigen;
using Eigen::MatrixXd;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 0;
int sec=0;
cv::Mat RGBimage3;//ここで定義する必要がある

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
    

	//ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示

	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	//画像マッチング（パラメータ推定）
  	cv::Mat img_prm[2], img_prmw[2], img_match, img_per, img_reg;
  	cv::Scalar color[2] = { cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0) };

   // 画像読み込み(初回はこっち)
    if(kaisu==0){
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage;}// 画像読み込み
	//二回目以降
	else{
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage3;} // 画像読み込み
	cv::rectangle(img_prm[0], cv::Point(0, 0), cv::Point(img_prm[0].cols, img_prm[0].rows), color[0], 2); // 外枠
	cv::rectangle(img_prm[1], cv::Point(0, 0), cv::Point(img_prm[1].cols, img_prm[1].rows), color[1], 2); // 外枠
	img_prmw[0] = cv::Mat::zeros(img_prm[0].size() * 2, img_prm[0].type());
	img_prmw[1] = cv::Mat::zeros(img_prm[1].size() * 2, img_prm[1].type());
	cv::Mat roi1 = img_prmw[0](cv::Rect(img_prmw[0].cols / 4, img_prmw[0].rows / 4, img_prm[0].cols, img_prm[0].rows));
	cv::Mat roi2 = img_prmw[1](cv::Rect(img_prmw[1].cols / 4, img_prmw[1].rows / 4, img_prm[1].cols, img_prm[1].rows));
	img_prm[0].copyTo(roi1); // 縦横倍のMatの中央にコピー
	img_prm[1].copyTo(roi2); // 縦横倍のMatの中央にコピー

		// オプティカルフローの計算手法の決定
	//cv::Ptr<cv::superres::DenseOpticalFlowExt> opticalFlow = cv::superres::createOptFlow_Farneback();     // Franeback
	cv::Ptr<cv::superres::DenseOpticalFlowExt> opticalFlow = cv::superres::createOptFlow_DualTVL1();		   // TVL1


		// オプティカルフローの計算
			cv::Mat flow_x, flow_y;
			opticalFlow->calc(img_prm[0], img_prm[1], flow_x, flow_y);

			// オプティカルフローの2次元ベクトルの角度と大きさを算出
			cv::Mat magnitude, angle;
			cv::cartToPolar(flow_x, flow_y, magnitude, angle, true);

			// 大きさを正規化
			normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX);

			// 表示
			cv::Mat display_img[2];
			cv::Mat result;
			display_img[0] = img_prm[1];
			magnitude.convertTo(magnitude, CV_8UC1);
			display_img[1] = magnitude;
			vconcat(display_img, 2, result);     // 縦方向に結合
			//cv::imshow("Result", result);


    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_result, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_result, result);
    //cv::imshow(win_dst, img_dst);
    //cv::imshow(win_dst2, img_dst2);
    //cv::imshow(win_depth, img_depth4);
    //cv::imshow(win_line, img_line);

	// 前のフレームを保存
	img_prm[0] = img_prm[1];

    RGBimage3 = bridgeRGBImage->image.clone();//一つ前の画像をメモリー保存
    kaisu=kaisu+1;
    sec=sec+1;

  	cv::waitKey(1);
   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub_W = n.advertise<visualization_msgs::Marker>("Would_marker", 10);
  // image_pub = n.advertise<sensor_msgs::Image>("maskImageData", 10);//realsenseの画像データをパブリッシュ


	ros::spin();//トピック更新待機
			
	return 0;
}