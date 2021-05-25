//RealsenseのRGB画像とDepth画像では画角が違うため画像情報が一致しない
//そのためDepthの画像を調節してRGB画像を一致させる必要がある
//ここではDepth画像をRGB画像に一致するように切り取り、拡大することでRGB画層とDeoth画像の一致を行う

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
#include <librealsense2/rs.hpp>//Realsenseの画角調整用


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

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 0;
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
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

//画像マッチング（パラメータ推定）
  cv::Mat img_prm[2], img_prmw[2], img_match, img_per, img_reg,img_gray;
  cv::Scalar color[2] = { cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0) };

  cv::Mat img_src = bridgeRGBImage->image.clone();
  cv::Mat img_depth = depthimage;

  /*//画像クロップ(近距離でほぼ一致)
  cv::Rect roi(cv::Point(93, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
  cv::Mat img_dst = img_depth(roi); // 切り出し画像
  cv::Mat img_dst2;*/

    //画像クロップ(中距離でほぼ一致)
  cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
  cv::Mat img_dst = img_depth(roi); // 切り出し画像
  cv::Mat img_dst2;

  resize(img_dst, img_dst2,cv::Size(), 1.6, 1.6);//クロップした画像を拡大


cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

//FLD変換
std::vector<cv::Vec4f> lines;
cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
 fld->detect( img_gray, lines);//特徴線検索

 //FLDの線描写
for(int i = 0; i < lines.size(); i++){
    cv::line(img_src,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
    //cv::line(img_depth,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
    cv::line(img_dst2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.8, cv::LINE_AA);
    }


  





    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_depth, img_depth);
    //cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);


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

 
	ros::spin();//トピック更新待機
			
	return 0;
}