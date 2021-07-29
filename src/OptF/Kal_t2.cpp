//hsvで色でフィルタを作りあるしきい値以上の色のみをマスクで取り出す
//その後取り出した画像に対し膨張圧縮処理を施しノイズ除去
//マスク処理してある１面積のみにした画像に対し重心を求め、その重心の移動値からカルマンフィルタを使い位置を推定する
//つまりこのプログラムは１つのマークに対してのみしか処理が行われていない
//このプログラムはリアルタイムでコロコロの緑色部分を追跡するプログラム(20210701)
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
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_lbl = "lbl";
std::string win_hsv = "hsv";
std::string win_bin1 = "bin1";
std::string win_bin3 = "bin3";
std::string win_max_index = "max_index";
std::string win_dst2 = "dst2";
//std::string win = "main";

//int h_upper = 115, h_lower = 60;
//int s_upper = 255, s_lower = 50;
//int v_upper = 200, v_lower = 20;

int h_upper = 66, h_lower = 60;//掃除道具のコロコロの柄の部分の緑色を追跡する
int s_upper = 255, s_lower = 50;
int v_upper = 255, v_lower = 20;
cv::Mat img_src,img_dst2;
int kaisu=0,kaisu2=0;
cv::KalmanFilter KF(4, 2);

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image,img_depth2,img_depth3,img_depth4;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
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

  img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
  depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

  img_src.copyTo(img_dst2);


  if(kaisu==0){
  //cv::KalmanFilter KF(4, 2);
  KF.statePre.at<float>(0) = 0;
  KF.statePre.at<float>(1) = 0;
  KF.statePre.at<float>(2) = 0;
  KF.statePre.at<float>(3) = 0;

  // 運動モデル
  KF.transitionMatrix = (cv::Mat_<float>(4, 4) << // 等速直線運動（速度利用あり）
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);

  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));
  setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
  setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));}

  cv::Mat img_hsv, img_gray, img_gray_th, img_bin1,img_bin2 ,img_bin3,img_lbl, img_dst, img_rgb_th;
  cv::Mat element8 = (cv::Mat_<uchar>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1); // 8近傍


    std::vector<cv::Mat> vec_hsv(3);

    // 追跡対象の抽出
    cv::cvtColor(img_src, img_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_src, img_hsv, cv::COLOR_BGR2HSV_FULL);//hsv画像化
    cv::split(img_hsv, vec_hsv);
    ROS_INFO("callback_functionが呼ばれたよ2");
    //std::cout <<"vec_hsv="<<vec_hsv<<std::endl;
    cv::imshow(win_hsv, img_hsv);

    // HSV閾値処理（あるしきい値以上の色のみを二値化で抽出）
    cv::inRange(img_hsv, cv::Scalar(h_lower, s_lower, v_lower), cv::Scalar(h_upper, s_upper, v_upper), img_bin1);
    ROS_INFO("callback_functionが呼ばれたよ3");
    cv::imshow(win_bin1, img_bin1);

    // ノイズ処理（抽出した画像に対しノイズ処理）
    //cv::dilate(img_bin1, img_bin3, element8, cv::Point(-1, -1), 5); // 膨張オンリー

    cv::erode(img_bin1, img_bin2, element8, cv::Point(-1, -1), 5); // 収縮
    cv::dilate(img_bin2, img_bin3, element8, cv::Point(-1, -1), 5); // 膨張
    ROS_INFO("callback_functionが呼ばれたよ4");
    cv::imshow(win_bin3, img_bin3);

    // 面積最大ラベルの選択
    cv::Mat stats, centroids;
    ROS_INFO("callback_functionが呼ばれたよ5");
    int labelnum = cv::connectedComponentsWithStats(img_bin3, img_lbl, stats, centroids);
    std::cout <<"labelnum="<<labelnum<<std::endl;
    ROS_INFO("callback_functionが呼ばれたよ6");
    //if (labelnum == 1) continue;
    long int max_area = 0, max_index = 0;
    for (int i = 1; i < labelnum; i++) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      std::cout <<"area="<<area<<std::endl;
      if (area > max_area) {
        max_area = area;
        max_index = i;
      }
    }
    std::cout <<"max_index="<<max_index<<std::endl;
    cv::compare(img_lbl, max_index, img_dst, cv::CMP_EQ);

    // 面積最大ラベルの重心(面積最大を計算しているので一つしか出ない）
    cv::Moments m = cv::moments(img_dst, true);//抽出画像の重心を求める
    cv::Point pos(m.m10 / m.m00, m.m01 / m.m00);//重心計算(x,y)
    std::cout <<"m.m10="<<m.m10<<std::endl;
    std::cout <<"m.m00="<<m.m00<<std::endl;
    std::cout <<"m.m01="<<m.m01<<std::endl;
    std::cout <<"m.m00="<<m.m00<<std::endl;

    // カルマンフィルタ
    // 観測
    cv::Mat measurement(2, 1, CV_32F);
    measurement.at<float>(0) = pos.x;//重心位置x座標
    measurement.at<float>(1) = pos.y;//重心位置y座標
    std::cout <<"pos.x="<<pos.x<<std::endl;
    std::cout <<"pos.y="<<pos.y<<std::endl;

    // 修正（カルマンフィルタ関数）
    cv::Mat correction = KF.correct(measurement);//観測値をカルマンフィルタに代入
    cv::waitKey(1);//ros::spinにジャンプする

    // 予測（カルマンフィルタ関数）
    cv::Mat prediction = KF.predict();
    cv::waitKey(1);//ros::spinにジャンプする

    // 結果の描画
    // 重心位置
    cv::circle(img_src, pos, 5, cv::Scalar(0, 0, 255), -1);//赤点


    // 予測位置
    cv::circle(img_src, cv::Point(prediction.at<float>(0), prediction.at<float>(1)), 5, cv::Scalar(0, 255, 255), -1);//予測の点（黄色）
    cv::ellipse(img_src, cv::Point(prediction.at<float>(0), prediction.at<float>(1)),
    cv::Size(abs(prediction.at<float>(2))*5, abs(prediction.at<float>(3))*5),0.0, 0.0, 360.0, cv::Scalar(0, 255, 255), 3);//予測の円（黄色）

    std::cout <<"予測座標x="<<prediction.at<float>(0)<<std::endl;
    std::cout <<"予測座標y="<<prediction.at<float>(1)<<std::endl;
    std::cout <<"予測範囲x="<<abs(prediction.at<float>(2))*5<<std::endl;//円のx方向の半径の大きさ
    std::cout <<"予測範囲y="<<abs(prediction.at<float>(3))*5<<std::endl;//円のY方向の半径の大きさ

    cv::rectangle(img_src, cv::Rect(prediction.at<float>(0)-abs(prediction.at<float>(2))*5, prediction.at<float>(1)-abs(prediction.at<float>(3))*5, abs(prediction.at<float>(2))*10, abs(prediction.at<float>(3))*10),
    cv::Scalar(255, 255, 255), 3);

    // 画面表示
    //cv::imshow(win, img_src);
    cv::imshow(win_src, img_src);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_lbl, img_lbl);
    kaisu=kaisu+20;
    kaisu2=kaisu2+10;

  cv::waitKey(1);//ros::spinにジャンプする
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
