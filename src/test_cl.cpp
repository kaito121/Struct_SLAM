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

std::string win = "main";
ros::Subscriber sub;//データをsubcribeする奴

int h_upper = 115, h_lower = 60;
int s_upper = 255, s_lower = 50;
int v_upper = 200, v_lower = 20;


void callback_function(const sensor_msgs::Image::ConstPtr& msg)//画像トピックが更新されたらよばれるやつ//センサーデータがmsgに入る
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    }

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入


//ここに処理項目
	cv::Mat img_src = image;
    cv::namedWindow(win);

    cv::KalmanFilter KF(4,2);
    KF.statePre.at<float>(0)=0;
    KF.statePre.at<float>(1)=0;
    KF.statePre.at<float>(2)=0;
    KF.statePre.at<float>(3)=0;

    KF.transitionMatrix = (cv::Mat_<float>(4,4)<<
    1,0,1,0,
    0,1,0,1,
    0,0,1,0,
    0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));

    cv::Mat img_hsv, img_gray, img_gray_th, img_bin, img_lbl, img_dst, img_rgb_th;

    cv::Mat element8 = (cv::Mat_<uchar>(3,3) <<1,1,1,1,1,1,1,1,1);

        //while(1){
        std::vector<cv::Mat> vec_hsv(3);

        cv::cvtColor(img_src,img_gray,cv::COLOR_BGR2GRAY);
        cv::cvtColor(img_src,img_hsv,cv::COLOR_BGR2HSV_FULL);
        cv::split(img_hsv,vec_hsv);

        cv::inRange(img_hsv,cv::Scalar(h_lower,s_lower,v_lower),cv::Scalar(h_upper,s_upper,v_upper),img_bin);

        cv::erode(img_bin, img_bin, element8, cv::Point(-1,-1),5);
        cv::dilate(img_bin, img_bin, element8, cv::Point(-1,-1),5);

        cv::Mat stats,centroids;
        int labelnum = cv::connectedComponentsWithStats(img_bin, img_lbl, stats, centroids);

        //if(labelnum == 1) continue;
        if(labelnum == 1) ;
        long int max_area = 0,max_index=0;
        for(int i=1;i<labelnum;i++){
            int area = stats.at<int>(i,cv::CC_STAT_AREA);

            if(area>max_area){
                max_area=area;
                max_index = i;
            }
        }
        cv::compare(img_lbl, max_index,img_dst, cv::CMP_EQ);

        cv::Moments m = cv::moments(img_dst,true);
        cv::Point pos(m.m10/m.m00,m.m01/m.m00);

        cv::Mat measurement(2,1,CV_32F);
        measurement.at<float>(0)=pos.x;
        measurement.at<float>(1)=pos.y;

        cv::Mat correction = KF.correct(measurement);

        cv::Mat prediction = KF.predict();

        cv::circle(img_src,pos,5,cv::Scalar(0,0,255),-1);

        cv::circle(img_src,cv::Point(prediction.at<float>(0),prediction.at<float>(1)),5,cv::Scalar(0,255,255),-1);
        cv::ellipse(img_src,cv::Point(prediction.at<float>(0),prediction.at<float>(1)),
        cv::Size(abs(prediction.at<float>(2)),abs(prediction.at<float>(3))),0.0,0.0,360.0,cv::Scalar(0,255,255),3);

        cv::imshow(win,img_src);
        //if(cv::waitKey(1)=='q') break;
        cv::waitKey(1);

   //}

   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/robot1/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録

	ros::spin();//トピック更新待機
			
	return 0;
}


/*#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

std::string win = "main";

int h_upper = 115, h_lower = 60;
int s_upper = 255, s_lower = 50;
int v_upper = 200, v_lower = 20;

int main(){

    cv::VideoCapture cap("lego2_small.wmv");
    cv::Mat img_src;
    cv::namedWindow(win);

    cv::KalmanFilter KF(4,2);
    KF.statePre.at<float>(0)=0;
    KF.statePre.at<float>(1)=0;
    KF.statePre.at<float>(2)=0;
    KF.statePre.at<float>(3)=0;

    KF.transitionMatrix = (cv::Mat_<float>(4,4)<<
    1,0,1,0,
    0,1,0,1,
    0,0,1,0,
    0,0,0,1);

    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));

    cv::Mat img_hsv, img_gray, img_gray_th, img_bin, img_lbl, img_dst, img_rgb_th;

    cv::Mat element8 = (cv::Mat_<uchar>(3,3) <<1,1,1,1,1,1,1,1,1);

    while(1){
        std::vector<cv::Mat> vec_hsv(3);
        cap >> img_src;

        cv::cvtColor(img_src,img_gray,cv::COLOR_BGR2GRAY);
        cv::cvtColor(img_src,img_hsv,cv::COLOR_BGR2HSV_FULL);
        cv::split(img_hsv,vec_hsv);

        cv::inRange(img_hsv,cv::Scalar(h_lower,s_lower,v_lower),cv::Scalar(h_upper,s_upper,v_upper),img_bin);

        cv::erode(img_bin, img_bin, element8, cv::Point(-1,-1),5);
        cv::dilate(img_bin, img_bin, element8, cv::Point(-1,-1),5);

        cv::Mat stats,centroids;
        int labelnum = cv::connectedComponentsWithStats(img_bin, img_lbl, stats, centroids);

        if(labelnum == 1) continue;
        long int max_area = 0,max_index=0;
        for(int i=1;i<labelnum;i++){
            int area = stats.at<int>(i,cv::CC_STAT_AREA);

            if(area>max_area){
                max_area=area;
                max_index = i;
            }
        }
        cv::compare(img_lbl, max_index,img_dst, cv::CMP_EQ);

        cv::Moments m = cv::moments(img_dst,true);
        cv::Point pos(m.m10/m.m00,m.m01/m.m00);

        cv::Mat measurement(2,1,CV_32F);
        measurement.at<float>(0)=pos.x;
        measurement.at<float>(1)=pos.y;

        cv::Mat correction = KF.correct(measurement);

        cv::Mat prediction = KF.predict();

        cv::circle(img_src,pos,5,cv::Scalar(0,0,255),-1);

        cv::circle(img_src,cv::Point(prediction.at<float>(0),prediction.at<float>(1)),5,cv::Scalar(0,255,255),-1);
        cv::ellipse(img_src,cv::Point(prediction.at<float>(0),prediction.at<float>(1)),
        cv::Size(abs(prediction.at<float>(2)),abs(prediction.at<float>(3))),0.0,0.0,360.0,cv::Scalar(0,255,255),3);

        cv::imshow(win,img_src);
        if(cv::waitKey(1)=='q') break;

    }
    return 0;

}*/
