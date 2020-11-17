//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>
#include <math.h>
#include <highgui.h>


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";

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
    cv::Mat img_gray,img_edge,img_dst;
    

    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::Canny(img_gray, img_edge, 200, 200);

    //標準的ハフ変換
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(img_edge, lines1, 1, CV_PI/180, 120);

    //確率的ハフ変換
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 80,30,10);
    
    for(int i = 0; i < lines.size(); i++){
       cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);  
    }

    /*for(int i = 0; i < lines1.size(); i++){
        double rho = lines1[i][0], theta = lines1[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double pt1x,pt1y,pt2x,pt2y;
        pt1x = x0 - img_dst.cols*b ,pt1y = y0 + img_dst.cols*a;
        pt2x = x0 + img_dst.cols*b ,pt2y = y0 - img_dst.cols*a;
        
        cv::line(img_dst,cv::Point(pt1x,pt1y),cv::Point(pt2x,pt2y),cv::Scalar(0,255,0), 2, cv::LINE_AA);  

    }*/

        double rho = 0, theta = 3.14/2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double pt1x,pt1y,pt2x,pt2y;
        pt1x = x0 - img_dst.cols*b ,pt1y = y0 + img_dst.cols*a;
        pt2x = x0 + img_dst.cols*b ,pt2y = y0 - img_dst.cols*a;
        
        cv::line(img_dst,cv::Point(pt1x,pt1y),cv::Point(pt2x,pt2y),cv::Scalar(0,255,0), 8, cv::LINE_AA);  //緑

        double rho1 = 0, theta1 = 0;
        double a1 = cos(theta1), b1 = sin(theta1);
        double x01 = a1*rho1, y01 = b1*rho1;
        double pt1x1,pt1y1,pt2x1,pt2y1;
        pt1x1 = x01 - img_dst.cols*b1 ,pt1y1 = y01 + img_dst.cols*a1;
        pt2x1 = x01 + img_dst.cols*b1 ,pt2y1 = y01 - img_dst.cols*a1;
        
        cv::line(img_dst,cv::Point(pt1x1,pt1y1),cv::Point(pt2x1,pt2y1),cv::Scalar(0,0,255), 8, cv::LINE_AA); //赤 

        double rho2 = 100, theta2 = 3.14/2;
        double a2 = cos(theta2), b2 = sin(theta2);
        double x02 = a2*rho2, y02 = b2*rho2;
        double pt1x2,pt1y2,pt2x2,pt2y2;
        pt1x2 = x02 - img_dst.cols*b2 ,pt1y2 = y02 + img_dst.cols*a2;
        pt2x2 = x02 + img_dst.cols*b2 ,pt2y2 = y02 - img_dst.cols*a2;
        
        cv::line(img_dst,cv::Point(pt1x2,pt1y2),cv::Point(pt2x2,pt2y2),cv::Scalar(255,0,0), 8, cv::LINE_AA);  //青

        double rho3 = 100, theta3 = 0;
        double a3 = cos(theta3), b3 = sin(theta3);
        double x03 = a3*rho3, y03 = b3*rho3;
        double pt1x3,pt1y3,pt2x3,pt2y3;
        pt1x3 = x03 - img_dst.cols*b3 ,pt1y3 = y03 + img_dst.cols*a3;
        pt2x3 = x03 + img_dst.cols*b3 ,pt2y3 = y03 - img_dst.cols*a3;
        
        cv::line(img_dst,cv::Point(pt1x3,pt1y3),cv::Point(pt2x3,pt2y3),cv::Scalar(255,255,255), 8, cv::LINE_AA); //白

        double rho4 = 100*sqrt(2), theta4 = 3.14/4;
        double a4 = cos(theta4), b4 = sin(theta4);
        double x04 = a4*rho4, y04 = b4*rho4;
        double pt1x4,pt1y4,pt2x4,pt2y4;
        pt1x4 = x04 - img_dst.cols*b4 ,pt1y4 = y04 + img_dst.cols*a4;
        pt2x4 = x04 + img_dst.cols*b4 ,pt2y4 = y04 - img_dst.cols*a4;
        
        cv::line(img_dst,cv::Point(pt1x4,pt1y4),cv::Point(pt2x4,pt2y4),cv::Scalar(0,255,255), 8, cv::LINE_AA); //黄色

        double rho5 = -100, theta5 = 3.14/4+3.14/2;
        double a5 = cos(theta5), b5 = sin(theta5);
        double x05 = a5*rho5, y05 = b5*rho5;
        double pt1x5,pt1y5,pt2x5,pt2y5;
        pt1x5 = x05 - img_dst.cols*b5 ,pt1y5 = y05 + img_dst.cols*a5;
        pt2x5 = x05 + img_dst.cols*b5 ,pt2y5 = y05 - img_dst.cols*a5;
        
        cv::line(img_dst,cv::Point(pt1x5,pt1y5),cv::Point(pt2x5,pt2y5),cv::Scalar(255,0,255), 8, cv::LINE_AA);  //紫


    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
	cv::waitKey(1);



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


