//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>



ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_fld = "fld";

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
    cv::Mat img_gray,img_edge,img_dst,img_FLD,img_line;
    double theta[1000],theta0,theta90;

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    img_src.copyTo(img_FLD);
    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::line(img_FLD,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青水平
    cv::line(img_FLD,cv::Point(100,0),cv::Point(100,200),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑垂直

    //FLD変換
    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines);//特徴線検索

    for(int i = 0; i < lines.size(); i++){
       cv::line(img_FLD,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
       cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA); 
    }

    //FLD抽出線座標とY軸との角度を求める
    for(int i = 0; i < lines.size(); i++){

        cv::circle(img_dst,Point(lines[i][0],lines[i][1]),5,Scalar(255,0,0),-1);//青点
        cv::circle(img_dst,Point(lines[i][2],lines[i][3]),5,Scalar(0,255,0),-1);//緑点

         std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<")"<< std::endl;
         std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<")"<< std::endl;

         //FLD抽出線のy軸との角度を求める
         theta[i]=M_PI-atan2((lines[i][2]-lines[i][0]),(lines[i][3]-lines[i][1]));
         std::cout <<"FLD抽出線の傾きl["<<i<<"]("<<theta[i]<<")"<< std::endl;
        }

  //thetaの数値を小さいにソート
  double tmp,tmp1x,tmp1y,tmp2x,tmp2y,tmpdep1,tmpdep2;
  for (int i=0; i<=lines.size(); ++i) {
     for (int j=i+1;j<lines.size(); ++j) {
         if (theta[i] > theta[j]) {
             tmp =  theta[i];
             tmp1x =  lines[i][0];
             tmp1y =  lines[i][1];
             tmp2x =  lines[i][2];
             tmp2y =  lines[i][3];
            
             theta[i] = theta[j];
             lines[i][0] = lines[j][0];
             lines[i][1] = lines[j][1];
             lines[i][2] = lines[j][2];
             lines[i][3] = lines[j][3];
             
             theta[j] = tmp;
             lines[j][0] = tmp1x;
             lines[j][1] = tmp1y;
             lines[j][2] = tmp2x;
             lines[j][3] = tmp2y;
            }
        }
    }
    std::cout <<"水平線の傾きl("<<theta0<<")"<< std::endl;
    std::cout <<"垂直線の傾きl("<<theta90<<")"<< std::endl;

    std::cout <<"並び替え後"<<std::endl;
    for (int i=0; i<=lines.size(); ++i) {
        std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<")"<< std::endl;
        std::cout <<"FLD抽出線の傾きl2["<<i<<"]("<<theta[i]<<")"<< std::endl;
    }
    for (int i=0; i<=lines.size(); ++i) {
    std::cout <<theta[i]<< std::endl;
    }

    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_fld, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_fld, img_FLD);
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