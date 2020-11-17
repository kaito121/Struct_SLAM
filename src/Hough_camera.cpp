//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>

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

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(img_edge, lines, 1, CV_PI/180, 120);
    
    for(int i = 0; i < lines.size(); i++){
        double rho = lines[i][0], theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double pt1x,pt1y,pt2x,pt2y;
        pt1x = x0 - img_dst.cols*b ,pt1y = y0 + img_dst.cols*a;
        pt2x = x0 + img_dst.cols*b ,pt2y = y0 - img_dst.cols*a;

        std::cout <<"ライン"<<i<< std::endl;
        std::cout <<"pt1x["<<i<<"]="<<pt1x<< std::endl;
        std::cout <<"pt1y["<<i<<"]="<<pt1y<< std::endl;
        std::cout <<"pt2x["<<i<<"]="<<pt2x<< std::endl;
        std::cout <<"pt2y["<<i<<"]="<<pt2y<< std::endl;
        std::cout <<"pt1["<<i<<"]("<<pt1x<<","<<pt1y<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<pt2x<<","<<pt2y<<")"<< std::endl;
        

        cv::line(img_dst,
            cv::Point(pt1x,pt1y),
            cv::Point(pt2x,pt2y),
            cv::Scalar(i,i*10,255), 2, cv::LINE_AA);

        cv::circle(img_dst,Point(pt1x,pt1y),10,Scalar(255,0,0),-1);//青点
        cv::circle(img_dst,Point(pt2x,pt2y),10,Scalar(0,255,0),-1);//緑点
        //cv::circle(img_dst,Point(x0,y0),10,Scalar(255,0,255),-1);//青点
        
        line(img_dst,Point(image.size().width/2,0),Point(image.size().width/2,image.size().height),Scalar(255,0,255),2);//画像の中心線

    
        

    }
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