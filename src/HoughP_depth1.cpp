//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
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


ros::Subscriber sub;//データをsubcribeする奴
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
    
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入


//ここに処理項目
	cv::Mat img_src = RGBimage;
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_edge,img_dst,img_dst2;
    cv::Mat img_line,img_line1;

    

    img_src.copyTo(img_dst);
    img_src.copyTo(img_dst2);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::Canny(img_gray, img_edge, 200, 200);

    float dep,dep1,dep2;
    
    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);
    //標準的ハフ変換1
    std::vector<cv::Vec2f> lines0;
    cv::HoughLines(img_edge, lines0, 1, CV_PI/180, 120);

    //確率的ハフ変換
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 80,30,10);
    
    for(int i = 0; i < lines.size(); i++){
        dep1= img_depth.at<float>(lines[i][0],lines[i][1]);//点の距離データ取得
        dep2= img_depth.at<float>(lines[i][2],lines[i][3]);

    //dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
    if(dep1>0 && dep2>0){
        
        dep= dep1 - dep2;
        if(abs(dep)<500){//2点の距離の差が500(50cm)以下だった場合水平か垂直の線
        cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);
        //img_lineは確率的ハフ変換のラインのみの画像
        cv::line(img_line,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);   }
        else{
        cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,255,255), 4, cv::LINE_AA);
        cv::line(img_line,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);  }

       //cv::line(img_depth,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);  

        cv::circle(img_dst,Point(lines[i][0],lines[i][1]),10,Scalar(255,0,0),-1);//青点
        cv::circle(img_dst,Point(lines[i][2],lines[i][3]),10,Scalar(0,255,0),-1);//緑点

        std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<","<<dep1<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<","<<dep2<<")"<< std::endl;
        }
    }


    //標準的ハフ変換１
    for(int i = 0; i < lines0.size(); i++){
        double rho = lines0[i][0], theta = lines0[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double pt1x,pt1y,pt2x,pt2y;
        pt1x = x0 - img_dst2.cols*b ,pt1y = y0 + img_dst2.cols*a;
        pt2x = x0 + img_dst2.cols*b ,pt2y = y0 - img_dst2.cols*a;

        cv::line(img_dst2,cv::Point(pt1x,pt1y), cv::Point(pt2x,pt2y), cv::Scalar(0,255,0), 2, cv::LINE_AA);  
    }

    cv::Canny(img_line, img_line1, 200, 200);

    //標準的ハフ変換2(確率的ハフ変換で求めた画像に対して標準的ハフ変換を行う)
    std::vector<cv::Vec2f> lines1;
    cv::HoughLines(img_line1, lines1, 1, CV_PI/180, 120);

     for(int i = 0; i < lines1.size(); i++){
         std::cout <<"lines1.size= "<<lines1.size()<< std::endl;
        double rho = lines1[i][0], theta = lines1[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double pt1x,pt1y,pt2x,pt2y,img_cols,img_rows;
        pt1x = x0 - img_line.cols*b ,pt1y = y0 + img_line.cols*a;
        pt2x = x0 + img_line.cols*b ,pt2y = y0 - img_line.cols*a;

        img_cols=img_line.cols;
        img_rows=img_line.rows;

        std::cout <<"img_cols(幅)= "<<img_cols<< std::endl;
        std::cout <<"img_rows(高さ)= "<<img_rows<< std::endl;
        std::cout <<"lines1["<<i<<"][0]= "<<lines1[i][0]<< std::endl;
        std::cout <<"lines1["<<i<<"][1]= "<<lines1[i][1]<< std::endl;


        
        cv::line(img_line,cv::Point(pt1x,pt1y),cv::Point(pt2x,pt2y),cv::Scalar(255,0,0), 2, cv::LINE_AA);  
        cv::circle(img_line,Point(pt1x,pt1y),10,Scalar(255,0,0),-1);//青点
        cv::circle(img_line,Point(pt2x,pt2y),10,Scalar(255,255,0),-1);//青点
        cv::circle(img_line,Point(img_cols,img_rows),10,Scalar(0,0,255),-1);//青点

        

    }



    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_depth, img_depth);
    cv::imshow(win_line, img_line);
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

