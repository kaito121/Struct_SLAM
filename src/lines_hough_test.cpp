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
#include <visualization_msgs/Marker.h>//ラインマーカー用
#include <cmath>
#include <struct_slam/MaskImageData.h>//パッケージ名要変更（自分で作ったデータを取り込んで）
#include <struct_slam/ImageMatchingData.h>


ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher marker_pub;
// ros::Publisher image_pub;
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_edge1 = "edge1";
std::string win_edge2 = "edge2";
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

    //オプティカルフローに送る画像データを代入

    
    // struct_slam::MaskImageData image_data;//image_data(トピック送信用）
    // int width,height;
    // float res,mapwidth=8,mapheight=8;
    // width = bridgeRGBImage->image.cols;//列のピクセル数
    // height = bridgeRGBImage->image.rows;//行のピクセル数
    // res=0.05;//マップサイズ調整

    // image_data.header = bridgeRGBImage->header;
    // image_data.mapWidth.data = mapwidth;
    // image_data.mapHeight.data = mapheight;
    // image_data.index.resize(width*height);//indexは総ピクセル数
    // image_data.pt.resize(width*height);//indexは総ピクセル数

    // image_data.mapRes.data = res;
    // image_data.mapWidthInt.data = (int)(width/res);
    // image_data.mapHeightInt.data = (int)(height/res);
     /*for (int i=0; i<=width; ++i) {
     for (int j=0;j<height; ++j) {
       image_data.pt.resize(width*height);//indexは総ピクセル数
       image_data.pt[].x = i;
       image_data.pt[].y = j;
       image_data.pt[].z = img_depth.at<float>;
     }}*/



//ここに処理項目
	  cv::Mat img_src = RGBimage;
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_edge,img_dst,img_dst2,img_dst3,img_dst4,img_edge2,img_edge1;
    cv::Mat img_line,img_line1;

    img_src.copyTo(img_dst);
    //img_src.copyTo(img_dst2);
    
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);
    cv::equalizeHist(img_gray,img_dst2);//ヒストグラム均一化

    //膨張縮小処理(一回目)
    Mat img_tmp,img_ab,img_tmpp;
    Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
    morphologyEx(img_dst2,img_tmp,MORPH_CLOSE,element8,Point(-1,-1),1);
    morphologyEx(img_tmp,img_ab,MORPH_OPEN,element8,Point(-1,-1),1);
    
    morphologyEx(img_ab,img_tmpp,MORPH_OPEN,element8,Point(-1,-1),1);
    morphologyEx(img_tmpp,img_dst4,MORPH_CLOSE,element8,Point(-1,-1),1);

    //エッジ抽出(二次微分)
    //cv::Mat img_tmp1;
    //cv::Laplacian(img_dst2, img_tmp1, CV_32F, 3);
    //cv::convertScaleAbs(img_tmp1, img_dst3, 1, 0);

    cv::Canny(img_dst4, img_edge, 200, 200);
    cv::Canny(img_gray, img_edge1, 200, 200);
    cv::Canny(img_dst2, img_edge2, 200, 200);

    float dep,dep1[100],dep2[100];
    double theta[100];
    

    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge1, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_edge1, img_edge1);
    cv::imshow(win_edge2, img_edge2);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    //cv::imshow(win_depth, img_depth);
    //cv::imshow(win_line, img_line);

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

  //ros::NodeHandle n;
  //marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // image_pub = n.advertise<sensor_msgs::Image>("maskImageData", 10);//realsenseの画像データをパブリッシュ

  

	ros::spin();//トピック更新待機
			
	return 0;
}