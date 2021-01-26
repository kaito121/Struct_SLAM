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
ros::Publisher  pub;
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_depth = "depth";

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

     // pointcloud を作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointCloud->points.reserve(RGBimage.size().width*RGBimage.size().height);//点の数


//ここに処理項目
	cv::Mat img_src = RGBimage;
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_edge,img_dst;
    

    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::Canny(img_gray, img_edge, 200, 200);

    float dep,dep1[100],dep2[100];
    double X_wariai=100;
    double Y_wariai=100;

    

    //確率的ハフ変換
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 80,30,10);
    
    for(int i = 0; i < lines.size(); i++){
        dep1[i]= img_depth.at<float>(lines[i][0],lines[i][1]);//点の距離データ取得
        dep2[i]= img_depth.at<float>(lines[i][2],lines[i][3]);

    //dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
    if(dep1[i]>0 && dep2[i]>0){
        
        dep= dep1[i] - dep2[i];
        if(abs(dep)<500){//2点の距離の差が10以下だった場合赤線
        cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);  }
        else{//2点の距離の差が10以上なら緑線
        cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(200,255,0), 4, cv::LINE_AA);  }

       //cv::line(img_depth,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);  

        cv::circle(img_dst,Point(lines[i][0],lines[i][1]),10,Scalar(255,0,0),-1);//青点
        cv::circle(img_dst,Point(lines[i][2],lines[i][3]),10,Scalar(0,255,0),-1);//緑点
        cv::circle(img_dst,Point(0,0),5,Scalar(0,0,255),-1);//赤点(原点0)
        cv::circle(img_dst,Point(0,100),5,Scalar(0,255,255),-1);//黄色点(0,100)
        cv::circle(img_dst,Point(100,0),5,Scalar(255,0,255),-1);//紫色点(100,0)

        std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<","<<dep1[i]<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<","<<dep2[i]<<")"<< std::endl;

        pcl::PointXYZRGB jk1;
        pcl::PointXYZRGB jk2;

        //XYはimageのデータなのでpclにそのままもって行くとでかい そこである定数で割ることでクラスタリング座標に変換する-------------------------(1)
             jk1.x=(float)lines[i][0]/X_wariai;//ピクセル距離をクラスタリング距離に変換
             jk1.y=(float)lines[i][1]/Y_wariai;
             jk1.z=(float)dep1[i];
             jk2.x=(float)lines[i][2]/X_wariai;
             jk2.y=(float)lines[i][3]/Y_wariai;
             jk2.z=(float)dep2[i];//ZはDepthデータなのでそのままで行ける

             pointCloud->points.emplace_back(jk1);//ポイントクラウドに座標データを移動
             pointCloud->points.emplace_back(jk2);


        }
    }

    //pointcloudサイズ設定
    pointCloud -> width = pointCloud -> points.size();
    pointCloud -> height = 1;
    pointCloud -> is_dense = true;

    /*// クラスタリングの設定
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pointCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);//s探索方法
	ec.setInputCloud (pointCloud);//クラスタリングするポイントクラウドの指定

    // クラスタリング実行
    std::vector<pcl::PointIndices> indices;
	ec.extract (indices);//結果*/

     sensor_msgs::PointCloud2 depth_pcl;
     pcl::toROSMsg (*pointCloud, depth_pcl);
     depth_pcl.header.stamp = ros::Time::now();
     depth_pcl.header.frame_id = rgb_msg->header.frame_id;
     pub.publish(depth_pcl);



    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_depth, img_depth);
	cv::waitKey(1);



   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"test_pcl");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    ros::NodeHandle nhPub;
    pub=nhPub.advertise<sensor_msgs::PointCloud2>("depth_pcl", 1000);

	ros::spin();//トピック更新待機
			
	return 0;
}
