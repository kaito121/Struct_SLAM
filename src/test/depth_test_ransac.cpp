//特徴点検出のテストプログラム
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
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

//床面推定
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_point = "point";

using namespace std;
using namespace cv;


cv::Mat img_dst;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
int kaisu;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

//ransac床面除去-------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);

//床面除去パラメータ
float f=699.649;//焦点距離、画像パラメータ
//--RANSAC
int ransacNum=500;
float distanceThreshold=0.1;
float epsAngle=15.0;
//--床面
float a,b,c,d;//平面の方程式の係数
float groundCandidateY=0.7;//カメラ目線から床までの高さ
float camHeight=0.7;//カメラの実際の高さ
float ground_th=0.2;//底面のしきい値
float height_th=1.0;//高さのしきい値
//マップパラメータ
//二次元グリッドマップのパラメータ
//float mapW=8.0;//width[m]
//float mapH=8.0;//height[m]
//float mapR=0.05;//resolution[m]
int mapWi;//(画像サイズ)マップサイズWidth[pixel]
int mapHi;//(画像サイズ)マップサイズHeight[pixel]



//--床面推定、床面除去
//----RANSAC
void groundEstimationRANSAC();
//void createPubDataRANSAC();//2次元ローカルマップ作成, マスク画像作成
//--座標変換
//センサ座標系ー＞マップ座標系
bool convertToGrid(const float& x,const float& y,int& xg,int& yg);


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    printf("処理の時間=%f\n", realsec);
  }

//サンプリング時間取得(ROS)
  ros::WallTime wall_now = ros::WallTime::now();
  ros::WallDuration wall_duration = wall_now - wall_begin;
  ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
  wall_systemtime = wall_duration - wall_prev;
  //ROS_INFO("systemtime:%u.%09u", wall_systemtime.sec, wall_systemtime.nsec);
  std::cout << "wall_systemtime=" <<wall_systemtime<< std::endl;//サンプリング時間

    //変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image;
//
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
//  
  	camera_info=*cam_info;//CameraInfo受け取り
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	  img_dst = image.clone();
    img_dst = cv::Scalar(255,255,255);

    float x_temp;
    float y_temp;
    float z_temp;
    pcl::PointXYZ p_temp;
    int k=0;//床面候補点の数(ground_pointsのサイズ)
    int hMax = bridgedepthImage->image.rows;//行
    int wMax = bridgedepthImage->image.cols;//列
    std::cout << "hMax="<<hMax<< std::endl;
    std::cout << "wMax="<<wMax<< std::endl;

    ground_points->points.resize(hMax/2*wMax);//候補点の最大値でリサイズ
    //画素高速アクセス用パラメータ(画像チャンネル数)
    int ch = bridgedepthImage->image.channels();
    //床面候補点抽出処理
    for(int h=hMax/2+1;h<hMax;h++){//画像下半分を走査
        //画素高速アクセス用パラメータ(先頭アドレス)
        float *p = bridgedepthImage->image.ptr<float>(h);//ポインタ取得
        for(int w=0;w<wMax;w++){
            // z_temp=depth_image.at<float>(h,w);
            z_temp = p[w*ch];//上行と同じ内容
            if(z_temp>0.5&&!std::isinf(z_temp)){
                y_temp=((float)hMax/2-h)*z_temp/f + camHeight;//高さ算出 ■この計算式どこから？
                if(std::abs(y_temp+camHeight)<groundCandidateY){//高さがgroundCandidateY未満の時
                    x_temp=-( ((float)w-(float)wMax/2)*z_temp/f);
                    //座標系の変換
                    p_temp.x=z_temp;
                    p_temp.y=x_temp;
                    p_temp.z=y_temp;
                    // ground_points->points.push_back(p_temp);
                    ground_points->points[k++] = p_temp;
                }
            }
        }
    }
    //再度リサイズ
    ground_points->points.resize(k);//
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    //再度リサイズ
    ground_points->points.resize(k);//
    ground_points->width=ground_points->points.size();
    ground_points->height=1;
    ROS_INFO_STREAM("ground_points->points.size():"<<ground_points->points.size()<<"\n");
    //床面式の算出
    seg.setInputCloud (ground_points);
	  seg.segment (*inliers, *coefficients);
    a=coefficients->values[0];
    b=coefficients->values[1];
    c=coefficients->values[2];
    d=coefficients->values[3];
    ROS_INFO_STREAM("Model coefficients: " << a << " "
                                          << b << " "
                                          << c << " "
                                           << d << "\n");
    //全部の画像データから面の法線ベクトルを作る
    //１つの画像から一個の面の法線ベクトルが出てくる
    //入力は画像ピクセルの三次元データ
  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);
	kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする
}



//メイン関数
int main(int argc,char **argv){
  //床面抽出パラメータ(ここでミスが起きてる)
  seg.setOptimizeCoefficients (true);
  //seg.setModelType (pcl::SACMODEL_PLANE);//全平面抽出
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//ある軸に垂直な平面を抽出
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (ransacNum);//RANSACの繰り返し回数
  seg.setDistanceThreshold (distanceThreshold);//モデルとどのくらい離れていてもいいか(モデルの評価に使用)
  seg.setAxis(Eigen::Vector3f (0.0,0.0,1.0));//法線ベクトル
  seg.setEpsAngle(epsAngle * (M_PI/180.0f));//許容出来る平面

	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

  
	ros::spin();//トピック更新待機
			
	return 0;
}