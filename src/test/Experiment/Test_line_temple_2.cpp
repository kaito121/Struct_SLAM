//線検出(FLD)→Depth取得不可能な線を削除
//まずナナメの線のみを取り出し、取り出した後のナナメの線が含まれていないグループを作る
//その後作成したグループからタテ、ヨコ線の検出を行う
//20210908 青線がばーって表示されるバグを直したが、kskの個数が０個になるとSegmentationFolutになるのが確認できた（たまに起こる）
//そのためkskの対策が必要

//rosのヘッダ
//#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <Eigen/Dense>
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <time.h>//処理の時間を出力する
#include <sys/time.h>

#include <geometry_msgs/PoseStamped.h>//tf
#include <tf/transform_broadcaster.h>//tf
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include <nav_msgs/Path.h>//経路情報を記録する

#include <geometry_msgs/Twist.h>//ロボットの指令値(速度)用ヘッダー
#include <nav_msgs/Odometry.h>
#include<fstream>


ros::Subscriber sub;//データをsubcribeする奴
ros::Subscriber odom_sub;//ロボットオドメトリー受信用
ros::Publisher  pub;//速度送信用
ros::Publisher Des_pub_plan;//カメラ経路送信用(指令値)
ros::Publisher Act_pub_plan;//カメラ経路送信用(観測値)
ros::Publisher Est_pub_plan;//カメラ経路送信用(推定値)
std::string source_frame = "map";//mapフレーム

std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_fld = "fld";//FLDの線を表示
std::string win_fld_ty = "FLD_TY";//抽出したFLDのタテ、ヨコの線を表示
std::string win_fld_t = "FLD_T";//抽出したFLDのタテ線を表示
std::string win_fld_y = "FLD_Y";//抽出したFLDのヨコの線を表示

std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_line3 = "line3";//分類分けしたタテ、ヨコ、ナナメの線を表示
std::string win_line4 = "line4";//分類分けしたタテ、ヨコの線を表示

std::string win_dstP = "dstP";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
//std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像
std::string win_graph = "graph";//グラフ作成

std::string win_tate = "tate";//縦線関連の画像表示用
std::string win_tmp = "tmp";//縦線関連の画像表示用


using namespace std;
using namespace cv;
using Eigen::MatrixXd;

int linesu=400;
int kaisu=0,kaisuM1=0,kaisuV1=0;
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)出を行うように、初期値を TRUE にする
bool reset = true;
bool yoko_histgram = false;//ヨコ線のヒストグラム(true実行)
bool tate_histgram = false;//タテ線のヒストグラム(true実行)
bool time0 = false;//カルマン推定データ初回回避用(カルマン動作した時TRUE)
bool X_25=false;
bool TH_90=false;
bool Y_05=false;
bool Tracking2=false;
bool kensyutu=true;

double VX,omegaZ,LX,THZ,LY;//ロボットの指令値パラメータ
int act_v=0;//ロボット速度制御用

//時間取得用
ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
struct timeval startTime2, endTime2;  // 構造体宣言
struct timeval startTimeV1, endTimeV1;  // 構造体宣言
struct timeval startTimeM1, endTimeM1;  // 構造体宣言
float realsec,ALLrealsec;//サンプリング時間（C++)
float realsecV1,ALLrealsecV1;//直進動作時間（C++)
float realsecM1,ALLrealsecM1;//回転動作時間（C++)

int template_size=10;
double cropx=10*2.5;//予測範囲の範囲(template_size*n)
double cropy=10*2.5;//予測範囲の範囲(template_size*n)
//double cropx=10*2.5;//予測範囲の範囲(template_size*n)
//double cropy=10*2.5;//予測範囲の範囲(template_size*n)


cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1,img_tmp;
cv::Mat img_FLD_TY,img_FLD_T,img_FLD_Y,img_graph;
cv::Mat img_tate;//縦線関連の画像表示
double Average_tate_theta,Average_tate_theta_Y;//最大クラスタ内の平均角度を求める

//マーカー関連
cv::Mat_<float>MarkerW[40]=cv::Mat_<float>(3, 1);//マーカーの世界座標
int ALLMarker=40;//全マーカー個数
float MC_point_prve[50][4];//一つ前のカメラ座標
float pixel[50][2],depth[100],MC_point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換
double LM[1][2],CameraLM[50][2];//ランドマーク世界座標
cv::Mat Zu[40];//センサーの観測値(仮)

//縦線のテンプレート取得関連
vector<cv::Point2f> tate_point_curr,tate_point_prev;//縦線の中点座標(画像座標系)
double DTPC[100],DTPP[100];//depth_tate_point_curr=DTPC,depth_tate_point_prev=DTPP(Depth取得可能縦線中点の数)
int DTPC_ok,DTPP_ok;//(Depth取得可能縦線数)
vector<cv::Point3f> TPCC,TPCP;//tate_point_camera_curr=TPCC,tate_point_camera_prev=TPCP(縦線中点のカメラ座標系)
cv::Mat TPCC_Templ[100],TPCP_Templ[100];//縦線中点のテンプレート画像

//テンプレートマッチ用変数
vector<cv::Point3f> Est_tate_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_tate_pixel;//特徴点定義(運動復元
vector<cv::Point2f> MTPC,MTPP;//テンプレートの中心座標(Matching_Tate_Point_Curr=MTPC,Matching_Tate_Point_Prev=MTPP)
int matchT_curr=0,matchT_prev=0;//マッチングしたテンプレート数
cv::Mat MTTC[500],MTTP[500];//マッチングしたテンプレート画像キープ用(Matching_Tate_Templ_Curr=MTTC,Matching_Tate_Templ_Prev=MTTP)
cv::Mat EST_tate_scope[100];//特徴点周囲の切り取った画像(予測範囲画像)
cv::Mat img_template1,img_master_temp;
cv::Point min_pt1[100], max_pt1[100];//テンプレートマッチング用変数
double min_val1[100], max_val1[100];


cv::Mat MT_curr_Templ[100],MT_prev_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MT_curr_pixel,  MT_prev_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MT_curr_camera, MT_prev_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MT_curr_world,  MT_prev_world;//マッチテンプレートの世界座標系
vector<cv::Point3f> MT_curr_world2, MT_prev_world2;//マッチテンプレートの世界座標系
double DMT_curr[600],DMT_prev[600];//Depth_Matching_tate_prev(マッチテンプレート座標のDepth)
int DMT_curr_ok=0,DMT_prev_ok;//マッチテンプレートのDepth取得可能数
double length;//テンプレートの距離比較用(追加更新動作)

//テンプレートマッチ用変数(3回目動作用)
cv::Mat MT_curr2_Templ[100];//マッチテンプレート画像
vector<cv::Point2f> MT_curr2_pixel;//マッチテンプレートの画像座標系
vector<cv::Point3f> MT_curr2_camera;//マッチテンプレートのカメラ座標系
vector<cv::Point3f> MT_curr2_world;//マッチテンプレートの世界座標
double DMT_curr2[600];//Depth_Matching_pixel_prev(マッチテンプレート座標のDepth)
int DMT_curr2_ok=0;//マッチテンプレートのDepth取得可能数

double MTcoix[300],MTcoiy[300];
vector<cv::Point3f> Est_MT_point;//特徴点定義(運動復元
vector<cv::Point2f> Est_MT_pixel;//特徴点定義(運動復元
cv::Mat EST_MT_scope[100];//予測範囲クロップ
int EST_MT_ok=0;//画面内のテンプレート数
double CameraLMT[600];
cv::Mat Zu_T[600];//センサーの観測値(仮)


geometry_msgs::Twist robot_velocity;//指令速度
nav_msgs::Odometry robot_odometry;//ロボットのオドメトリー

//ロボット動作関連
double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

//カルマンフィルタ関連
cv::Mat ACT_Robot,DES_Robot,EST_Robot;//状態方程式の行列化
cv::Mat At,Mt,Ft,Cov,Qt,K,Ht,I,hu;

geometry_msgs::Pose Des_Robot_pose;//ロボットtf(指令値)
geometry_msgs::Pose Act_Robot_pose;//ロボットtf(観測値)
geometry_msgs::Pose Est_Robot_pose;//ロボットtf(推定値)
nav_msgs::Path Des_path;//カメラ経路表示設定
nav_msgs::Path Act_path;//カメラ経路表示設定
nav_msgs::Path Est_path;//カメラ経路表示設定
geometry_msgs::PoseStamped Des_pose;//ロボット姿勢(指令値)
geometry_msgs::PoseStamped Act_pose;//ロボット姿勢(観測値)
geometry_msgs::PoseStamped Est_pose;//ロボット姿勢(推定値)

ofstream temp_kosuu("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/temp_kosuu.txt");
ofstream temp_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/temple_time1.txt");
ofstream ALLtemp_time("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/aLL_temple_time.txt");
ofstream ALLtemp_time0("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match0_ALLtime.txt");
ofstream ALLtemp_time1("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match1_ALLtime.txt");
ofstream ALLtemp_time2("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match2_ALLtime.txt");
ofstream ALLtemp_time3("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match3_ALLtime.txt");
ofstream ALLtemp_time4("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match4_ALLtime.txt");
ofstream ALLtemp_time5("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match5_ALLtime.txt");
ofstream ALLtemp_time6("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match6_ALLtime.txt");
ofstream ALLtemp_time7("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match7_ALLtime.txt");
ofstream ALLtemp_time8("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match8_ALLtime.txt");

ofstream max_match0("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match0_max.txt");
ofstream max_match1("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match1_max.txt");
ofstream max_match2("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match2_max.txt");
ofstream max_match3("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match3_max.txt");
ofstream max_match4("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match4_max.txt");
ofstream max_match5("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match5_max.txt");
ofstream max_match6("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match6_max.txt");
ofstream max_match7("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match7_max.txt");
ofstream max_match8("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match8_max.txt");

ofstream min_match0("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match0_min.txt");
ofstream min_match1("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match1_min.txt");
ofstream min_match2("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match2_min.txt");
ofstream min_match3("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match3_min.txt");
ofstream min_match4("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match4_min.txt");
ofstream min_match5("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match5_min.txt");
ofstream min_match6("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match6_min.txt");
ofstream min_match7("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match7_min.txt");
ofstream min_match8("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match8_min.txt");

ofstream match0_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match0_pixel_x.txt");
ofstream match1_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match1_pixel_x.txt");
ofstream match2_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match2_pixel_x.txt");
ofstream match3_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match3_pixel_x.txt");
ofstream match4_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match4_pixel_x.txt");
ofstream match5_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match5_pixel_x.txt");
ofstream match6_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match6_pixel_x.txt");
ofstream match7_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match7_pixel_x.txt");
ofstream match8_pixel_x("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match8_pixel_x.txt");


ofstream match0_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match0_pixel_y.txt");
ofstream match1_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match1_pixel_y.txt");
ofstream match2_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match2_pixel_y.txt");
ofstream match3_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match3_pixel_y.txt");
ofstream match4_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match4_pixel_y.txt");
ofstream match5_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match5_pixel_y.txt");
ofstream match6_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match6_pixel_y.txt");
ofstream match7_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match7_pixel_y.txt");
ofstream match8_pixel_y("/home/fuji/catkin_ws/src/Struct_SLAM/src/test/date/match8_pixel_y.txt");



//コールバック関数
void callback(const nav_msgs::Odometry::ConstPtr& msg,const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
    //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
    gettimeofday(&startTime, NULL);// 開始時刻取得
    if(time0 != false){
      time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
      suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
      realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
      ALLrealsec=ALLrealsec+realsec;
      printf("処理の時間=%f\n", realsec);
      printf("処理時間合計=%f\n", ALLrealsec);
    }

	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	cv::Mat image;//opencvの画像
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

    cv::Mat img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
    cv::Mat img_depth = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
        //ここに処理項目
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_fld,img_line2,img_line3,img_line4;


    //ロボット指令-------------------------------------------------------------------
    robot_odometry=*msg;
    /*//廊下直進動作----------------------------------------------------------------------------------------------------------------
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      //直進動作
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
        robot_velocity.angular.z = (Des_RobotTH-Act_RobotTH)*2.2;

        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.linear.x  = VX;} 
        robot_velocity.angular.z = 0.0;
        if(Des_RobotX>=LX){//停止命令
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      }*/
    

    /*//廊下動作1-----------------------------------------------------------------------------------------------------------------
      std::cout << "X_25=" <<X_25<<",TH_90=" <<TH_90<<",Y_05=" <<Y_05<< std::endl;
      //直進動作
      if(X_25==false&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);//実行指令値
        robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.linear.x  = VX;} 
        robot_velocity.angular.z = 0.0;
        if(Des_RobotX>=LX){//停止命令
          X_25=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      //回転動作
      else if(X_25==true&&TH_90==false&&Y_05==false){
        robot_velocity.linear.x  =  0;
        robot_velocity.angular.z  =  THZ+(Des_RobotTH-Act_RobotTH)*2.2;//実行指令値
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        robot_velocity.linear.x  =  0.0;
        robot_velocity.angular.z  =  THZ; 
        if(Act_RobotTH<=0.0001){robot_velocity.angular.z  =  0;}//指令値の遅れを考慮(推定に使用する指令値)
        else{robot_velocity.angular.z  =  THZ;} 
        if(Des_RobotTH>=3.141592653/omegaZ){//廊下
          TH_90=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==true&&Y_05==false){
        robot_velocity.linear.x  = VX+2.2*(Des_RobotX-Act_RobotX)+2.2*(Des_RobotY-Act_RobotY);
        robot_velocity.angular.z=(Des_RobotTH-Act_RobotTH)*2.2;
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
        if(Act_RobotV<=0.1){robot_velocity.linear.x=0;}
        else{robot_velocity.linear.x  = VX;} // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        if(Des_RobotY>=LY){
          Y_05=true;
          robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
          robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
          pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
          usleep(7*100000);//0.5秒ストップ(マイクロ秒)
        }
      }
      else if(X_25==true&&TH_90==true&&Y_05==true){
        robot_velocity.linear.x  = 0.0; // 並進速度vの初期化
        robot_velocity.angular.z = 0.0; // 回転速度ωの初期化}//xが1以上になったら終了
        pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）
      }*/

    Act_RobotV=robot_odometry.twist.twist.linear.x+robot_odometry.twist.twist.linear.y;//速度ベクトルの合成
    std::cout << "Act_RobotV=" <<Act_RobotV<< std::endl;

    Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
    std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;
//ロボット指令終わり---------------------------------------------------------------------------------

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_fld = img_src.clone();
    img_fld = cv::Scalar(255,255,255);
    img_line2 = img_src.clone();
    img_line2 = cv::Scalar(255,255,255);
    img_line3 = img_src.clone();
    img_line3 = cv::Scalar(255,255,255);
    img_line4 = img_src.clone();
    img_line4 = cv::Scalar(255,255,255);
    img_FLD_TY = img_src.clone();
    img_FLD_TY = cv::Scalar(255,255,255);
    img_FLD_T = img_src.clone();
    img_FLD_T = cv::Scalar(255,255,255);
    img_FLD_Y = img_src.clone();
    img_FLD_Y = cv::Scalar(255,255,255);
    img_tate = img_src.clone();

    img_graph= img_src.clone();
    img_graph= cv::Scalar(255,255,255);
    cv::line(img_graph,cv::Point(0,480),cv::Point(640,480),cv::Scalar(0,0,0), 3, cv::LINE_AA);//X軸 
    cv::line(img_graph,cv::Point(0,480),cv::Point(0,0),cv::Scalar(0,0,0), 3, cv::LINE_AA);//Y軸 
    cv::line(img_graph,cv::Point(90*6,0),cv::Point(90*6,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(80*6,0),cv::Point(80*6,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(45*6,0),cv::Point(45*6,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(10*6,0),cv::Point(10*6,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
    
    img_src.copyTo(img_dst);
    img_src.copyTo(img_dst2);

    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

  if (reset == true) {
    std::cout <<"検出動作"<< std::endl;

    img_tmp = img_src.clone();
    //FLD変換
    std::vector<cv::Vec4f> lines_fld;
    std::vector<cv::Vec4f> lines_std;
    
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines_fld);//特徴線検索
    fld->detect( img_gray, lines_std);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines_fld.size(); i++){
       //cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_fld,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }
    std::cout <<"lines_fld.size()="<<lines_fld.size()<< std::endl;

    cv::cvtColor(img_fld, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 125, 255);

    double lines2[lines_fld.size()][4],theta0,theta90;
    float dep1[lines_fld.size()],dep2[lines_fld.size()];

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    //FLD抽出線とY軸との角度を求める+三次元距離データの結合
    std::cout <<"並び替え前"<<std::endl;
    int ksk=0;
    for(int i =  0; i < lines_fld.size(); i++){
        dep1[i]= img_depth.at<float>(cv::Point(lines_fld[i][0],lines_fld[i][1]));//点の三次元距離データ取得//20210908ここでDepth情報を取得する必要はない気がする
        dep2[i]= img_depth.at<float>(cv::Point(lines_fld[i][2],lines_fld[i][3]));
        if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
            lines2[ksk][0]=lines_fld[i][0];//ここで距離０を除いてる
            lines2[ksk][1]=lines_fld[i][1];
            lines2[ksk][2]=lines_fld[i][2];
            lines2[ksk][3]=lines_fld[i][3];
            dep1[ksk]=dep1[i];
            dep2[ksk]=dep2[i];
            ksk=ksk+1;//距離データを正しく持つ線のみを取得
        }
    }
    std::cout <<"テスト:Depthデータが取得可能な線の個数ksk="<<ksk<<std::endl;

    double lines_NNM[lines_fld.size()][4],lines_NNM2[lines_fld.size()][4],lines_NNM_lc[lines_fld.size()],lines_NNM_thetal[lines_fld.size()];
    double lines3[lines_fld.size()][4],lines3_dep[lines_fld.size()][2],lines3_theta[lines_fld.size()];//抽出されたナナメの線以外
    double NNMA[lines_fld.size()],NNMB[lines_fld.size()],NNM_TATE_X[lines_fld.size()][lines_fld.size()],NNM_TATE_Y[lines_fld.size()][lines_fld.size()];
    int lines_NNM_count=0,lines3_count=0;
	  double minlength = image.cols * image.cols * 0.02 ;// (線の最小長さの要件を定義する)
    double NNM_A[lines_fld.size()],NNM_C[lines_fld.size()],NNM_A_MAX,NNM_C_MAX;//斜め線の一次関数
    double NNM_XN[lines_fld.size()],NNM_YN[lines_fld.size()],NNM_CHK[lines_fld.size()];//縦線の結合用
    double NNM_L[lines_fld.size()],tempx,tempy;//縦線の長さ
    double NNM_line[lines_fld.size()][4];//縦線結合後
    int nnmno=0;//縦線結合後個数
    //ナナメの線の抽出を行う-----------------------------------------------------------------------------------
    for(int i=0; i<ksk; i++){
	    //(ほぼ垂直の場合は無視)
	    if ( abs(lines2[i][0]-lines2[i][2]) < 10 || abs(lines2[i][1]-lines2[i][3]) < 10){ //check if almost vertical
            lines3[lines3_count][0]=lines2[i][0];//ナナメの線以外を抽出する
            lines3[lines3_count][1]=lines2[i][1];
            lines3[lines3_count][2]=lines2[i][2];
            lines3[lines3_count][3]=lines2[i][3];
            lines3_dep[lines3_count][0]=dep1[ksk];
            lines3_dep[lines3_count][1]=dep2[ksk];
            cv::line(img_FLD_TY,cv::Point(lines3[lines3_count][0],lines3[lines3_count][1]),cv::Point(lines3[lines3_count][2],lines3[lines3_count][3]),cv::Scalar(0,0,0), 2, cv::LINE_AA);

            //FLD抽出線のy軸との角度を求める
            lines3_theta[lines3_count]=M_PI-atan2((lines3[lines3_count][2]-lines3[lines3_count][0]),(lines3[lines3_count][3]-lines3[lines3_count][1]));
            //std::cout <<"FLDの線の傾きlines3_theta["<<lines3_count<<"]("<<lines3_theta[lines3_count]<<")"<< std::endl;

            lines3_count=lines3_count+1;//ナナメの線以外を線を数える
			continue;
        }
		//(短い線を無視する (x1-x2)^2 + (y2-y1)^2 < minlength)
		if( ((lines2[i][0]-lines2[i][2])*(lines2[i][0]-lines2[i][2]) +(lines2[i][1]-lines2[i][3])*(lines2[i][1]-lines2[i][3])) < minlength){
			continue;
        }   
        lines_NNM[lines_NNM_count][0]=lines2[i][0];//ナナメの線のみを抽出する
        lines_NNM[lines_NNM_count][1]=lines2[i][1];
        lines_NNM[lines_NNM_count][2]=lines2[i][2];
        lines_NNM[lines_NNM_count][3]=lines2[i][3];

        cv::line(img_line2,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
        cv::line(img_line4,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
        cv::line(img_dst,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA); 

        //座標から一次関数を引く関数
        lines_NNM_thetal[lines_NNM_count]=(M_PI/2)-(M_PI-atan2((lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0]),(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])));
        lines_NNM_lc[lines_NNM_count]=(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])*(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])+(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])*(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1]);
        lines_NNM2[lines_NNM_count][0]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//X1座標
        lines_NNM2[lines_NNM_count][1]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//Y1座標
        lines_NNM2[lines_NNM_count][2]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//X2座標
        lines_NNM2[lines_NNM_count][3]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//Y2座標

        cv::line(img_line2,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_line4,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_FLD_TY,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_dst,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        NNM_CHK[lines_NNM_count]=0;//斜め線判別要素初期化（斜め線の結合に使用)
        lines_NNM_count=lines_NNM_count+1;//ナナメの線の数をカウント
	  }//------------------------------------------------------------------------------------------------------------------------(ナナメ線抽出)

    //線のグループ化-----------------------------------------------------------------------------------------------------------
    for (int i=0; i<lines_NNM_count; ++i) {
        //lines_NNM[i][1]の位置を上にする
        if(lines_NNM[i][1]>lines_NNM[i][3]){
            tempx=lines_NNM[i][0];
            tempy=lines_NNM[i][1];
            lines_NNM[i][0]=lines_NNM[i][2];
            lines_NNM[i][1]=lines_NNM[i][3];
            lines_NNM[i][2]=tempx;
            lines_NNM[i][3]=tempy;
        }
        NNM_L[i]=sqrt((lines_NNM[i][0]-lines_NNM[i][2])*(lines_NNM[i][0]-lines_NNM[i][2])
         +(lines_NNM[i][1]-lines_NNM[i][3])*(lines_NNM[i][1]-lines_NNM[i][3]));//縦線の点間の長さ
        NNM_XN[i]=(lines_NNM[i][0]+lines_NNM[i][2])/2;//縦線の中点N
        NNM_YN[i]=(lines_NNM[i][1]+lines_NNM[i][3])/2;
        NNM_A[i]=(lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]);
        NNM_C[i]=lines_NNM[i][1]-(((lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]))*lines_NNM[i][0]);
    }

    double clusNNM_R=0.5;//しきい値
     //角度が類似する線をまとめる
    for (int j=0; j<lines_NNM_count; ++j) {
        if(NNM_CHK[j]==0){
            //std::cout <<"斜めの線["<<j<<"]"<< std::endl;
            int equal=1;//同じ線の要素数
            double NNM_MAX_LONG=NNM_L[j];
            double NNM_MIN=lines_NNM[j][1];
            double NNM_MAX=lines_NNM[j][3];
            NNM_line[nnmno][0]=lines_NNM[j][0],NNM_line[nnmno][1]=lines_NNM[j][1];//基準はjの線(更新が起きなければj単体)
            NNM_line[nnmno][2]=lines_NNM[j][2],NNM_line[nnmno][3]=lines_NNM[j][3];

            for (int k=j+1; k<lines_NNM_count; ++k) {
                //角度の差がしきい値以下の時同じ線とみなす
                if(abs(lines_NNM_thetal[j]-lines_NNM_thetal[k])<clusNNM_R){
                    //std::cout <<"同じ線とみなすlines_NNM["<<k<<"]="<<lines_NNM_thetal[j]-lines_NNM_thetal[k]<< std::endl;
                    NNM_CHK[k]=1;//同じ線とみなす
                    //Y最小が直線の下の端点、Y最大が上の端点になる
                    if(NNM_MIN>lines_NNM[k][1]){
                        NNM_MIN=lines_NNM[k][1];
                        NNM_line[nnmno][1]=lines_NNM[k][1];//Y最小が直線の上の端点
                    }
                    if(NNM_MAX<lines_NNM[k][3]){
                        NNM_MAX=lines_NNM[k][3];
                        NNM_line[nnmno][3]=lines_NNM[k][3];//Y最大が下の端点
                    }
                    //長さが最も長い直線を見つける(長い方がデータに信頼性がある)
                    if(NNM_MAX_LONG<NNM_L[k]){
                        NNM_MAX_LONG=NNM_L[k];//最大長さ更新
                        NNM_A_MAX=NNM_A[k];//最大長さの一次関数を保存
                        NNM_C_MAX=NNM_C[k];//最大長さの一次関数を保存
                    }
                }
            }
            //最も長い線の一次関数からy最大、最小のxを求める
            NNM_line[nnmno][0]=(NNM_line[nnmno][1]-NNM_C_MAX)/NNM_A_MAX;
            NNM_line[nnmno][2]=(NNM_line[nnmno][3]-NNM_C_MAX)/NNM_A_MAX;
            //cv::line(img_dst,cv::Point(NNM_line[nnmno][0],NNM_line[nnmno][1]),cv::Point(NNM_line[nnmno][2],NNM_line[nnmno][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);
            nnmno=nnmno+1;//縦線の合計個数(まとめ後)
        }
    }
    //std::cout <<"縦線のクラスタ数(まとめ前)lines_NNM_count="<<lines_NNM_count<< std::endl;
    //std::cout <<"縦線の合計個数(まとめ後)tateno="<<nnmno<< std::endl;
    //------------------------------------------------------------------------------------------------------------------------------- 

    //thetaの数値を小さいにソート
    double tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    int yokot,tatet,p,yokoyouso,tateyouso;
    double lines3X,lines3Y,yokolines3[lines_fld.size()][5],tatelines3[lines_fld.size()][5],yokotheta[lines_fld.size()],tatetheta[lines_fld.size()];
    double yokoZ1[lines_fld.size()],yokoZ2[lines_fld.size()],tateZ1[lines_fld.size()],tateZ2[lines_fld.size()];
    double yokothetal[lines_fld.size()],tatethetal[lines_fld.size()],yokolc[lines_fld.size()],tatelc[lines_fld.size()],yokol[lines_fld.size()][4],tatel[lines_fld.size()][4];
    double datat[lines_fld.size()],datay[lines_fld.size()];//ヒストグラムデータ用
    int clusCY,clusCT,clusy[lines_fld.size()],clust[lines_fld.size()],MAXY=0,MAXT=0;//クラスタリング用
    double yokothetaCL[lines_fld.size()],tatethetaCL[lines_fld.size()],clusR;//クラスタリング用
    double yokoclusy[100][200][5],tateclust[100][200][5];
    double Average_yoko_theta[lines_fld.size()];
    double MINI_theta=100,mini_theta;
    int CLUSTER[lines_fld.size()],clus_no=0,YOKO_CLUST;
    int nanamet=0,nanamey=0,NANAME_Line[100];
    double lines_T_theta[lines_fld.size()];
    double tateA[lines_fld.size()],tateB[lines_fld.size()];
    double TATE_A[lines_fld.size()],TATE_C[lines_fld.size()],TATE_A_MAX,TATE_C_MAX;//縦線の一次関数y=TATE_Ax+TATE_C
    double TATE_D[lines_fld.size()][lines_fld.size()],TATE_XN[lines_fld.size()],TATE_YN[lines_fld.size()];//縦線の結合用
    double TATE_L[lines_fld.size()],TATE_K[lines_fld.size()];//縦線の長さ
    double TATE_line[lines_fld.size()][4];//縦線結合後
    int tateno=0,tateok=0;//縦線結合後個数
    
    yokot=0,tatet=0,p=0,yokoyouso=0,tateyouso=0;

    for (int j=0; j< lines3_count; ++j) {
        lines3X=abs(lines3[j][0]-lines3[j][2]);//傾きを調べる（x成分)
        lines3Y=abs(lines3[j][1]-lines3[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(lines3X>lines3Y){
            //std::cout <<"yoko(lines3X>lines3Y)="<<lines3X<<">"<<lines3Y<< std::endl;
            //std::cout <<"lines3_theta["<<j<<"](ヨコ)="<<lines3_theta[j]<< std::endl;

            yokolines3[yokoyouso][0]=lines3[j][0];//(x成分)
            yokolines3[yokoyouso][1]=lines3[j][1];//(y成分)
            yokolines3[yokoyouso][2]=lines3[j][2];//(x成分)
            yokolines3[yokoyouso][3]=lines3[j][3];//(y成分)
            yokolines3[yokoyouso][4]=lines3_theta[j];

            //yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=lines3_dep[j][0];
            yokoZ2[yokoyouso]=lines3_dep[j][1];
            //cv::line(img_line3,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            //cv::line(img_FLD_Y,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            cv::line(img_line2,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]), 4, cv::Scalar(255, 0, 255), 1.5);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]), 4, cv::Scalar(255, 0, 255), 1.5);

            yokotheta[yokoyouso]=lines3_theta[j]*180/M_PI;//deg表示化
            //θの範囲を0〜180にする
            if(yokotheta[yokoyouso]>=180){
                yokotheta[yokoyouso]=yokotheta[yokoyouso]-180;
                yokolines3[yokoyouso][4]=yokolines3[yokoyouso][4]-M_PI;
            }
            //std::cout <<"yokotheta["<<yokoyouso<<"]="<<yokotheta[yokoyouso]<< std::endl;

            if(yokotheta[yokoyouso]>=90){
                yokotheta[yokoyouso]=180-yokotheta[yokoyouso];
            }
            else{yokotheta[yokoyouso]=yokotheta[yokoyouso];}
            //std::cout <<"yokotheta["<<yokoyouso<<"](クラスタリング用)="<<yokotheta[yokoyouso]<< std::endl;

            clusy[yokoyouso]=0;//クラスタリング用
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)
        }
        //縦線に分類
        else{
            if(lines3Y>lines3X){
                //std::cout <<"tate(lines3Y>lines3X)="<<lines3Y<<">"<<lines3X<< std::endl;
                //std::cout <<"lines3_theta["<<j<<"](タテ)="<<lines3_theta[j]<< std::endl;
                tatelines3[tateyouso][0]=lines3[j][0];
                tatelines3[tateyouso][1]=lines3[j][1];
                tatelines3[tateyouso][2]=lines3[j][2];
                tatelines3[tateyouso][3]=lines3[j][3];
                tatelines3[tateyouso][4]=lines3_theta[j];

                //tatetheta[tateyouso]=lines3_theta[j];
                tateZ1[tateyouso]=lines3_dep[j][0];
                tateZ2[tateyouso]=lines3_dep[j][1];
                //cv::line(img_line3,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                //cv::line(img_FLD_T,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                cv::line(img_line2,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);

                //確率ハフ変換を使用しない時
                tatetheta[tateyouso]=lines3_theta[j]*180/M_PI;//deg表示化

                //θの範囲を0〜180にする
                if(tatetheta[tateyouso]>=180){
                    tatetheta[tateyouso]=tatetheta[tateyouso]-180;
                    tatelines3[tateyouso][4]=tatelines3[tateyouso][4]-M_PI;
                    }
                //std::cout <<"tatetheta["<<tateyouso<<"]="<<tatetheta[tateyouso]<< std::endl;

                //クラスタリング時に最大個数を持つ縦線のクラスタが２つ存在してしまうため、90度で反転させてクラスタリング処理を行う。
                //例(θ=0~10,170~180→180-170=10)
                if(tatetheta[tateyouso]>=90){
                    tatetheta[tateyouso]=180-tatetheta[tateyouso];
                }
                else{tatetheta[tateyouso]=tatetheta[tateyouso];}
                //std::cout <<"tatetheta["<<tateyouso<<"](クラスタリング用)="<<tatetheta[tateyouso]<< std::endl;

                clust[tateyouso]=0;//クラスタリング用
                tateyouso=tateyouso+1;//縦線に分類されたグループ数(tateyouso)
            }
        }
    }

    //ここの並び替え上の並び替えとまとめられそう（先に範囲を狭めてから並び替えして分類する感じにしたらできそう）
    //今は並び替えして分類して範囲狭めて再び並び替えしてる
    //クラスタリング用θで並び替えを行う
    //tatethetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<tateyouso; ++i) {
       for (int j=i+1;j<tateyouso; ++j) {
            if (tatetheta[i] > tatetheta[j]) {
                tmp =  tatetheta[i];
                tmp1x =  tatelines3[i][0];
                tmp1y =  tatelines3[i][1];
                tmp2x =  tatelines3[i][2];
                tmp2y =  tatelines3[i][3];
                tmp3 =  tatelines3[i][4];

                tatetheta[i] = tatetheta[j];
                tatelines3[i][0] = tatelines3[j][0];
                tatelines3[i][1] = tatelines3[j][1];
                tatelines3[i][2] = tatelines3[j][2];
                tatelines3[i][3] = tatelines3[j][3];
                tatelines3[i][4] = tatelines3[j][4];

                tatetheta[j] = tmp;
                tatelines3[j][0] = tmp1x;
                tatelines3[j][1] = tmp1y;
                tatelines3[j][2] = tmp2x;
                tatelines3[j][3] = tmp2y;
                tatelines3[j][4] = tmp3;
            }
        }
    }
    
    //クラスタリング用θで並び替えを行う
    //yokothetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<yokoyouso; ++i) {
       for (int j=i+1;j<yokoyouso; ++j) {
            if (yokotheta[i] > yokotheta[j]) {
                tmp =  yokotheta[i];
                tmp1x =  yokolines3[i][0];
                tmp1y =  yokolines3[i][1];
                tmp2x =  yokolines3[i][2];
                tmp2y =  yokolines3[i][3];
                tmp3 =  yokolines3[i][4];

                yokotheta[i] = yokotheta[j];
                yokolines3[i][0] = yokolines3[j][0];
                yokolines3[i][1] = yokolines3[j][1];
                yokolines3[i][2] = yokolines3[j][2];
                yokolines3[i][3] = yokolines3[j][3];
                yokolines3[i][4] = yokolines3[j][4];

                yokotheta[j] = tmp;
                yokolines3[j][0] = tmp1x;
                yokolines3[j][1] = tmp1y;
                yokolines3[j][2] = tmp2x;
                yokolines3[j][3] = tmp2y;
                yokolines3[j][4] = tmp3;
            }
        }
    }

    //クラスタリング-----------------------------------------------------------------------------------------------------------------------
    //タテ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCT:クラスタ数,Clust[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    //clusR=0.15,clusCT=0,Average_tate_theta=0;
    clusR=0.5,clusCT=0,Average_tate_theta=0;
    //要素数が0の時は実行しない
    if(tateyouso==0){
        std::cout <<"実行しない--------------tateyouso="<<tateyouso<< std::endl;
        tate_histgram = false;//タテ線のヒストグラムを実行しない
    }
    else{
        tate_histgram = true;//タテ線のヒストグラムを実行
        for (int j=0; j< tateyouso; ++j) {
            if(tateyouso==1){//要素がひとつしかないとき比較ができない
                std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                std::cout <<"クラスタ番号clusCT="<<clusCT<< std::endl;//クラスタ数
                std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                tateclust[0][clust[0]][0]=tatelines3[j][0];//(x成分)
                tateclust[0][clust[0]][1]=tatelines3[j][1];//(y成分)
                tateclust[0][clust[0]][2]=tatelines3[j][2];//(x成分)
                tateclust[0][clust[0]][3]=tatelines3[j][3];//(y成分)
                tateclust[0][clust[0]][4]=tatelines3[j][4];//角度
                clust[MAXT]=1;
            }

            else{
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",clusCT="<<clusCT<<"----------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        MAXT=clusCT;
                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //中間動作
                if(j!=0&&j+1!=tateyouso){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //後方クラスタリング半径範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //後方クラスタリング半径範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",clusCT="<<clusCT<<"--------------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //最終動作
                if(j+1==tateyouso){
                    //std::cout <<"最終動作j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                    tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                    tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                    tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                    tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                    cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                    cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                    //std::cout <<"最終:最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<< std::endl;
                }
                if(clust[clusCT]>=clust[MAXT]){ MAXT=clusCT; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<<"\n"<< std::endl;
            }
        }
        //線のグループ化-----------------------------------------------------------------------------------------------------------
        //短い線はノイズなので除去する
        for (int i=0; i<clust[MAXT]; ++i) {
            if(sqrt((tateclust[MAXT][i][0]-tateclust[MAXT][i][2])*(tateclust[MAXT][i][0]-tateclust[MAXT][i][2])
             +(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])*(tateclust[MAXT][i][1]-tateclust[MAXT][i][3]))>40){
                tateclust[MAXT][tateok][0]=tateclust[MAXT][i][0];
                tateclust[MAXT][tateok][1]=tateclust[MAXT][i][1];
                tateclust[MAXT][tateok][2]=tateclust[MAXT][i][2];
                tateclust[MAXT][tateok][3]=tateclust[MAXT][i][3];
                tateok=tateok+1;//縦線の合計個数(まとめ後)
            }
        }
        //Xの値が小さい順に並び変える
        tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
        for (int i=0; i<tateok; ++i) {
           for (int j=i+1;j<tateok; ++j) {
                if (tateclust[MAXT][i][0]+tateclust[MAXT][i][2] > tateclust[MAXT][j][0]+tateclust[MAXT][j][2]) {
                    tmp1x =  tateclust[MAXT][i][0];
                    tmp1y =  tateclust[MAXT][i][1];
                    tmp2x =  tateclust[MAXT][i][2];
                    tmp2y =  tateclust[MAXT][i][3];
                    tmp3 =  tateclust[MAXT][i][4];

                    tateclust[MAXT][i][0] = tateclust[MAXT][j][0];
                    tateclust[MAXT][i][1] = tateclust[MAXT][j][1];
                    tateclust[MAXT][i][2] = tateclust[MAXT][j][2];
                    tateclust[MAXT][i][3] = tateclust[MAXT][j][3];
                    tateclust[MAXT][i][4] = tateclust[MAXT][j][4];

                    tateclust[MAXT][j][0] = tmp1x;
                    tateclust[MAXT][j][1] = tmp1y;
                    tateclust[MAXT][j][2] = tmp2x;
                    tateclust[MAXT][j][3] = tmp2y;
                    tateclust[MAXT][j][4] = tmp3;
                }
            }
            //tateclust[MAXT][j][1]の位置を上にする
            if(tateclust[MAXT][i][1]>tateclust[MAXT][i][3]){
                tempy=tateclust[MAXT][i][1];
                tempx=tateclust[MAXT][i][0];
                tateclust[MAXT][i][1]=tateclust[MAXT][i][3];
                tateclust[MAXT][i][0]=tateclust[MAXT][i][2];
                tateclust[MAXT][i][3]=tempy;
                tateclust[MAXT][i][2]=tempx;
            }
            TATE_L[i]=sqrt((tateclust[MAXT][i][0]-tateclust[MAXT][i][2])*(tateclust[MAXT][i][0]-tateclust[MAXT][i][2])
             +(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])*(tateclust[MAXT][i][1]-tateclust[MAXT][i][3]));//縦線の点間の長さ
            TATE_XN[i]=(tateclust[MAXT][i][0]+tateclust[MAXT][i][2])/2;//縦線の中点N
            TATE_YN[i]=(tateclust[MAXT][i][1]+tateclust[MAXT][i][3])/2;
        }

        double clusTATE_R=10;//しきい値
        //縦線の中点と直線の距離を使用し、直線をまとめる(距離がしきい値以内なら同じ線とみなす)
       for (int j=0; j<tateok; ++j) {
            int equal=0;//同じ線の要素数
            TATE_line[tateno][0]=tateclust[MAXT][j][0],TATE_line[tateno][1]=tateclust[MAXT][j][1];//基準はjの線(更新が起きなければj単体)
            TATE_line[tateno][2]=tateclust[MAXT][j][2],TATE_line[tateno][3]=tateclust[MAXT][j][3];
            double TATE_MAX_LONG=TATE_L[j];
            double TATE_MIN=tateclust[MAXT][j][1];
            double TATE_MAX=tateclust[MAXT][j][3];

            //縦線がy軸と平行でない時のみ実行
            if(tateclust[MAXT][j][0]!=tateclust[MAXT][j][2]){
                //std::cout <<"縦線が斜めのとき:tatethetal["<<j<<"]"<< std::endl;

                TATE_A[j]=(tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]);
                TATE_C[j]=tateclust[MAXT][j][1]-(((tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]))*tateclust[MAXT][j][0]);
                for (int k=j+1; k<tateok; ++k) {
                    TATE_D[j][k]=abs((TATE_A[j]*TATE_XN[k])-TATE_YN[k]+TATE_C[j])/sqrt((TATE_A[j]*TATE_A[j])+1);//点と直線の距離
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        //std::cout <<"(斜め)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_A_MAX=(tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]);
                            TATE_C_MAX=tateclust[MAXT][k][1]-(((tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]))*tateclust[MAXT][k][0]);
                            //最も長い線の一次関数からy最大、最小のxを求める
                            TATE_line[tateno][0]=(TATE_line[tateno][1]-TATE_C_MAX)/TATE_A_MAX;
                            TATE_line[tateno][2]=(TATE_line[tateno][3]-TATE_C_MAX)/TATE_A_MAX;
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い
                }
            }
            //縦線がy軸と平行なとき
            else{
                //std::cout <<"縦線がY軸と平行:tatethetal["<<j<<"]"<< std::endl;
                for (int k=j+1; k<tateok; ++k) {
                    TATE_D[j][k]=abs(tateclust[MAXT][j][0]-TATE_XN[k]);//点と直線の距離(y軸並行)
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        //std::cout <<"(平行)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_line[tateno][0]=tateclust[MAXT][k][0];//最大長さのxを保存
                            TATE_line[tateno][2]=tateclust[MAXT][k][2];
                            
                        }
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い
                }
            }
            cv::circle(img_dst2, cv::Point(TATE_line[tateno][0], TATE_line[tateno][1]), 3, Scalar(255,0,0), -1, cv::LINE_AA);//線の角度グラフ
            cv::circle(img_dst2, cv::Point(TATE_line[tateno][2], TATE_line[tateno][3]), 3, Scalar(0,255,0), -1, cv::LINE_AA);//線の角度グラフ

            //短い線はノイズなので除去する
            if(sqrt((TATE_line[tateno][0]-TATE_line[tateno][2])*(TATE_line[tateno][0]-TATE_line[tateno][2])
             +(TATE_line[tateno][1]-TATE_line[tateno][3])*(TATE_line[tateno][1]-TATE_line[tateno][3]))>40){
                tateno=tateno+1;//縦線の合計個数(まとめ後)
            }
            j=j+equal;//同じ線とみなされた個数分進む
        }
        //std::cout <<"縦線のクラスタ数(まとめ前)clust[MAXT]="<<clust[MAXT]<< std::endl;
        //std::cout <<"縦線の合計個数(まとめ後)tateno="<<tateno<< std::endl;
        //-------------------------------------------------------------------------------------------------------------------------------
        tate_point_curr.resize(1000);//配列初期設定(縦線の中点座標)

        int tateno2=0;
        //一次関数を描写するプログラム
        for (int j=0; j<tateno; ++j) {
            //std::cout <<"j="<< j<< std::endl;
            //std::cout <<"clust["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"tateclust["<<MAXT<<"]["<<j<<"][0]="<<TATE_line[j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数
            tatethetal[j]=(M_PI/2)-(M_PI-atan2((TATE_line[j][2]-TATE_line[j][0]),(TATE_line[j][3]-TATE_line[j][1])));
            tatelc[j]=(TATE_line[j][2]-TATE_line[j][0])*(TATE_line[j][2]-TATE_line[j][0])+(TATE_line[j][3]-TATE_line[j][1])*(TATE_line[j][3]-TATE_line[j][1]);
            tatel[j][0]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*100;//X1座標
            tatel[j][1]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*100;//Y1座標
            tatel[j][2]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*-100;//X2座標
            tatel[j][3]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*-100;//Y2座標

            cv::line(img_line4,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst2,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_tate,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_tmp,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst2,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            cv::line(img_tate,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            cv::line(img_tmp,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);

            datat[j]=TATE_line[clust[j]][4];
            //最大クラスタ内の平均角度を求める
            Average_tate_theta=Average_tate_theta+datat[j];

            //縦線の中点を求める
            tate_point_curr[j].x=(TATE_line[j][0]+TATE_line[j][2])/2;
            tate_point_curr[j].y=(TATE_line[j][1]+TATE_line[j][3])/2;
	        cv::circle(img_dst2, cv::Point(tate_point_curr[j]), 3, Scalar(0,255,255),  -1, cv::LINE_AA);
	        cv::circle(img_tmp, cv::Point(tate_point_curr[j]), 3, Scalar(0,255,255),  -1, cv::LINE_AA);

            //tate_point_curr[tateno2].x=TATE_line[j][0];
            //tate_point_curr[tateno2].y=TATE_line[j][1];
            //tate_point_curr[tateno2+1].x=TATE_line[j][2];
            //tate_point_curr[tateno2+1].y=TATE_line[j][3];
	        //cv::circle(img_dst2, cv::Point(tate_point_curr[tateno2]), 3, Scalar(0,255,255),  -1, cv::LINE_AA);
	        //cv::circle(img_dst2, cv::Point(tate_point_curr[tateno2+1]), 3, Scalar(0,255,255),  -1, cv::LINE_AA);
            //tateno2=tateno2+2;
        }
        tate_point_curr.resize(tateno);//リサイズ(縦線の中点座標)
        //tate_point_curr.resize(tateno2);//リサイズ(縦線の中点座標)

        //最大クラスタの要素数が１つだけの時を考慮
        if(clust[MAXT]>1){Average_tate_theta=Average_tate_theta/(clust[MAXT]-1);}//最大クラスタの要素が２つ以上なら通常の平均計算
        else{Average_tate_theta=Average_tate_theta;}//最大クラスタの要素数が２つ未満ならその値を平均値とする

        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta),380+100*cos(Average_tate_theta)),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta<<"\n"<< std::endl;
        //cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta/(clust[MAXT]-1)),380+100*cos(Average_tate_theta/(clust[MAXT]-1))),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta/(clust[MAXT]-1)<<"\n"<< std::endl;

        //縦線の法線ベクトルを求める-----------------------------------------
         MatrixXd R(2,2);
         R(0,0)=cos(M_PI/2);
         R(0,1)=-sin(M_PI/2);
         R(1,0)=sin(M_PI/2);
         R(1,1)=cos(M_PI/2);

        MatrixXd n(2,1);
        MatrixXd N(2,1);//Nは2行L列の行列
        N(0,0)=100-100*sin(Average_tate_theta);
        N(1,0)=380+100*cos(Average_tate_theta);
        //N(0,0)=100-100*sin(Average_tate_theta/(clust[MAXT]-1));
        //N(1,0)=380+100*cos(Average_tate_theta/(clust[MAXT]-1));

        MatrixXd P0(2,1);//推定交点
        P0(0,0)=(100+(100-100*sin(Average_tate_theta)))/2;
        P0(1,0)=(380+(380+100*cos(Average_tate_theta)))/2;
        //P0(0,0)=(100+(100-100*sin(Average_tate_theta/(clust[MAXT]-1))))/2;
        //P0(1,0)=(380+(380+100*cos(Average_tate_theta/(clust[MAXT]-1))))/2;

        n=R*(P0-N);//直線を90度回転させたベクトルn
        cv::line(img_graph,cv::Point(P0(0,0),P0(1,0)),cv::Point(P0(0,0)-n(0,0),P0(1,0)-n(1,0)),cv::Scalar(255,0,255), 3, cv::LINE_AA);//法線ベクトル

        //縦線の平均方向の法線ベクトルとy軸との角度を求める
        Average_tate_theta_Y=M_PI-atan2(-n(0,0),-n(1,0));
        std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<<"\n"<< std::endl;

        //次はこの法線ベクトルの角度に最も近いヨコ線のクラスタを求める
        //求める方法としては各クラスタの平均角度を求め、その平均角度と比較して最も比較の値が小さいやつをヨコ線のクラスタとする。
        //ただしその比較値がある値以上だった場合はヨコ線の方向は更新せず前ステップでのヨコ線の方向を使う。

        //縦線が検出されなかった場合もまた同様に縦線の方向は前ステップでの方向を利用する
    }

    std::cout <<"\n"<< std::endl;

    //ヨコ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCY:クラスタ数,Clusy[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    clusR=0.15,clusCY=0;
    //要素数が0の時は実行しない
    if(yokoyouso==0){
        std::cout <<"実行しない--------------yokoyouso="<<yokoyouso<< std::endl;
        yoko_histgram = false;//ヨコ線のヒストグラムを実行しない
    }
    else{
        yoko_histgram = true;//ヨコ線のヒストグラムを実行
        for (int j=0; j< yokoyouso; ++j) {
            if(yokoyouso==1){//要素がひとつしかない時は比較ができない(クラスタリングできない)
                std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                std::cout <<"クラスタ番号clusCY="<<clusCY<< std::endl;//クラスタ数
                std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                yokoclusy[0][clusy[0]][0]=yokolines3[j][0];//(x成分)
                yokoclusy[0][clusy[0]][1]=yokolines3[j][1];//(y成分)
                yokoclusy[0][clusy[0]][2]=yokolines3[j][2];//(x成分)
                yokoclusy[0][clusy[0]][3]=yokolines3[j][3];//(y成分)
                yokoclusy[0][clusy[0]][4]=yokolines3[j][4];
                clusy[MAXY]=1;
            }

            else{//クラスタリング
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];// 角度
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す(初回動作)
                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<< ",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度
                        
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);
                        MAXY=clusCY;

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求める(初回動作クラスタリング範囲外なのでデータが１つしかない)
                        //std::cout <<"初回動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での平均値初期化
                    }
                }
                //中間動作
                if(j!=0&&j+1!=yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //後方範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //後方範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        
                        if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                        }
                        else{//クラスタの要素が一つしかない時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均=要素(要素がひとつしかないから=平均)
                        }
                        //std::cout <<"中間動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;
                        //要素数が3個以上のクラスタをキープする(開始要素が0を考慮)
                        if(clusy[clusCY]>=3-1){
                            clus_no=clus_no+1;
                            //std::cout <<"要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                            CLUSTER[clus_no]=clusCY;//クラスタ番号を CLUSTER[clus_no]に保存
                        }
                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での次の平均値初期化
                    }
                }
                //最終動作(j<yokoyouso個)
                if(j+1==yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"最終動作j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                    cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                    cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                    Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                    if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                    }
                    else{//クラスタの要素が一つしかない時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均を求める
                    }
                    //std::cout <<"最終動作:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<< std::endl;
                    //要素数が3個以上のクラスタをキープする
                    if(clusy[clusCY]>=3-1){
                        clus_no=clus_no+1;
                        //std::cout <<"最終動作:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                        CLUSTER[clus_no]=clusCY;
                    }
                }

                //if(clusy[clusCY]>=clusy[MAXY]){ MAXY=clusCY; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXY=clusCY="<<MAXY<<",最大クラスタの内部個数clusy[MAXY]="<<clusy[MAXY]<< std::endl;
            }
        }
        //クラスタリングの要素数が３つ以上のクラスタがあるときだけ実行
        if(clus_no>0){
            //std::cout <<"テスト:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
            for(int h=1;h<=clus_no;h++){
                //std::cout <<"要素数が3個以上のクラスタ  クラスタ番号clusCY=CLUSTER[clus_no="<<h<<"]="<<CLUSTER[h]<<std::endl;
                //std::cout <<"Average_yoko_theta["<<CLUSTER[h]<<"]="<<Average_yoko_theta[CLUSTER[h]]<< std::endl;
                //縦線の法線ベクトルとヨコ線の平均角度の差を見てる
                if(MINI_theta>abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]])){
                    MINI_theta=abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]]);
                    YOKO_CLUST=CLUSTER[h];
                }
            }
            //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<< std::endl;
            //std::cout <<"タテの法線ベクトルと最も並列なヨコ線クラスタ clusCY=YOKO_CLUST="<<YOKO_CLUST<<",Average_yoko_theta_Y="<<Average_yoko_theta[YOKO_CLUST]<< std::endl;
            cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_yoko_theta[YOKO_CLUST]),380+100*cos(Average_yoko_theta[YOKO_CLUST])),cv::Scalar(255,0,0), 3, cv::LINE_AA);

            //std::cout <<"clusy[YOKO_CLUST="<<YOKO_CLUST<<"]="<<clusy[YOKO_CLUST]<< std::endl;

            //一次関数を描写するプログラム
            for(int j = 0; j <clusy[YOKO_CLUST]; j++){
                //std::cout <<"j="<< j<< std::endl;
                //std::cout <<"clusy["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
                //std::cout <<"yokoclusy["<<YOKO_CLUST<<"]["<<j<<"][0]="<<yokoclusy[YOKO_CLUST][j][0]<< std::endl;//確認用

                //座標から一次関数を引く関数(線を延長してる)
                yokothetal[j]=(M_PI/2)-(M_PI-atan2((yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0]),(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])));
                yokolc[j]=(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])*(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])+(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])*(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1]);
                yokol[j][0]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*100;//X1座標
                yokol[j][1]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*100;//Y1座標
                yokol[j][2]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*-100;//X2座標
                yokol[j][3]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*-100;//Y2座標

                cv::line(img_line4,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::line(img_dst,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::line(img_dst,cv::Point(yokoclusy[YOKO_CLUST][j][0],yokoclusy[YOKO_CLUST][j][1]),
                 cv::Point(yokoclusy[YOKO_CLUST][j][2],yokoclusy[YOKO_CLUST][j][3]),cv::Scalar(255,0,0), 4, cv::LINE_AA);

                datay[j]=yokoclusy[YOKO_CLUST][j][4];
            }
        }
    }
    //タテ線とナナメ線の交点を検出する--------------------------------------------------------------------------------------------------
    //縦線の一次関数を求める
    for (int j=0; j<lines_NNM_count; ++j) {
        NNMA[j]=(lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]);
        NNMB[j]=lines_NNM[j][1]-(((lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]))*lines_NNM[j][0]);
        for (int i=0; i<clust[MAXT]; ++i) {
            if(tateclust[MAXT][i][0]!=tateclust[MAXT][i][2]){
                tateA[i]=(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])/(tateclust[MAXT][i][0]-tateclust[MAXT][i][2]);
                tateB[i]=tateclust[MAXT][i][1]-tateA[i]*tateclust[MAXT][i][0];
                NNM_TATE_X[j][i]=(tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]);
                NNM_TATE_Y[j][i]=NNMA[j]*((tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]))+NNMB[j];
            }
            else{
                NNM_TATE_X[j][i]=tateclust[MAXT][i][0];
                NNM_TATE_Y[j][i]=(NNMA[j]*NNM_TATE_X[j][i])+NNMB[j];
            }
            //std::cout <<"縦線とヨコ線の交点["<<j<<"]["<<i<<"]=("<<NNM_TATE_X[j][i]<<","<<NNM_TATE_Y[j][i]<<")"<< std::endl;
            //cv::circle(img_dst,cv::Point(NNM_TATE_X[j][i],NNM_TATE_Y[j][i]),3,Scalar(0,255,255),-1);
        }
    }
    //線検出プログラム終了-----------------------------------------------------------------------------------------------------------------------
    TPCC.resize(1000);//配列初期設定(tate_point_camera_c)
    DTPC_ok=0;
    //テンプレートマッチングプログラム
    //縦線の中点を使用してテンプレートを作成する(線検出の段階ではDepthの不具合などは考慮していないためここで考慮する+クロップ不具合)
    for (int i=0; i<tate_point_curr.size(); ++i) {
        if(template_size<tate_point_curr[i].y&&tate_point_curr[i].y<480-template_size){
            if(template_size<tate_point_curr[i].x&&tate_point_curr[i].x<640-template_size){
                DTPC[i] = img_depth.at<float>(cv::Point(tate_point_curr[i].x,tate_point_curr[i].y));//depth_tate_point_curr=DTPC
                //Depthが取得できない縦線中点を削除する+Depthの外れ値を除く
                if(DTPC[i]>0.001&&DTPC[i]<10000){//tate_point_camera_c=TPCC
                    TPCC[DTPC_ok].x = -DTPC[i] * ((tate_point_curr[i].x - 320) / 615.337)/1000;//メートル表示変換
                    TPCC[DTPC_ok].y = -DTPC[i] * ((tate_point_curr[i].y - 240) / 615.458)/1000;
                    TPCC[DTPC_ok].z = DTPC[i]/1000;
                    tate_point_curr[DTPC_ok] = tate_point_curr[i];
	    	            cv::circle(img_tate, tate_point_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
                    //テンプレート作成----------------------------------------------------------------------------------------
                    std::cout <<"縦線中点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
                    cv::Rect roi2(cv::Point(tate_point_curr[i].x-template_size,tate_point_curr[i].y-template_size), cv::Size(template_size*2, template_size*2));//縦線中点を中心とした16☓16pixelの画像を切り取る
                    TPCC_Templ[DTPC_ok] = img_src(roi2); // 切り出し画像
                    cv::rectangle(img_tate, roi2,cv::Scalar(255, 255, 255), 2);//テンプレート位置
                    cv::rectangle(img_tmp, roi2,cv::Scalar(255, 255, 255), 2);//テンプレート位置
                    if(i==0){cv::rectangle(img_tmp, roi2,cv::Scalar(255, 255, 0), 2);}//テンプレート位置
                    //cv::rectangle(img_master_temp, cv::Point(tate_point_curr[i].x-template_size,tate_point_curr[i].y+template_size), 
                    //cv::Point(tate_point_curr[i].x+template_size,tate_point_curr[i].y-template_size), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)
                    std::cout <<"縦線中点の画像座標(DTPC_ok)["<<DTPC_ok<<"]="<<tate_point_curr[DTPC_ok]<< std::endl;//縦線中点の座標(範囲制限後)
                    DTPC_ok=DTPC_ok+1;//Depth取得可能+テンプレ制限範囲内の個数をカウント
                }
            }
        }
    }
    tate_point_curr.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    tate_point_prev.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    TPCC.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_c)
    TPCP.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)
    Est_tate_point.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)
    Est_tate_pixel.resize(DTPC_ok);//Depth取得数でリサイズ(tate_point_camera_prev)

    kensyutu=true;
    Tracking2=true;
    std::cout <<"初回検出プログラム終了"<< std::endl;
  }//if(reset == true)→end

  //テンプレート範囲予測とテンプレートマッチング----------------------------------------------------------------------
  //テンプレートマッチングは1つ前のテンプレートを使用(テンプレート:t+n-1,マッチング画像:t+n)
  else{//初回以降動作
    if(Tracking2==true){
      MTPC.resize(1000);//Matching_Tate_Pixel_Curr=MTPC定義(ここの大きさを超えるとエラーになる)
      //ロボットの動きから特徴点の運動復元を行う(特徴点の状態方程式を使用)
      std::cout <<"二回目動作:DTPP_ok="<<DTPP_ok<< std::endl;

      matchT_curr=0;
      for(int i=0;i<DTPP_ok;i++){
        Est_tate_point[i].x=-TPCP[i].x+robot_odometry.twist.twist.angular.z*realsec*TPCP[i].z+Act_RobotV*sin(-Act_RobotTH)*realsec;
        Est_tate_point[i].y=-TPCP[i].y;
        Est_tate_point[i].z=TPCP[i].z-robot_odometry.twist.twist.angular.z*realsec*-TPCP[i].x+Act_RobotV*cos(-Act_RobotTH)*realsec;

        Est_tate_pixel[i].x=320+(Est_tate_point[i].x/Est_tate_point[i].z)*615.337;
        Est_tate_pixel[i].y=240+(Est_tate_point[i].y/Est_tate_point[i].z)*615.458;

        //cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x,Est_tate_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
        cv::line(img_tate,cv::Point(tate_point_prev[i].x,tate_point_prev[i].y),cv::Point(Est_tate_pixel[i].x,Est_tate_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);


        //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
        std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;

        //予測範囲が全て画面内の時
        if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
          std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          //cv::imshow("EST_tate_scope", EST_tate_scope[i]);//黄色の特徴点を中心としたクロップ画像
        }
        //左側
        else if(0<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<cropx){
          //左上(xとyどちらもはみでる)
          if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_tate_pixel[i].x+cropx, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
           cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左側(xははみ出ない)
          else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_tate_pixel[i].y-cropy), cv::Size(Est_tate_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //左下(xとyどちらもはみでる)
          else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
            std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(0,Est_tate_pixel[i].y-cropy), cv::Size(Est_tate_pixel[i].x+cropx, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //上側(yははみ出ない)
        else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
          std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        //右側
        else if(640-cropx<Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640){
          //右上(xとyどちらもはみでる)
          if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,0), cv::Size(640-Est_tate_pixel[i].x+cropx, Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右側(xははみ出ない)
          else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(640-Est_tate_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
          //右下(xとyどちらもはみでる)
          else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
            std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(640-Est_tate_pixel[i].x+cropx, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
          }
        }
        //下側(yははみ出ない)
        else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&480-cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<480){
          std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
          cv::Rect roiEST(cv::Point(Est_tate_pixel[i].x-cropx,Est_tate_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_tate_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
          EST_tate_scope[i] = img_src(roiEST); // 切り出し画像
          cv::rectangle(img_tate, roiEST,cv::Scalar(0, 255, 255), 2);//マッチング予測範囲
        }
        else{
          std::cout << "画面外["<<i<<"]"<< std::endl;
        }

        //std::cout <<"TPCP_Templ[i].cols="<<TPCP_Templ[i].cols<<"TPCP_Templ[i].rows="<<TPCP_Templ[i].rows<< std::endl;//最初のフレーム
        //std::cout <<"EST_tate_scope[i].cols="<<EST_tate_scope[i].cols<<"EST_tate_scope[i].rows="<<EST_tate_scope[i].rows<< std::endl;//最初のフレーム

        //予測範囲に対しテンプレートマッチングを行う
        std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
        TPCP_Templ[i].copyTo(img_template1); // 切り出し画像
        //cv::imshow("win_TPCP_Templ", TPCP_Templ[i]);//黄色の特徴点を中心としたクロップ画像

        cv::Mat img_minmax1;
        // テンプレートマッチング
        cv::matchTemplate(EST_tate_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
        cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
        std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
        std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
          /*if(i==0){
            min_match0<<min_val1[i]<<"\n";
            max_match0<<max_val1[i]<<"\n";
            match0_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match0_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time0<<ALLrealsec<<"\n";
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);
            }
          else if(i==1){
            min_match1<<min_val1[i]<<"\n";
            max_match1<<max_val1[i]<<"\n";
            match1_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match1_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time1<<ALLrealsec<<"\n";
            }
          else if(i==2){
            min_match2<<min_val1[i]<<"\n";
            max_match2<<max_val1[i]<<"\n";
            match2_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match2_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time2<<ALLrealsec<<"\n";
            }
          else if(i==3){
            min_match3<<min_val1[i]<<"\n";
            max_match3<<max_val1[i]<<"\n";
            match3_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match3_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time3<<ALLrealsec<<"\n";
            }
          else if(i==4){
            min_match4<<min_val1[i]<<"\n";
            max_match4<<max_val1[i]<<"\n";
            match4_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match4_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time4<<ALLrealsec<<"\n";
            }
          else if(i==5){
            min_match5<<min_val1[i]<<"\n";
            max_match5<<max_val1[i]<<"\n";
            match5_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match5_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time5<<ALLrealsec<<"\n";
            }
          else if(i==6){
            min_match6<<min_val1[i]<<"\n";
            max_match6<<max_val1[i]<<"\n";
            match6_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match6_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time6<<ALLrealsec<<"\n";
            }
          else if(i==7){
            min_match7<<min_val1[i]<<"\n";
            max_match7<<max_val1[i]<<"\n";
            match7_pixel_x<<max_pt1[i].x+img_template1.cols/2<<"\n";
            match7_pixel_y<<max_pt1[i].y+img_template1.rows/2<<"\n";
            ALLtemp_time7<<ALLrealsec<<"\n";
            }*/

        if(0.8<max_val1[i]){//最小値がしきい値以下なら表示
          //予測範囲が全て画面内の時
          if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
            std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像(Matching_Tate_Templ_curr=MTTC)
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
             //左側
          else if(0<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
              std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
              MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
              std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
              matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
              std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
              std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
          }
          //上側(yははみ出ない)
          else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
            std::cout << "マッチング:上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
          //右側
          else if(640-cropx<Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<cropy){
              std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480-cropy){
              std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_tate_pixel[i].y&&Est_tate_pixel[i].y<=480){
              std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_tate_pixel[i].x&&Est_tate_pixel[i].x<=640-cropx&&480-cropy<=Est_tate_pixel[i].y&&Est_tate_pixel[i].y<480){
            std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::rectangle(img_tate, cv::Rect(Est_tate_pixel[i].x-cropx+max_pt1[i].x, Est_tate_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
            cv::circle(img_tate, cv::Point(Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
            MTPC[matchT_curr].x=Est_tate_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
            MTPC[matchT_curr].y=Est_tate_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
            MTTC[matchT_curr]=TPCP_Templ[i];//マッチングしたテンプレート画像
            std::cout <<"マッチングの中心座標[matchT_curr="<<matchT_curr<<"]="<<MTPC[matchT_curr]<< std::endl;
            matchT_curr=matchT_curr+1;//マッチングの中心座標個数
          }
          else{
            std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
      }//for(int i=0;i<DTPP_ok;i++)→end (範囲予測+テンプレートマッチング)
      std::cout <<"matchT_curr="<<matchT_curr<< std::endl;
      temp_kosuu<<matchT_curr<<"\n";
      MTPC.resize(matchT_curr);//Depth取得可能数でリサイズ(運動復元画像座標)
      MT_curr_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
      MT_curr_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)
      std::cout <<"test"<< std::endl;

      //マッチングしたテンプレートをマッチテンプレートとしてキープする
      //マッチテンプレートのDepthが取得不能な点を削除
      DMT_curr_ok=0;
      for (int i = 0; i < matchT_curr; i++) {
        DMT_curr[i] = img_depth.at<float>(cv::Point(MTPC[i].x,MTPC[i].y));//DMT_curr=Depth_MTPC
        //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
        if(DMT_curr[i]>0.001&&DMT_curr[i]<10000){
          MT_curr_Templ[DMT_curr_ok] = MTTC[i];//Depth取得可能なマッチテンプレート
          MT_curr_pixel[DMT_curr_ok] = MTPC[i];//Depth取得可能なマッチング中心画像座標
          MT_curr_camera[DMT_curr_ok].x = -DMT_curr[i] * ((MTPC[i].x - 320) / 615.337)/1000;//カメラ座標変換
          MT_curr_camera[DMT_curr_ok].y = -DMT_curr[i] * ((MTPC[i].y - 240) / 615.458)/1000;
          MT_curr_camera[DMT_curr_ok].z = DMT_curr[i]/1000;
          //std::cout <<"MT_curr_pixel["<<DMT_curr_ok<<"]="<<MT_curr_pixel[DMT_curr_ok]<< std::endl;
          //std::cout <<"MT_curr_camera["<<DMT_curr_ok<<"]="<<MT_curr_camera[DMT_curr_ok]<< std::endl;
          //std::cout <<"MT_curr_Templ["<<DMT_curr_ok<<"].cols="<<MT_curr_Templ[DMT_curr_ok].cols<<"MT_curr_Templ["<<DMT_curr_ok<<"].rows="<<MT_curr_Templ[DMT_curr_ok].rows<< std::endl;//最初のフレーム
          DMT_curr_ok=DMT_curr_ok+1;//Depthが取得可能なマッチテンプレート数
          //DMT_prev_ok1=DMT_prev_ok1+1;//新規テンプレート数
        }
        else{
          std::cout <<"Depth エラーMTPC["<<i<<"]="<<MTPC[i]<< std::endl;
        }
		  }
      std::cout <<"新規テンプレート数:DMT_curr_ok="<<DMT_curr_ok<< std::endl;
      //類似するテンプレートを削除する
      int notsimil=0;
      for (int i = 0; i < DMT_curr_ok; i++) {
        int similar=0;
        for (int j = i+1; j < DMT_curr_ok; j++) {
          length=sqrt((MT_curr_pixel[i].x-MT_curr_pixel[j].x)*(MT_curr_pixel[i].x-MT_curr_pixel[j].x)
            +(MT_curr_pixel[i].y-MT_curr_pixel[j].y)*(MT_curr_pixel[i].y-MT_curr_pixel[j].y));
          if(template_size*2>length){
            similar=similar+1;
          }
        }
        //類似するテンプレートを繰り上げ削除
        if(similar==0){
          MT_curr_Templ[notsimil]=MT_curr_Templ[i];
          MT_curr_pixel[notsimil]=MT_curr_pixel[i];
          MT_curr_camera[notsimil]=MT_curr_camera[i];
          notsimil=notsimil+1;
        }
        else{
          std::cout <<"類似する削除MT_curr_pixel["<<i<<"]="<<MT_curr_pixel[i]<< std::endl;
        }
      }
      DMT_curr_ok=notsimil;
      std::cout <<"新規テンプレート数(修正後):DMT_curr_ok="<<DMT_curr_ok<< std::endl;
      //if(DMT_curr_ok<=1){reset=true;}

      MT_curr_pixel.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
      MT_curr_camera.resize(DMT_curr_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)
      }

      //3回目以降動作
      else if(Tracking2 == false){
        std::cout <<"3回目以降動作-------------------------------------------------------"<< std::endl;
        //ロボットの動きから一つ前のマッチング座標の運動復元を行う(特徴点の状態方程式を使用)
        EST_MT_ok=0;
        Est_MT_point.resize(1000);//配列初期設定
        Est_MT_pixel.resize(1000);//配列初期設定
        for(int i=0;i<DMT_prev_ok;i++){
          Est_MT_point[i].x=-MT_prev_camera[i].x+robot_odometry.twist.twist.angular.z*realsec*MT_prev_camera[i].z+Act_RobotV*sin(-Act_RobotTH)*realsec;
          Est_MT_point[i].y=-MT_prev_camera[i].y;
          Est_MT_point[i].z=MT_prev_camera[i].z-robot_odometry.twist.twist.angular.z*realsec*-MT_prev_camera[i].x+Act_RobotV*cos(-Act_RobotTH)*realsec;

          Est_MT_pixel[i].x=320+(Est_MT_point[i].x/Est_MT_point[i].z)*615.337;
          Est_MT_pixel[i].y=240+(Est_MT_point[i].y/Est_MT_point[i].z)*615.458;

          //cv::circle(img_tate, cv::Point(Est_pixel[i].x,Est_pixel[i].y), 6, Scalar(0,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
          //一つ前のマッチ画像座標と次の推定画像座標
          cv::line(img_tate,cv::Point(MT_prev_pixel[i].x,MT_prev_pixel[i].y),cv::Point(Est_MT_pixel[i].x,Est_MT_pixel[i].y),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)

          std::cout <<"Est_MT_point["<<i<<"]=\n"<<Est_MT_point[i]<< std::endl;//予測カメラ座標
          std::cout <<"Est_MT_pixel["<<i<<"]=\n"<<Est_MT_pixel[i]<< std::endl;//予測画像座標

          //求めた運動復元結果からテンプレートマッチングの予測範囲を作る(とりあえずタテヨコ2倍)
          //ここで予測点が画面外に行ったらそのテンプレートを削除する
          std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;
          //MTcoix[i]=Est_MT_pixel[i].x+template_size;//予測範囲の中心座標
          //MTcoiy[i]=Est_MT_pixel[i].y+template_size;//予測範囲の中心座標
          //予測範囲が全て画面内の時
          if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
            std::cout << "予測範囲が全て画面内の時["<<i<<"]"<< std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropx), cv::Size(cropx*2, cropx*2));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
           // cv::imshow("EST_MT_scope", EST_MT_scope[EST_MT_ok]);//黄色の特徴点を中心としたクロップ画像
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            std::cout << "予測範囲の端点["<<i<<"]("<<Est_MT_pixel[i].x-cropx<<","<<Est_MT_pixel[i].y-cropx<<"),size=("<<cropx*2<<","<<cropx*2<<")"<< std::endl;
          }
           //左側
          else if(0<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<cropx){
            //左上(xとyどちらもはみでる)
            if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              std::cout << "左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,0), cv::Size(Est_MT_pixel[i].x+cropx, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左側(xははみ出ない)
            else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              std::cout << "左側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MT_pixel[i].y-cropy), cv::Size(Est_MT_pixel[i].x+cropx, cropx*2));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //左下(xとyどちらもはみでる)
            else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
              std::cout << "左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(0,Est_MT_pixel[i].y-cropy), cv::Size(Est_MT_pixel[i].x+cropx, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //上側(yははみ出ない)
          else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
            std::cout << "上側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,0), cv::Size(cropx*2, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
          }
          //右側
          else if(640-cropx<Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640){
            //右上(xとyどちらもはみでる)
            if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              std::cout << "右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,0), cv::Size(640-Est_MT_pixel[i].x+cropx, Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右側(xははみ出ない)
            else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              std::cout << "右側(xははみ出ない)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(640-Est_MT_pixel[i].x+cropx, cropy*2));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
            //右下(xとyどちらもはみでる)
            else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
              std::cout << "右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
              cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(640-Est_MT_pixel[i].x+cropx, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
              EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
              MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
              MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
              Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
              Est_MT_point[EST_MT_ok] = Est_MT_point[i];
              MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
              MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
              MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
              cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
              EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
            }
          }
          //下側(yははみ出ない)
          else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&480-cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<480){
            std::cout << "下側(yははみ出ない)["<<i<<"]"<<std::endl;
            cv::Rect roiEST(cv::Point(Est_MT_pixel[i].x-cropx,Est_MT_pixel[i].y-cropy), cv::Size(cropx*2, 480-Est_MT_pixel[i].y+cropy));//線の中点を中心とした線の画像を切り取る
            EST_MT_scope[EST_MT_ok] = img_src(roiEST); // 切り出し画像
            MTcoix[EST_MT_ok]=Est_MT_pixel[i].x;
            MTcoiy[EST_MT_ok]=Est_MT_pixel[i].y;
            Est_MT_pixel[EST_MT_ok] = Est_MT_pixel[i];
            Est_MT_point[EST_MT_ok] = Est_MT_point[i];
            MT_prev_pixel[EST_MT_ok]=MT_prev_pixel[i];
            MT_prev_camera[EST_MT_ok]=MT_prev_camera[i];
            MT_prev_Templ[EST_MT_ok]=MT_prev_Templ[i];
            cv::rectangle(img_tate, roiEST,cv::Scalar(0, 50, 255), 2);//マッチング予測範囲
            EST_MT_ok=EST_MT_ok+1;//予測範囲が画面内のテンプレート数
          }
          else{
            std::cout << "画面外["<<i<<"]"<< std::endl;
          }
        }//for(int i=0;i<DMT_prev_ok;i++)→end (範囲予測+テンプレートマッチング)
        std::cout << "EST_MT_ok="<<EST_MT_ok<< std::endl;
        Est_MT_pixel.resize(EST_MT_ok);//リサイズ
        Est_MT_point.resize(EST_MT_ok);//リサイズ
        MT_prev_pixel.resize(EST_MT_ok);//リサイズ
        MT_prev_camera.resize(EST_MT_ok);//リサイズ
        MTPP.resize(1000);//配列初期設定

        //テンプレートマッチングが原因っぽい
        //予測範囲に対しテンプレートマッチングを行う
        matchT_prev=0;
        for(int i=0;i<EST_MT_ok;i++){
          std::cout <<"\n"<< std::endl;
          std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
          //MT_prev_Templ[i].copyTo(img_template1); // 切り出し画像(多分これが原因)
          img_template1 = MT_prev_Templ[i].clone();

          cv::Mat img_minmax1;
          // テンプレートマッチング
          cv::matchTemplate(EST_MT_scope[i], img_template1, img_minmax1, cv::TM_CCOEFF_NORMED);//正規化相互相関(ZNCC)
          cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
          std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
          std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
          

          if(0.75<max_val1[i]){//最小値がしきい値以下なら表示
            //予測範囲が全て画面内の時
            if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
              std::cout << "マッチング:全て画面内の時["<<i<<"]"<< std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
              std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
            if(i==0){
              min_match0<<min_val1[i]<<"\n";
              max_match0<<max_val1[i]<<"\n";
              match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time0<<ALLrealsec<<"\n";
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
              }
            else if(i==1){
              min_match1<<min_val1[i]<<"\n";
              max_match1<<max_val1[i]<<"\n";
              match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time1<<ALLrealsec<<"\n";
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
              }
            else if(i==2){
              min_match2<<min_val1[i]<<"\n";
              max_match2<<max_val1[i]<<"\n";
              match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time2<<ALLrealsec<<"\n";
            }
            else if(i==3){
              min_match3<<min_val1[i]<<"\n";
              max_match3<<max_val1[i]<<"\n";
              match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time3<<ALLrealsec<<"\n";
              }
            else if(i==4){
              min_match4<<min_val1[i]<<"\n";
              max_match4<<max_val1[i]<<"\n";
              match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time4<<ALLrealsec<<"\n";
              }
            else if(i==5){
              min_match5<<min_val1[i]<<"\n";
              max_match5<<max_val1[i]<<"\n";
              match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time5<<ALLrealsec<<"\n";
              }
            else if(i==6){
              min_match6<<min_val1[i]<<"\n";
              max_match6<<max_val1[i]<<"\n";
              match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match6_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time6<<ALLrealsec<<"\n";
              }
            else if(i==7){
              min_match7<<min_val1[i]<<"\n";
              max_match7<<max_val1[i]<<"\n";
              match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time7<<ALLrealsec<<"\n";
              }
              else if(i==8){
              min_match8<<min_val1[i]<<"\n";
              max_match8<<max_val1[i]<<"\n";
              match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
              match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
              ALLtemp_time8<<ALLrealsec<<"\n";
              }
           }
            //左側
            else if(0<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<cropx){
              //左上(xとyどちらもはみでる)
              if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
                std::cout << "マッチング:左上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              if(i==0){
                min_match0<<min_val1[i]<<"\n";
                max_match0<<max_val1[i]<<"\n";
                match0_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match0_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                ALLtemp_time0<<ALLrealsec<<"\n";
                }
              else if(i==1){
                min_match1<<min_val1[i]<<"\n";
                max_match1<<max_val1[i]<<"\n";
                match1_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match1_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
                ALLtemp_time1<<ALLrealsec<<"\n";
                }
              else if(i==2){
                min_match2<<min_val1[i]<<"\n";
                max_match2<<max_val1[i]<<"\n";
                match2_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match2_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                ALLtemp_time2<<ALLrealsec<<"\n";
              }
              else if(i==3){
                min_match3<<min_val1[i]<<"\n";
                max_match3<<max_val1[i]<<"\n";
                match3_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match3_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                ALLtemp_time3<<ALLrealsec<<"\n";
                }
              else if(i==4){
                min_match4<<min_val1[i]<<"\n";
                max_match4<<max_val1[i]<<"\n";
                match4_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match4_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                ALLtemp_time4<<ALLrealsec<<"\n";
                }
              else if(i==5){
                min_match5<<min_val1[i]<<"\n";
                max_match5<<max_val1[i]<<"\n";
                match5_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match5_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                ALLtemp_time5<<ALLrealsec<<"\n";
                }
              else if(i==6){
                min_match6<<min_val1[i]<<"\n";
                max_match6<<max_val1[i]<<"\n";
                match6_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match6_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                ALLtemp_time6<<ALLrealsec<<"\n";
                }
              else if(i==7){
                min_match7<<min_val1[i]<<"\n";
                max_match7<<max_val1[i]<<"\n";
                match7_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match7_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                ALLtemp_time7<<ALLrealsec<<"\n";
                }
              else if(i==8){
                min_match8<<min_val1[i]<<"\n";
                max_match8<<max_val1[i]<<"\n";
                match8_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                match8_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                ALLtemp_time8<<ALLrealsec<<"\n";
                }
              }

              //左側(xははみ出ない)
              else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
                std::cout << "マッチング:左側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  } 
              }
              //左下(xとyどちらもはみでる)
              else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
                std::cout << "マッチング:左下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
              }
            }
            //上側(yははみ出る)
            else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
              std::cout << "マッチング:上側(yははみ出る)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
              std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
              if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match6_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
            }
            //右側
            else if(640-cropx<Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640){
              //右上(xとyどちらもはみでる)
              if(0<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<cropy){
                std::cout << "マッチング:右上(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match6_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
              }
              //右側(xははみ出ない)
              else if(cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480-cropy){
                std::cout << "マッチング:右側(xははみ出ない)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match6_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
              }
              //右下(xとyどちらもはみでる)
              else if(480-cropy<Est_MT_pixel[i].y&&Est_MT_pixel[i].y<=480){
                std::cout << "マッチング:右下(xとyどちらもはみでる)["<<i<<"]"<<std::endl;
                cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
                cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

                MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
                MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
                cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
                MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

                //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
                MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
                MT_prev_camera[matchT_prev]=MT_prev_camera[i];
                MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
                std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
                cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
                matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match6_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
              
            }
            //下側(yははみ出ない)
            else if(cropx<=Est_MT_pixel[i].x&&Est_MT_pixel[i].x<=640-cropx&&480-cropy<=Est_MT_pixel[i].y&&Est_MT_pixel[i].y<480){
              std::cout << "マッチング:下側(yははみ出ない)["<<i<<"]"<<std::endl;
              cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
              cv::circle(img_tate, cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2), Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標

              MTPP[matchT_prev].x=Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2);//マッチング中心座標(画像全体座標)
              MTPP[matchT_prev].y=Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2);//マッチング中心座標(画像全体座標)
              cv::Rect roi_match(cv::Point(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y), cv::Size(template_size*2, template_size*2));//テンプレートの更新
              MTTP[matchT_prev] = img_src(roi_match); // 切り出し画像

              //MTTP[matchT_prev]=img_template1;//マッチングしたテンプレート画像
              MT_prev_pixel[matchT_prev]=MT_prev_pixel[i];
              MT_prev_camera[matchT_prev]=MT_prev_camera[i];
              MT_prev_Templ[matchT_prev]=MT_prev_Templ[i];
              std::cout <<"マッチングの中心座標[matchT_prev="<<matchT_prev<<"]="<<MTPP[matchT_prev]<< std::endl;
              cv::line(img_tate,cv::Point(MTPP[matchT_prev]),cv::Point(MT_prev_pixel[matchT_prev]),cv::Scalar(0,255,255), 3, cv::LINE_AA);
              matchT_prev=matchT_prev+1;//マッチングの中心座標個数
                if(i==0){
                  min_match0<<min_val1[i]<<"\n";
                  max_match0<<max_val1[i]<<"\n";
                  match0_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match0_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 0), 3);//白枠
                  ALLtemp_time0<<ALLrealsec<<"\n";
                  }
                else if(i==1){
                  min_match1<<min_val1[i]<<"\n";
                  max_match1<<max_val1[i]<<"\n";
                  match1_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match1_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  cv::rectangle(img_tate, cv::Rect(Est_MT_pixel[i].x-cropx+max_pt1[i].x, Est_MT_pixel[i].y-cropy+max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 255), 3);//白枠
                  ALLtemp_time1<<ALLrealsec<<"\n";
                  }
                else if(i==2){
                  min_match2<<min_val1[i]<<"\n";
                  max_match2<<max_val1[i]<<"\n";
                  match2_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match2_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time2<<ALLrealsec<<"\n";
                }
                else if(i==3){
                  min_match3<<min_val1[i]<<"\n";
                  max_match3<<max_val1[i]<<"\n";
                  match3_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match3_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time3<<ALLrealsec<<"\n";
                  }
                else if(i==4){
                  min_match4<<min_val1[i]<<"\n";
                  max_match4<<max_val1[i]<<"\n";
                  match4_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match4_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time4<<ALLrealsec<<"\n";
                  }
                else if(i==5){
                  min_match5<<min_val1[i]<<"\n";
                  max_match5<<max_val1[i]<<"\n";
                  match5_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match5_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time5<<ALLrealsec<<"\n";
                  }
                else if(i==6){
                  min_match6<<min_val1[i]<<"\n";
                  max_match6<<max_val1[i]<<"\n";
                  match6_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match6_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time6<<ALLrealsec<<"\n";
                  }
                else if(i==7){
                  min_match7<<min_val1[i]<<"\n";
                  max_match7<<max_val1[i]<<"\n";
                  match7_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match7_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time7<<ALLrealsec<<"\n";
                  }
                else if(i==8){
                  min_match8<<min_val1[i]<<"\n";
                  max_match8<<max_val1[i]<<"\n";
                  match8_pixel_x<<Est_MT_pixel[i].x-cropx+max_pt1[i].x+(img_template1.cols/2)<<"\n";
                  match8_pixel_y<<Est_MT_pixel[i].y-cropy+max_pt1[i].y+(img_template1.rows/2)<<"\n";
                  ALLtemp_time8<<ALLrealsec<<"\n";
                  }
              }
            }
            else{
              std::cout << "画面外["<<i<<"]"<< std::endl;
            }
          }//if(min_val1[i]<max_val1[i]*0.05)→end(テンプレートマッチング)
          else{
              std::cout << "マッチしない["<<i<<"]((0.99<max_val1["<<i<<"])="<<0.99<<"<"<<max_val1[i]<< std::endl;
          }
        }//for(int i=0;i<DMT_prev_ok;i++)→end (範囲予測+テンプレートマッチング)

        std::cout <<"matchT_prev="<<matchT_prev<< std::endl;
        temp_kosuu<<matchT_prev<<"\n";
        MTPP.resize(matchT_prev);//Depth取得可能数でリサイズ(運動復元画像座標)
        MT_prev_pixel.resize(matchT_prev);//リサイズ
        MT_prev_camera.resize(matchT_prev);//リサイズ
        MT_curr2_pixel.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心画像座標)
        MT_curr2_camera.resize(1000);//配列数初期設定(Depth取得可能なマッチング中心カメラ座標)

        //マッチングしたテンプレートをマッチテンプレートとしてキープする
        //マッチテンプレートのDepthが取得不能な点を削除
        DMT_curr2_ok=0;
        for (int i = 0; i < matchT_prev; i++) {
          DMT_curr2[i] = img_depth.at<float>(cv::Point(MTPP[i].x,MTPP[i].y));//DMP_prev=Depth_MTPP
          //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
          if(DMT_curr2[i]>0.001&&DMT_curr2[i]<10000){
            MT_curr2_Templ[DMT_curr2_ok] = MTTP[i];//Depth取得可能なマッチテンプレート
            MT_curr2_pixel[DMT_curr2_ok] = MTPP[i];//Depth取得可能なマッチング中心画像座標
            MT_curr2_camera[DMT_curr2_ok].x = -DMT_curr2[i] * ((MTPP[i].x - 320) / 615.337)/1000;//カメラ座標変換
            MT_curr2_camera[DMT_curr2_ok].y = -DMT_curr2[i] * ((MTPP[i].y - 240) / 615.458)/1000;
            MT_curr2_camera[DMT_curr2_ok].z = DMT_curr2[i]/1000;
            std::cout <<"MT_curr2_pixel["<<DMT_curr2_ok<<"]="<<MT_curr2_pixel[DMT_curr2_ok]<< std::endl;
            std::cout <<"MT_curr2_camera["<<DMT_curr2_ok<<"]="<<MT_curr2_camera[DMT_curr2_ok]<< std::endl;
            DMT_curr2_ok=DMT_curr2_ok+1;//Depthが取得可能な全マッチテンプレート数
            //DMT_prev_ok3=DMT_prev_ok3+1;//保持テンプレート数
          }
		    }

        MT_curr2_pixel.resize(DMT_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心画像座標)
        MT_curr2_camera.resize(DMT_curr2_ok);//Depth取得可能数でリサイズ(マッチング中心カメラ座標)
        std::cout <<"全テンプレート数:DMT_prev_ok="<<DMT_curr2_ok<< std::endl;
        //if(DMT_curr2_ok<=1){reset=true;}

      }//if(Tracking == true)→end
    }//if (reset == false)→end

    //ロボットの状態方程式
    //雑音の影響を考慮した実際の状態(カルマン推定前)
    if(robot_odometry.twist.twist.angular.z==0){
      Act_RobotX=Act_RobotX+(Act_RobotV*cos(Act_RobotTH)*realsec);
      Act_RobotY=Act_RobotY+(Act_RobotV*sin(Act_RobotTH)*realsec);
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }
    else{
      Act_RobotX=Act_RobotX+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(sin(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)-sin(Act_RobotTH)));
      Act_RobotY=Act_RobotY+((Act_RobotV/robot_odometry.twist.twist.angular.z)*(-cos(Act_RobotTH+robot_odometry.twist.twist.angular.z*realsec)+cos(Act_RobotTH)));
      Act_RobotTH=Act_RobotTH+(robot_odometry.twist.twist.angular.z*realsec);
    }

    std::cout << "Act_RobotX=" <<Act_RobotX<< std::endl;
    std::cout << "Act_RobotY=" <<Act_RobotY<< std::endl;
    std::cout << "Act_RobotTH=" <<Act_RobotTH<< std::endl;

    ACT_Robot= (cv::Mat_<double>(3,1) <<
      Act_RobotX,
      Act_RobotY,
      Act_RobotTH);

    if(Act_RobotX>0||Act_RobotY>0||Act_RobotTH>0){//ラグの調整
      ////目標指令状態
      if(robot_velocity.angular.z==0){
        Des_RobotX=Des_RobotX+(Des_RobotV*cos(Des_RobotTH)*realsec);
        Des_RobotY=Des_RobotY+(Des_RobotV*sin(Des_RobotTH)*realsec);
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
      else{
        Des_RobotX=Des_RobotX+((Des_RobotV/robot_velocity.angular.z)*(sin(Des_RobotTH+robot_velocity.angular.z*realsec)-sin(Des_RobotTH)));
        Des_RobotY=Des_RobotY+((Des_RobotV/robot_velocity.angular.z)*(-cos(Des_RobotTH+robot_velocity.angular.z*realsec)+cos(Des_RobotTH)));
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
    }

    std::cout << "Des_RobotX=" <<Des_RobotX<< std::endl;
    std::cout << "Des_RobotY=" <<Des_RobotY<< std::endl;
    std::cout << "Des_RobotTH=" <<Des_RobotTH<< std::endl;

    DES_Robot= (cv::Mat_<double>(3,1) <<
      Des_RobotX,
      Des_RobotY,
      Des_RobotTH);

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_graph, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_tate, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_tmp, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    //cv::imshow(win_dst, img_dst);
    //cv::imshow(win_dst2, img_dst2);
    //cv::imshow(win_fld, img_fld);
    //cv::imshow(win_line2, img_line2);
    //cv::imshow(win_line4, img_line4);
    //cv::imshow(win_graph, img_graph);
    cv::imshow(win_tate, img_tate);//縦線関連の表示用
    cv::imshow(win_tmp, img_tmp);

    //初回動作時＋検出時
    if (reset == true && kensyutu==true){
      std::cout << "(初回)検出時引き継ぎ動作" << std::endl;
      cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
      cv::swap(tate_point_curr, tate_point_prev);//縦線中点の画像座標を保存(tate_point_curr→tate_point_prev)
      cv::swap(TPCC, TPCP);//縦線中点の三次元カメラ座標を保存(TPCC→TPCP)
      cv::swap(TPCC_Templ, TPCP_Templ);//今のテンプレートを一つ前のテンプレートとして保存
      DTPP_ok=DTPC_ok;//テンプレート取得数を一つ前の取得数として保存()
      reset=false;
      kensyutu=false;
      std::cout << "(初回)reset=" <<reset<<", kensyutu="<<kensyutu<<", Tracking2="<<Tracking2<< std::endl;
    }
    //2回目動作時
    else{
      if(Tracking2==true){ 
        std::cout << "2回目引き継ぎ動作" << std::endl;
        DMT_prev_ok=DMT_curr_ok;//Depth取得可能マッチテンプレート数をキープ+(3回目以降はこれに一つ前のテンプレート要素を追加する)
        cv::swap(MT_curr_camera,MT_prev_camera);//マッチ中心カメラ座標
        cv::swap(MT_curr_pixel, MT_prev_pixel);//マッチ中心画像座標
        cv::swap(MT_curr_Templ, MT_prev_Templ);//マッチ座標
        Tracking2=false;//3回目動作
        std::cout << "(2回目)reset=" <<reset<<", kensyutu="<<kensyutu<<", Tracking2="<<Tracking2<< std::endl;

      }
      //3回目以降動作時
      else if(Tracking2 == false){
        //Depth取得可能マッチング要素キープ
        std::cout << "3回目引き継ぎ動作" << std::endl;
        DMT_prev_ok=DMT_curr2_ok;
        cv::swap(MT_curr2_camera, MT_prev_camera);//マッチ中心カメラ座標
        cv::swap(MT_curr2_pixel,  MT_prev_pixel);//マッチ中心画像座標
        cv::swap(MT_curr2_Templ,  MT_prev_Templ);//マッチ座標
        std::cout << "(3回目)reset=" <<reset<<", kensyutu="<<kensyutu<<", Tracking2="<<Tracking2<< std::endl;

      }
    }

    robot_velocity.linear.x  = 0.0; // 並進速度の初期化
    robot_velocity.angular.z = 0.0; // 回転速度の初期化
    kaisu++;
    time0=true;//一回目スキップ
    temp_time<<realsec<<"\n";
    ALLtemp_time<<ALLrealsec<<"\n";
    endTime=startTime;//動作終了時刻取得
    endTimeV1=startTimeV1;//動作終了時刻取得
    endTimeM1=startTimeM1;//動作終了時刻取得
	cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){
	ros::init(argc,argv,"FLD_clustering_3");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
    if(kaisu!=0){ros_begin = ros::Time::now();}

  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nhSub, "/robot1/odom", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub,rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

	ros::spin();//トピック更新待機
			
	return 0;
}