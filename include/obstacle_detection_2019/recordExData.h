//多重インクルード防止
#ifndef INCLUDE_RECORD_EXPERIMENT_CLASS
#define INCLUDE_RECORD_EXPERIMENT_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//subscribe msg
#include <nav_msgs/Odometry.h>
#include <obstacle_detection_2019/ClassificationVelocityData.h>
// publish msg
#include <obstacle_detection_2019/recordData.h>
//追加（デバッグ用）
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/MarkerArray.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection_2019/recordExDataConfig.h>
//ファイル出力
#include<fstream>
//クラスの定義
class recordExData{
    private:
        //受信データ
		ros::NodeHandle nhSub;
		ros::Subscriber sub1,sub2;
    	obstacle_detection_2019::ClassificationVelocityData velClstr,velClstr_global;//速度データ付きのクラスタデータ(推定前)
        //送信データ（データ解析用のトピックを配信）
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        obstacle_detection_2019::recordData recordData; 
        bool outputCSV;//csv出力するか  
        std::string userpath,filepath,filename;
        //データ処理用
        ros::Time timeNow;
        int robotNum;
        nav_msgs::Odometry init_turtle_odom, init_beego_odom;
        nav_msgs::Odometry measure_odom, estimate_odom;//measure_odom=/robot1/odom, estimate_odom=estimateData
        nav_msgs::Odometry turtle_odom, beego_odom;
        double init_turtle_yaw, init_beego_yaw;
        double distanceThreshold;
        int targetNum;//移動障害物（タートルボット）検出番号
        //デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebMarker;
        int debugType;
        // float colors[12][3] ={{1.0,0,0},{0,1.0,0},{0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<obstacle_detection_2019::recordExDataConfig> server;
        dynamic_reconfigure::Server<obstacle_detection_2019::recordExDataConfig>::CallbackType f;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        recordExData();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~recordExData();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルからのパラメータ書き込み
        //--rqt_reconfigureからの読み込み
        void configCallback(obstacle_detection_2019::recordExDataConfig &config, uint32_t level);
        //ゲット：内部パラメータの読み込み
        // bool getParam();
        //
        //in methods.cpp
        //その他メソッド
        //--メソッド管理
        void manage();
        //--データ受信コールバック
        void velCluster_callback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg);
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
        //処理
        void transObstaclePos();
        void targetDetect();
        void targetDetect_movingRobot();
        void transEstimationPos();
        void setEstimateData();
		void setRecordData();
        // データ送信
        void publishRecordData();//データ送信
        void setCSVDataLabel();//csvラベル出力
        void outputCSVData();//csv出力
        // データ更新
        void renewMessages();
        //データ変換
        tf::Vector3 transPointToVector3(geometry_msgs::Point& pt);//transform geometry_msgs::Point to tf::Vector3
        void transVector3ToPoint(const tf::Vector3& pt, geometry_msgs::Point& bt);

        //デバッグ用のメソッド
        void debug();
        void showMarker();
};
#endif

