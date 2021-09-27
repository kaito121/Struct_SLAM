//多重インクルード防止
#ifndef INCLUDE_VELOCITY_ESTIMATION_CLASS
#define INCLUDE_VELOCITY_ESTIMATION_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//self msg
#include <obstacle_detection_2019/ClassificationVelocityData.h>
#include <obstacle_detection_2019/ClassificationData.h>
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
#include <obstacle_detection_2019/velocityEstimationConfig.h>
//クラスの定義
class velocityEstimation{
    private:
        //受信データ
		ros::NodeHandle nhSub1;
		ros::Subscriber sub;
    	obstacle_detection_2019::ClassificationVelocityData preClstr, curClstr;//速度データ付きのクラスタデータ
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        obstacle_detection_2019::ClassificationVelocityData filtedClstr;//速度データ付きのクラスタデータ
        struct average_twist{
            int n;
            geometry_msgs::Twist twist;
        };
		struct record_twist{
            int n;
            geometry_msgs::Twist sum_twist;
            std::vector<geometry_msgs::Twist> twistArray;
        };
        //---calmanfilter
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t;
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t;
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > xh_t_1;
		std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd> > sig_xh_t_1;
		Eigen::MatrixXd sig_ut;
		Eigen::MatrixXd del_t;
		Eigen::MatrixXd sig_x0;
		Eigen::MatrixXd sig_wk;
		Eigen::MatrixXd I;
        //decision
        int trackThreshold;
        float sizeMinThreshold;        
        float sizeMaxThreshold;
        float velSigmaThreshold;
        float velMinThreshold;
        float velMaxThreshold;
        //avarage filter
        int filterN;
        std::vector<average_twist> average_twists;
        std::vector<average_twist> pre_average_twists;
        std::vector<record_twist> record_twists;
        //デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebMarker;
        int debugType;
        float timeRange, timeInteval;//表示時間範囲(~秒後まで表示),表示時間間隔(~秒ごとに表示)
        // float colors[12][3] ={{1.0,0,0},{0,1.0,0},{0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        float colors[12][3] ={{1.0,0,1.0},{1.0,1.0,0},{0,1.0,1.0},{1.0,0,0},{0,1.0,0},{0,0,1.0},{0.5,1.0,0},{0,0.5,1.0},{0.5,0,1.0},{1.0,0.5,0},{0,1.0,0.5},{1.0,0,0.5}};//色リスト
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<obstacle_detection_2019::velocityEstimationConfig> server;
        dynamic_reconfigure::Server<obstacle_detection_2019::velocityEstimationConfig>::CallbackType f;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        velocityEstimation();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~velocityEstimation();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルからの探索窓パラメータ書き込み
        //ゲット：内部パラメータの読み込み
        bool isPreCluster();
        //
        //in methods.cpp
        //その他メソッド
        //--メソッド管理
        void manage();
        //--センサーデータ受信
        void cluster_callback(const obstacle_detection_2019::ClassificationVelocityData::ConstPtr& msg);
        //--rqt_reconfigureからの読み込み
        void configCallback(obstacle_detection_2019::velocityEstimationConfig &config, uint32_t level);
        //処理
		void kalmanFilter(void);
        void decisionObstacleType();
        void averageFilter();
        void recordTwistData();
        // データ送信
        void publishData();//データ送信
        // データ更新
        void renewMessages();
        //デバッグ用のメソッド
        void debug();
        void showPointcloud();
        void showMarker();
};
#endif