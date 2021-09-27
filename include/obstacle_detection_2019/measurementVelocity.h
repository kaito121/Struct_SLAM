//多重インクルード防止
#ifndef INCLUDE_MEASUREMENT_VELOCITY_CLASS
#define INCLUDE_MEASUREMENT_VELOCITY_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//self msg
#include <obstacle_detection_2019/SensorMapData.h>
#include <obstacle_detection_2019/ClassificationVelocityData.h>
#include <obstacle_detection_2019/ClassificationData.h>
#include <obstacle_detection_2019/ImageMatchingData.h>
//デバッグ用
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/MarkerArray.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection_2019/measurementVelocityConfig.h>

//クラスの定義
class measurementVelocity{
    private:
        //受信データ
		ros::NodeHandle nhSub1,nhSub2;
		ros::Subscriber subCluster,subMatch;
		ros::CallbackQueue queue1,queue2;
        obstacle_detection_2019::ClassificationData curClstr,prvClstr;
        obstacle_detection_2019::ImageMatchingData matchData;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
    	obstacle_detection_2019::ClassificationVelocityData cvd;//速度データ付きのクラスタデータ
        //クラスタマップ
    	std::vector<int> prvClstrMap, curClstrMap;
    	//マッチング評価
	    std::vector<int> matchResult;
        float weightImage, weightSize, weightGravity;//マッチング重み
        int trackThreshold;//追跡回数閾値
        //デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl,pubDebMarker;
        std::vector<int> trackNum;
        int debugType;
        float timeRange, timeInteval;//表示時間範囲(~秒後まで表示),表示時間間隔(~秒ごとに表示)
        // float colors[12][3] ={{255,0,0},{0,255,0},{0,0,255},{255,255,0},{0,255,255},{255,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
        float colors[12][3] ={{255,0,255},{255,255,0},{0,255,255},{255,0,0},{0,255,0},{0,0,255},{127,255,0},{0,127,255},{127,0,255},{255,127,0},{0,255,127},{255,0,127}};//色リスト
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<obstacle_detection_2019::measurementVelocityConfig> server;
        dynamic_reconfigure::Server<obstacle_detection_2019::measurementVelocityConfig>::CallbackType f;
        //
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        measurementVelocity();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~measurementVelocity();
        //
        //メソッド:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        void setLaunchParam();
        void configCallback(obstacle_detection_2019::measurementVelocityConfig &config, uint32_t level);
        //セット：内部パラメータの書き込み
        void setParam(int& temp);//(未使用)
        //ゲット：内部パラメータの読み込み
        //
        //in methods.cpp
        void manage();//処理の流れを管理
        //その他メソッド
        //--センサーデータ受信
        // void subscribeClusterData();//データ受信
        void cluster_callback(const obstacle_detection_2019::ClassificationData::ConstPtr& msg);
        // void subscribeImageMatchingData();//データ受信
        void matching_callback(const obstacle_detection_2019::ImageMatchingData::ConstPtr& msg);
        //--マッチング
        bool isPrvClstr();//連続データがあるか確認
        bool isCurClstr();//curClstrのデータの有無
        bool isMatchData();//matchDataのデータの有無
        void creatClstrMap();//クラスタマップ作製
        void matchingClstr();//マッチング処理（途中）
        void measurementProcess();//計測処理
        // クラスタデータ送信
        void publishClassificationData();//データ送信
        //データ更新
        void renewClstrData();
        //デバッグ用のメソッド
        void debug();
        void showPointcloud();
        void showMarker();
};
#endif