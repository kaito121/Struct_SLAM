//多重インクルード防止
#ifndef INCLUDE_SYNCRO_IMAGE_CLASS
#define INCLUDE_SYNCRO_IMAGE_CLASS
//include haeders
#include <ros/ros.h>
//
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
//
#include<obstacle_detection_2019/synchronizedImage.h>
//debug
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


//クラスの定義
class syncroImageClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
        sensor_msgs::Image rgbImage,depthImage;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        obstacle_detection_2019::synchronizedImage syncImg;
        //
        message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
        message_filters::Subscriber<sensor_msgs::Image> depth_sub;
        //
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;
        //debug
		ros::NodeHandle nhDeb;
        ros::Publisher pubDebPcl;
        float f = 701.010;
        int debugType;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        syncroImageClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~syncroImageClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        void conbineImage();
        //--センサーデータ受信
        void callback(const sensor_msgs::Image::ConstPtr& rgbMsg,const sensor_msgs::Image::ConstPtr& depthMsg);
        //データ送信
        void publishRgbCamData();
        //
        void debug();
        void showSynchroPCL();
};
#endif
