//多重インクルード防止
#ifndef INCLUDE_CONV_PCL_TO_IMAGE_CLASS
#define INCLUDE_CONV_PCL_TO_IMAGE_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl_ros/point_cloud.h>

//クラスの定義
class convPCLDataClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
        sensor_msgs::PointCloud2 cloud2Data;
    	pcl::PointCloud<pcl::PointXYZRGB> pclData;
        cv_bridge::CvImage bridgeRgb,bridgeDepth;
        bool sensorData_subscribed_flag;
        //送信データ
		ros::NodeHandle nhPub1,nhPub2;
        ros::Publisher pubRgb,pubDepth;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        convPCLDataClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~convPCLDataClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setParam(int& temp);//(未使用)
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        //--センサーデータ受信
        void subscribeSensorData();//データ受信
        void sensor_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        //データ変換
        void convCloud2toPCL();
        void convPCLtoImages();
        //データ送信
        void publishRgbCamData();
        void publishDepthCamData();
};
#endif
