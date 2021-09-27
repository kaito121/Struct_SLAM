//多重インクルード防止
#ifndef INCLUDE_CONV_LRF_DATA_CLASS
#define INCLUDE_CONV_LRF_DATA_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for LRF processing on ros
#include <sensor_msgs/LaserScan.h>
//self msg
#include <obstacle_detection_2019/SensorMapData.h>
#include <cv_bridge/cv_bridge.h>
//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//クラスの定義
class convLRFDataClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
        ros::CallbackQueue queue;
        sensor_msgs::LaserScan sensorData;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        obstacle_detection_2019::SensorMapData smd;
        //マップパラメータ
        float mapW;//width[m]
        float mapH;//height[m]
        float mapR;//resolution[m]
        int mapWi;//マップサイズWidth[pixel]
        int mapHi;//マップサイズHeight[pixel]
        int hMax; //■とりあえず追加
        int wMax; //■とりあえず追加
        float sensorHigh;//LRFの高さ
        cv::Point2f posLRF;//LRFの位置(2次元マップ上)(未使用)
        //--DEM　(未完成)
        cv::Mat dem;//ステレオカメラセンサによるローカルマップ
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        convLRFDataClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~convLRFDataClass();
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
        void sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        //--送信データ作成
        void create2dMap();
        //--座標変換
        //センサ座標系ー＞マップ座標系
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //センサデータ送信
        void publishConvLRFData();//データ送信
};
#endif