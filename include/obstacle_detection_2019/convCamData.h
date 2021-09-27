//多重インクルード防止
#ifndef INCLUDE_CONV_CAM_DATA_CLASS
#define INCLUDE_CONV_CAM_DATA_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//self msg
#include <obstacle_detection_2019/SensorMapData.h>
#include <obstacle_detection_2019/MaskImageData.h>
//床面推定
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection_2019/convCamDataConfig.h>
//クラスの定義
class convCamDataClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
        cv_bridge::CvImagePtr bridgeImage;
        //送信データ
		ros::NodeHandle nhPub1,nhPub2;
        ros::Publisher pubConv,pubMask;
        //add by sta
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointIndices::Ptr inliers;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        //床面除去パラメータ
        float f;//焦点距離、画像パラメータ
        //--RANSAC
        int ransacNum;
        float distanceThreshold;
        float epsAngle;
        //--床面
        float a,b,c,d;
        float groundCandidateY;
        float camHeight;
        float ground_th;
        float height_th;
        //マップパラメータ
        float mapW;//width[m]
        float mapH;//height[m]
        float mapR;//resolution[m]
        int mapWi;//マップサイズWidth[pixel]
        int mapHi;//マップサイズHeight[pixel]
        cv::Point2f posCamera;//カメラの位置(2次元マップ上)(未使用)
        //--DEM　(未完成)
        //cv::Mat dem;//ステレオカメラセンサによるローカルマップ
        //--rqt_reconfigure
        //bool rqt_reconfigure;//rqt_reconfigureを使用するか
        //dynamic_reconfigure::Server<obstacle_detection_2019::convCamDataConfig> server;
        //dynamic_reconfigure::Server<obstacle_detection_2019::convCamDataConfig>::CallbackType fc;
    //protected:
        //obstacle_detection_2019::SensorMapData smd;
        //obstacle_detection_2019::MaskImageData mid;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        convCamDataClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~convCamDataClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルから書き込み
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        //--センサーデータ受信
        //bool subscribeSensorData();//データ受信
        //void sensor_callback(const sensor_msgs::ImageConstPtr& msg);
        //--rqt_reconfigureからの読み込み
        //void configCallback(obstacle_detection_2019::convCamDataConfig &config, uint32_t level);
        //--manage
        void manage();
        //--床面推定、床面除去
        //----RANSAC
        void groundEstimationRANSAC();
        void createPubDataRANSAC();//2次元ローカルマップ作成, マスク画像作成
        //----DEM　(未完成)
        //void createDEM();//DEM作成
        //--座標変換
        //センサ座標系ー＞マップ座標系
        //bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //センサデータ送信
        //void publishConvCamData();//データ送信
        //マスク画像送信
        //void publishMaskImage();//データ送信
        //データクリア
        void clearMessages();
};
#endif