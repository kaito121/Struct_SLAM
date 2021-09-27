//多重インクルード防止
#ifndef INCLUDE_IMAGE_MATCHING_CLASS
#define INCLUDE_IMAGE_MATCHING_CLASS
//include haeders
#include <ros/ros.h>
#include <ros/callback_queue.h>
//for image processing on ros
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>//オプティカルフロー用
#include <opencv2/features2d/features2d.hpp>
// #include <opencv2/features2d.hpp>
//self msg
#include <obstacle_detection_2019/MaskImageData.h>
#include <obstacle_detection_2019/ImageMatchingData.h>
// rqt_reconfige
#include <dynamic_reconfigure/server.h>
#include <obstacle_detection_2019/imageMatchingConfig.h>
//debug
#include <pcl_ros/point_cloud.h>

//クラスの定義
class imageMatchingClass{
    private:
        //センサーデータ
		ros::NodeHandle nhSub1,nhSub2;
		ros::Subscriber subImg,subMskImg;
		ros::CallbackQueue queue1,queue2;
        cv_bridge::CvImagePtr bridgeImagePre,bridgeImageCur;
        obstacle_detection_2019::MaskImageData midPre,midCur;
        bool imgCurOnce,imgPreOnce;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pubMatch;
        obstacle_detection_2019::ImageMatchingData matchData;
        //マップパラメータ
        float mapW;//width[m]
        float mapH;//height[m]
        float mapR;//resolution[m]
        int mapWi;//マップサイズWidth[pixel]
        int mapHi;//マップサイズHeight[pixel]
        cv::Point2f posCamera;//カメラの位置(2次元マップ上)(未使用)
        //特徴点抽出
        cv::Mat grayImgPre,grayImgCur;//グレースケール
        int nh,nw;//画像分割個数
        int maxDetectPoint;//最大検出数
        int maxPoint;//最大抽出数
        cv::Mat clipImg;//分割画像Mat
        //特徴点マッチング
        std::vector< cv::Point2f > featurePointsPre;
        std::vector< cv::Point2f > featurePointsCur;
        std::vector< cv::Point2f > featurePointsTemp;
		std::vector<uchar> sts;
		std::vector<float> ers;
		int ws;//window size
        std::vector<int> tranckNumCur, tranckNumPre;
        int trackThreshold;
        //デバッグ用
		ros::NodeHandle nhDeb;
        ros::Publisher pubDeb,pubDebPcl;
        int debugType;
        //--rqt_reconfigure
        bool rqt_reconfigure;//rqt_reconfigureを使用するか
        dynamic_reconfigure::Server<obstacle_detection_2019::imageMatchingConfig> server;
        dynamic_reconfigure::Server<obstacle_detection_2019::imageMatchingConfig>::CallbackType f;

    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        imageMatchingClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~imageMatchingClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setLaunchParam();
        void configCallback(obstacle_detection_2019::imageMatchingConfig &config, uint32_t level);
        //ゲット：内部パラメータの読み込み
        int& getParam();//(未使用)
        //
        //in methods.cpp
        //その他メソッド
        void manage();//処理の流れを管理
        //--センサーデータ受信
        // void subscribeImageData();//データ受信(RGB画像)
        // void subscribeMaskImageData();//データ受信(mask画像)
        void image_callback(const sensor_msgs::ImageConstPtr& msg);//データ受信(RGB画像)
        void maskImage_callback(const obstacle_detection_2019::MaskImageData::ConstPtr& msg);//データ受信(mask画像)
        //--グレースケール化
        void cvtGray();
        //--データ確認
        bool isBridgeImageCur();//
        bool isBridgeImagePre();//
        bool isMaskImageCur();
        bool isMaskImagePre();//
        bool isFpointPre();
        bool checkPointSize();
        //データ更新
        void resetData();
        //特徴点抽出
        void getFeaturePoints();//特徴点抽出
        bool dicideAddPoints();//特徴点追加判断
        void addPreFeaturePoints();//特徴点追加
        void featureMatching();//特徴点マッチング
        void checkMatchingError();//エラーチェック
        void creatMatchingData();//
        //--座標変換
        //センサ座標系ー＞マップ座標系
        bool convertToGrid(const float& x,const float& y,int& xg,int& yg);
        //センサデータ送信
        void publishMatchingData();//データ送信
        //デバッグ用メソッド
        void debug();
        void showMatchingMap();
        void showMatchingImage();
        void cvArrow(cv::Mat* img, cv::Point2i pt1, cv::Point2i pt2, cv::Scalar color = cv::Scalar(0,200,200), int thickness=4, int lineType=8, int shift=0);
        void showGroundDeletePCL();
};
#endif