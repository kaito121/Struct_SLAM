//多重インクルード防止
#ifndef INCLUDE_DARKNET_DEBUG
#define INCLUDE_DARKNET_DEBUG
#include <obstacle_detection_2019/darknetImg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


class darknetImgDebug : public darknetImg {
    private:
        ros::Publisher bbImage_pub;         //BoundingBoxesのパブリッシャ
        ros::Publisher bbMaskImage_pub;     //BoundingBoxesのパブリッシャ
        ros::Publisher pcl_pub;             //物体のポイントクラウドのパブリッシャ
        ros::Publisher depth2points_pub;
        ros::Publisher pickUpGroundPointCandidates_pub;
        ros::Publisher estimateGroundCoefficients_pub;
        ros::Publisher removeGroundPoints_pub;
        ros::Publisher trimPoints_pub;
        cv_bridge::CvImagePtr mapImageCB;   //クラスタ毎に色分けされたマップセル画像
        std::vector<int> colorMap;          //色付けを行うためのカラーマップ
        sensor_msgs::PointCloud2 pcl_msg;   //物体のポイントクラウド
        sensor_msgs::PointCloud2 depthPCL_msg;      //深度画像全て
        sensor_msgs::PointCloud2 groundCanPCL_msg;  //地面候補
        sensor_msgs::PointCloud2 groundPCL_msg;     //地面点群
        sensor_msgs::PointCloud2 obstaclePCL_msg;   //地面取り除いた後の点群
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_points; //深度画像のポイントクラウド
        int cellMargin;         //セルの余白[px]
        int cellSideLength;     //セルの辺の長さ[px]
        int mapImageRows;       //マップイメージの行
        int mapImageCols;       //マップイメージの列
        std::vector<int> num_temp;  //一時変数（要素数はマップセルの数）
        void registerPubData(); //パブリッシュするデータを登録
        std::string topic_clusterImage; //クラスタ用セルのパブリッシュトピック
        std::string topic_clusterPCL;   //クラスタ用ポイントクラウドのパブリッシュトピック
        std::string topic_gridMapImage; //グリッドマップ用ポイントクラウドのパブリッシュトピック
        std::string topic_bbImage;        //bbとdepthImageを結合した時のパブリッシュトピック
        std::string topic_bbMaskImage;         //Imageにマスクをかけた時のパブリッシュトピック
        std::string topic_depth2points;  //
        std::string topic_pickUpGroundPointCandidates;  //indices solution 
        std::string topic_estimateGroundCoefficients;   
        std::string topic_removeGroundPoints;
        std::string topic_trimPoints;
    protected:
        /**クラスタに属しているセルを探してマップ画像に色を付ける
         * cluster 入力クラスタ
         * colorIndex 色番号
         * image 出力画像
         */
        void drawClusterCells(obstacle_detection_2019::ClassificationElement& cluster, int colorIndex, cv::Mat& image);

        /**マップ画像の詳細設定を行う
         * 画像の幅、高さをcv::Matに適用
         */ 
        void setMapImageConfig();

        /**カラーマップの配列を3の倍数(BGR?)に設定
         */ 
        void setColorMap(std::vector<int>&);
        cv::Scalar getColorFromColorMap(int colorIndex);
    public:
        darknetImgDebug();
        ~darknetImgDebug();

        void setParam();
        void debug_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb,const sensor_msgs::Image::ConstPtr& image);
        void setCallback();     //コールバック関数の設定

        /**グリッドマップをポイントクラウドに変換
         * 実装予定無し
         */
        void gridmap2PointCloud();

        /**グリッドマップを画像に変換
         * セルに存在する点の数が閾値よりも大きい場合、そのセルを赤色にする
         * それ以外は黒色
         * smdml マップのレイヤ(入力)
         * pt_num_threthold 閾値
         * image 出力画像
         */
        void gridmap2Image(obstacle_detection_2019::SensorMapDataMultiLayer& smdml,int pt_num_threthold ,cv::Mat& image);

        /**
         * クラスタ情報を含むグリッドマップをポイントクラウドに変換する
         */ 
        void cluster2PointCloud(obstacle_detection_2019::ClassificationData& clusterData, sensor_msgs::PointCloud2& pc_msg);
        void cluster2Image(obstacle_detection_2019::ClassificationData& clusterData, cv::Mat& image);       //クラスタ情報を含むグリッドマップを画像に変換
        void publishDebugData();    //パブリッシュリストに存在するデータをpublishする
        /**
         * 画像にマスクかけて、出力する
         */ 
        void mask2Image(std::vector<std::vector<char>> input_mask, sensor_msgs::Image input_image);
        void addBBGroupRecursively(darknet_ros_msgs::BoundingBoxes& bbs, std::vector<bool>& checkFlag, int coreNumber, int groupNumber);
        void pickUpGroundPointCandidates();     //床面候補点の選択
        void estimateGroundCoefficients();      //床面係数abcdの算出
        void removeGroundPoints();              //床面の点を除外する
        void depth2points();
        void publishTrimMask();
};

#endif