//20211006 差分二乗和(SSD)を使用したテンプレートマッチングテストプログラム
//差分二乗和を使用することで誤マッチング時、マッチングを消すことができる
//またSSDは値が小さくなるほど一致度が高くなる
//テンプレート作成は特徴点検出時のみ実行


#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>//reconfig用
#include <mutex>
//#include <OpenCV1/wakuhairetu.h>//自作メッセージ用ヘッダ
#include <algorithm>//並び替え用
#include <math.h>
#include <stdlib.h>//絶対値用関数
#include <highgui.h>
#include <visualization_msgs/Marker.h>//ラインマーカー用
#include <cmath>
#include <struct_slam/MaskImageData.h>//パッケージ名要変更（自分で作ったデータを取り込んで）
#include <struct_slam/ImageMatchingData.h>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <librealsense2/rs.hpp>//Realsenseの画角調整用

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_dst = "dst";
std::string win_template1 = "template1";//テンプレート画像
std::string win_minmax1 = "minmax1";
std::string win_1 = "1";//テンプレート画像作成用
std::string win_2 = "2";//テンプレート位置表示用
std::string win_point = "point";//特徴点検出+マッチング結果表示用


using namespace std;
using namespace cv;

int best = 30;
int kaisu= 0;
int depth_point_prev_ok=0; //depth取得可能な特徴点の数
int depth_point_curr_ok=0; //depth取得可能な特徴点の数


bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool swap_on = false;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)

cv::Mat img_minmax1,img_dst;
cv::Mat img_template1;

cv::Mat img_1,img_2;
cv::Mat image_curr,image_prev,imageCopy;

vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
float depth_point_prev[600],depth_point_curr[600];

sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage;//opencvの画像
  cv::Mat depthimage;//opencvの画像

    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示

	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

  cv::Mat img_src = bridgeRGBImage->image.clone();
  cv::Mat img_dst = bridgeRGBImage->image.clone();
  cv::Mat img_depth = depthimage;
  cv::Mat imageCopy = img_src.clone();
  camera_info=*cam_info;//CameraInfo受け取り

 //特徴点----------------------------------------------------------------------------------------------------------
  //特徴点検出--------------------------------------------------------------------------------------------------------------
  //一つ目前の特徴点の数と現在の特徴点の数を合わせる必要がある。ここでは追跡可能か,それぞれDepthデータ取得可能であるかでサイズ調整を行っている
	cv::cvtColor(RGBimage, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
  //初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset == true) {
	  std::cout <<"初回検出プログラム"<< std::endl;
    depth_point_prev_ok=0; //depth取得可能な特徴点の数(初期化)
    swap_on=false;
    //cv::goodFeaturesToTrack(今画像, 今の特徴点, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 100, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);// 特徴点検出(グレースケール画像から特徴点検出)
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    camera_point_p.resize(points_prev.size());//要素数初期設定(座標変換用)
	  std::cout <<"test2_points_prev.size()="<<points_prev.size()<< std::endl;
    
		for (int i = 0; i < points_prev.size(); i++) {
			cv::circle(imageCopy, points_prev[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_prev[i] = depthimage.at<float>(cv::Point(points_prev[i].x,points_prev[i].y));
      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_prev[i]>0.001&&depth_point_prev[i]<10000){
        camera_point_p[depth_point_prev_ok].x = depth_point_prev[i] * ((points_prev[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_p[depth_point_prev_ok].y = depth_point_prev[i] * ((points_prev[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_p[depth_point_prev_ok].z = depth_point_prev[i]/1000;

        //std::cout << "特徴点のカメラ座標:camera_point_p[depth_point_prev_ok="<<depth_point_prev_ok<<"]={"<< camera_point_p[depth_point_prev_ok].x <<","<<camera_point_p[depth_point_prev_ok].y<<","<<camera_point_p[depth_point_prev_ok].z<<"}"<< std::endl;
        points_prev[depth_point_prev_ok] = points_prev[i];
        depth_point_prev_ok=depth_point_prev_ok+1;//Depth取得可能の個数をカウント
      }
		}
    points_prev.resize(depth_point_prev_ok);//Depth取得可能数でリサイズ(二次元)
    camera_point_p.resize(depth_point_prev_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)



    //テンプレート作成は特徴点検出時のみ実行(テンプレートは特徴点を中心とした16☓16の画像)
    //テンプレート画像作成
    RGBimage.copyTo(img_1);
    RGBimage.copyTo(img_2);
    cv::Rect roi1(cv::Point(100, 100), cv::Size(100, 100));
    img_template1 = img_1(roi1); // 切り出し画像
    cv::rectangle(img_2, cv::Point(100, 100),cv::Point(200, 200), cv::Scalar(255, 255, 255), 5);//テンプレート位置表示


	  std::cout <<"初回検出プログラム終了"<< std::endl;
    reset = false;//if文切り替え
  }

  //オプティカルフロー-------------------------------------------------------------------------------------
  else{
	  std::cout <<"オプティカルフロー"<< std::endl;// 特徴点追跡(二回目のフレーム)
	  vector<uchar> status;//特徴点の数
	  vector<float> err;
    swap_on = true;
    depth_point_curr_ok=0; //depth取得可能な特徴点の数(初期化)

    cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);//オプティカルフロー
    //一つ前のOPTで追跡可能だったpoints_prevと
    //cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_curr, points_prev, status, err);//オプティカルフロー(points_currは追跡可能points)
	  std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

	  // 追跡できなかった特徴点をリストから削除する
	  int i, k,n,j=0;
	  for (i = k = n =0; i < status.size(); i++){
	  	//std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
	  	//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
	  	//statusが0以外の時値を更新する(0は追跡不可能)
	  	if (status[i] != 0) {	
	  	  points_prev[k] = points_prev[i];
        camera_point_p[k] = camera_point_p[i];
	  	  points_curr[k++] = points_curr[i];
	  	}
    }
	  points_curr.resize(k);//ここでkの個数でリサイズ
	  points_prev.resize(k);
	  camera_point_p.resize(k);
	  camera_point_c.resize(k);

    for (j= n = 0; j < points_curr.size(); j++){
      //追跡距離が極端に長い追跡を削除(追跡距離が100以下の時のみ使用)
      if(abs(points_prev[j].x-points_curr[j].x)<50||abs(points_prev[j].y-points_curr[j].y)<50){
        	points_prev[n] = points_prev[j];
          camera_point_p[n] = camera_point_p[j];
	    	  points_curr[n++] = points_curr[j];
      }
    }
    points_curr.resize(n);//ここでkの個数でリサイズ
	  points_prev.resize(n);
	  camera_point_p.resize(n);
	  camera_point_c.resize(n);

    if(camera_point_p.size()<30||k<30){//特徴点が100個以下になったら再び特徴点を検出する
      reset = true;
      std::cout <<" 特徴点再検出リセット"<<std::endl;
    }

	  // 特徴点を丸で描く-------------------------------------------------------------------------------------------
	  for (int i = 0; i < points_curr.size(); i++) {
      //std::cout <<"OPT後マッチングの中心座標["<<i<<"]="<<points_curr[i]<< std::endl;
      cv::circle(imageCopy, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
	    cv::circle(imageCopy, points_curr[i], 4, Scalar(0, 255, 0), -1, cv::LINE_AA);//今の座標情報
	    //cv::line(imageCopy,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      cv::line(imageCopy,cv::Point(points_curr[i].x,points_prev[i].y),cv::Point(points_prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      cv::line(imageCopy,cv::Point(points_prev[i].x,points_curr[i].y),cv::Point(points_prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
    
      //画像→カメラ座標変換----------------------------------------------------------------------
      depth_point_curr[i] = depthimage.at<float>(cv::Point(points_curr[i].x,points_curr[i].y));
      //Depthが取得できない特徴点を削除する+Depthの外れ値を除く
      if(depth_point_curr[i]>0.001&&depth_point_curr[i]<10000){
        camera_point_c[depth_point_curr_ok].x = depth_point_curr[i] * ((points_curr[i].x - camera_info.K[2]) / camera_info.K[0])/1000;//メートル表示変換
        camera_point_c[depth_point_curr_ok].y = depth_point_curr[i] * ((points_curr[i].y - camera_info.K[5]) / camera_info.K[4])/1000;
        camera_point_c[depth_point_curr_ok].z = depth_point_curr[i]/1000;

        camera_point_p[depth_point_curr_ok]=camera_point_p[i];
        points_prev[depth_point_curr_ok] = points_prev[i];
        points_curr[depth_point_curr_ok] = points_curr[i];
        depth_point_curr_ok=depth_point_curr_ok+1;//Depth取得可能の個数をカウント
      }
    }

    cv::circle(imageCopy, points_prev[0], 6, Scalar(0,0,255), -1, cv::LINE_AA);//一つ前の画像の座標
	  cv::circle(imageCopy, points_curr[0], 6, Scalar(255, 0, 0), -1, cv::LINE_AA);//今の座標情報

    std::cout <<"depth_point_curr_ok="<<depth_point_curr_ok<< std::endl;
    camera_point_p.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    camera_point_c.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(三次元カメラ座標)
    points_prev.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)
    points_curr.resize(depth_point_curr_ok);//Depth取得可能数でリサイズ(二次元座標)

    // テンプレートマッチング
    cv::matchTemplate(img_src, img_template1, img_minmax1, cv::TM_SQDIFF);//差分二乗和

    // 最小位置に矩形描画
    cv::Point min_pt1, max_pt1;
    double min_val1, max_val1;

    cv::minMaxLoc(img_minmax1, &min_val1, &max_val1, &min_pt1, &max_pt1);//配列の最大と最小を求める関数
    std::cout << "min_val1(白)=" << min_val1 << std::endl;//一致度が上がると値が小さくなる
    std::cout << "max_val1(白)=" << max_val1 << std::endl;
    //最小値がしきい値以下なら表示
    if(min_val1<max_val1*0.01){
      cv::rectangle(imageCopy, cv::Rect(min_pt1.x, min_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 255), 10);//白
    }
  }

  // ウインドウ生成
  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_template1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_1, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_2, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_point, cv::WINDOW_AUTOSIZE);

  // 表示
  cv::imshow(win_src, img_src);
  cv::imshow(win_1, img_1);
  cv::imshow(win_2, img_2);
  cv::imshow(win_template1, img_template1);
  cv::imshow(win_point,imageCopy);

  cv::swap(image_curr, image_prev);

  if(swap_on ==true){
    cv::swap(points_curr, points_prev);//二次元画像座標を保存(points_curr→points_prev)(追跡可能点をpoints_prevに保存)
    cv::swap(camera_point_c, camera_point_p);//三次元カメラ座標を保存(camera_point_c→camera_point_p)
    cv::imshow(win_dst, img_dst);
  }


   kaisu=kaisu+1;
  cv::waitKey(1);
   //ros::spinにジャンプする
   
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);
  
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));

 
	ros::spin();//トピック更新待機
			
	return 0;
}
