//Depth画像の色付け:https://codehero.jp/c%2B%2B/13840013/opencv-how-to-visualize-a-depth-image
//特徴点検出のテストプログラム
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <time.h>//処理の時間を出力する
#include <sys/time.h>
#include<mutex>//forEach用
#include"librealsense2/rs.hpp"//RealsenseのDepthフィルタ用ヘッダー
#include"librealsense2/h/rs_processing.h"//RealsenseのDepthフィルタ用ヘッダー
#include"librealsense2/hpp/rs_processing.hpp"//RealsenseのDepthフィルタ用ヘッダー

#include <opencv2/aruco/charuco.hpp>//マーカー検出
#include<fstream>

std::string win_src = "src";
std::string win_dst = "dst";
std::string win_dst2 = "ds2t";
std::string win_depth = "depth";
std::string win_point = "point";

using namespace std;
using namespace cv;

int ALLMarker=40;//全マーカー個数
float MC_point_prve[50][4];//一つ前のカメラ座標
float pixel[50][2],depth[100],MC_point[50][4],x,y,r2,f,ux,uy;//画像→カメラ座標変換
//double CameraLM[50][2];//カメラから見たマーカーまでの距離[0]と角度[1]


cv::Mat img_dst,img_dst2;//画像定義
vector<cv::Point2f> points_prev, points_curr;//特徴点定義
vector<cv::Point3f> camera_point_p,camera_point_c;//特徴点定義
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
int kaisu;


ros::Time ros_begin;//プログラム時間
ros::WallTime wall_begin = ros::WallTime::now();//プログラム開始時の時間
ros::WallDuration wall_prev;//一つ前の時間
ros::WallDuration wall_systemtime;//サンプリング時間
//ros::Time ros_begin = ros::Time::now();
struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)

//Depthフィルター初期化
//rs2::decimation_filter dec_filter;
//dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
//rs2::disparity_transform depth_to_disparity(true);
//rs2::disparity_transform disparity_to_depth(false);
//rs2::spatial_filter spat_filter;
//spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
//spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
//spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
//spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
//rs2::temporal_filter temp_filter;
//temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
//temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
//temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
//rs2::hole_filling_filter hf_filter;
//hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

ofstream depth_marker("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/depth_marker.txt");
ofstream ALLdepth_marker("/home/fuji/catkin_ws/src/Struct_SLAM/src/robot/date/ALLdepth_marker.txt");


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
  ROS_INFO("callback_functionが呼ばれたよ");

  //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
  gettimeofday(&startTime, NULL);// 開始時刻取得
  if(kaisu!=0){
    time_t diffsec = difftime(startTime.tv_sec, endTime.tv_sec);    // 秒数の差分を計算
    suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
    realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
    printf("処理の時間=%f\n", realsec);
  }

//サンプリング時間取得(ROS)
  ros::WallTime wall_now = ros::WallTime::now();
  ros::WallDuration wall_duration = wall_now - wall_begin;
  ROS_INFO("WALL:%u.%09u", wall_duration.sec, wall_duration.nsec);
  wall_systemtime = wall_duration - wall_prev;
  //ROS_INFO("systemtime:%u.%09u", wall_systemtime.sec, wall_systemtime.nsec);
std::cout << "wall_systemtime=" <<wall_systemtime<< std::endl;//サンプリング時間

    //変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,img_depth,image;
//
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",rgb_msg->encoding.c_str());
        return ;
    }
    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
	  //エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",depth_msg->encoding.c_str());
        return ;}
//  
  	camera_info=*cam_info;//CameraInfo受け取り
   
  	image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  	img_depth = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

	img_dst = image.clone();
    img_dst = cv::Scalar(255,255,255);
    image.copyTo(img_dst2);

   
    


    //cv::Mat map = cv::imread("image", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);

    //double min;
    //double max;
    //cv::minMaxIdx(img_depth, &min, &max);
    //cv::Mat adjimg_depth;
    //// expand your range to 0..255. Similar to histEq();
    //img_depth.convertTo(adjimg_depth,CV_8UC1, 255 / (max-min), -min); 
//
    //// this is great. It converts your grayscale image into a tone-mapped one, 
    //// much more pleasing for the eye
    //    // function is found in contrib module, so include contrib.hpp 
    //// and link accordingly
    //cv::Mat falseColorsimg_depth;
    //applyColorMap(adjimg_depth, falseColorsimg_depth, cv::COLORMAP_AUTUMN);
//
    ////グレースケール化
    //cvtColor(adjimg_depth,adjimg_depth,COLOR_BGR2GRAY);
//
    //cv::imshow("Out", falseColorsimg_depth);
    
    //Depth画像の色付けを行なっている
    double min;
    double max;
    cv::minMaxIdx(img_depth, &min, &max);
    cv::Mat adjimg_depth;
    cv::Mat adjimg_depth2;
    // Histogram Equalization
    float scale = 255 / (max-min);
    float scale2 = 500 / (max-min);
    img_depth.convertTo(adjimg_depth,CV_8UC1, scale, -min*scale); 
    img_depth.convertTo(adjimg_depth2,CV_8UC1, scale2, -min*scale2); 
    cv::Mat falseColorsimg_depth;
    //cv::imshow("adjimg_depth", adjimg_depth);
    //cv::imshow("adjimg_depth500", adjimg_depth2);

    applyColorMap(adjimg_depth2, falseColorsimg_depth, cv::COLORMAP_WINTER);//ここのcvで色を変えられる
    
    cv::imshow("Out", falseColorsimg_depth);






    int W=15,H=15;
    std::vector<std::vector<double>> T(image.size().width, std::vector<double>(image.size().height));
    //double T[image.size().width-W][image.size().height-H];

    for(int x = 0;x < image.size().width-W;x=x+W){
        for(int y = 0;y < image.size().height-H;y=y+H){
           //Mat cut_img(img_dst7,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            //Mat cut_img(image,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            Mat cutdepth_img(img_depth,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            int siro=0,kaisu=0,P1=0,zero=0;
            std::mutex mut1;
            std::cout<<"test2"<<std::endl;
            //cut_img.forEach<unsigned char>([&](unsigned char &p, const int  position[]) -> void{
            //    std::lock_guard<std::mutex> lock(mut1); 
            //    if(p>=200){siro=siro+1;} });//白を検出(200以上なら白)
            double Dmax=-100,Dmin=1000,depth=0;//初期値代入のため
            std::mutex mut2; 
            cutdepth_img.forEach<float>([&](float &p, const int  position[]) -> void{
                std::lock_guard<std::mutex> lock(mut2);//ここ
                kaisu=kaisu+1;P1=p/1000;
                if(Dmax<=(p/1000)){Dmax=p/1000;}
                if(Dmin>=(p/1000)){Dmin=p/1000;}
                if((p/1000)<=0.2){zero=zero+1;}
                depth=depth+p/1000;//ピクセル内の距離の合計
                if(W*H-zero<=2){}
                else{
                if(kaisu==W*H){T[x][y]=(depth-(Dmax+Dmin))/(W*H-2-zero);}
                }//最大値と最小値を除いて平均化
                //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<P1<< std::endl;//printf
                //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<cutdepth_img.at<float>(position[0],position[1])<< std::endl;//printf
             });
             //A[x][y]=(siro/(W*H))*100;//ピクセル内の白の割合
            //std::cout <<"T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;//printf

            /*if(T[x][y]*100<255){
  	            cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(T[x][y]*80, 10, 10), 2);
                std::cout <<"第一範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255<T[x][y]*100&&T[x][y]*100<255*2){
  	            cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(T[x][y]*80, T[x][y]*40, 10), 2);
  	            //cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(255,T[x][y]*40,0), 2);
                std::cout <<"第二範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255*2<T[x][y]*100&&T[x][y]*100<255*3){
  	            cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(T[x][y]*80, T[x][y]*40, T[x][y]*20), 2);
  	            //cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(255,255,T[x][y]*20), 2);
                std::cout <<"第三範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255*3<T[x][y]*100){
  	            cv::circle(img_dst, cv::Point(x,y), 3, cv::Scalar(0,0,T[x][y]*40), 2);
                std::cout <<"範囲オーバー:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}*/

            if(T[x][y]*100<255){
  	            cv::circle(img_dst, cv::Point(x,y),W/2, cv::Scalar(T[x][y]*80, 10, 10), 2);
  	            cv::circle(img_dst2, cv::Point(x,y), W/2, cv::Scalar(T[x][y]*80, 10, 10), 1.7);
                std::cout <<"第一範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255<T[x][y]*100&&T[x][y]*100<255*2){
  	            cv::circle(img_dst, cv::Point(x,y), W/2, cv::Scalar(255,T[x][y]*40,0), 2);
  	            cv::circle(img_dst2, cv::Point(x,y), W/2, cv::Scalar(255,T[x][y]*40,0), 1.7);
                std::cout <<"第二範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255*2<T[x][y]*100&&T[x][y]*100<255*3){
  	            cv::circle(img_dst, cv::Point(x,y), W/2, cv::Scalar(255,255,T[x][y]*20), 2);
  	            cv::circle(img_dst2, cv::Point(x,y), W/2, cv::Scalar(255,255,T[x][y]*20), 1.7);
                std::cout <<"第三範囲:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}
            if(255*3<T[x][y]*100){
  	            cv::circle(img_dst, cv::Point(x,y), W/2, cv::Scalar(0,0,T[x][y]*40), 2);
  	            cv::circle(img_dst2, cv::Point(x,y), W/2, cv::Scalar(0,0,T[x][y]*40), 1.7);
                std::cout <<"範囲オーバー:T["<<x<<"]["<<y<<"]="<<T[x][y]<< std::endl;}


        }
    }

      //マーカー検出+外部パラメータ推定-------------------------------------------------------------------------------------------  
  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);
  fs.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  intrinsic_K=cameraMatrix;

  //カメラの歪みパラメータ読み込み
  cv::Mat distCoeffs;
  cv::FileStorage fd;
  fd.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fd["distortion"]>>distCoeffs;

  //マーカ辞書作成 6x6マスのマーカを250種類生成
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();//マーカー検出時メソッドを指定
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

  //マーカー検出
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners;
  cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

  std::vector<cv::Vec3d> rvecs,tvecs;//マーカーの姿勢(回転ベクトル、並進ベクトル)
  cv::Mat_<double> rvecs2[50],jacobian[50];
  int MarkerC[50][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
  float MCx[50],MCy[50];//マーカーの中心座標
  float depth0,depthFT;//Depth修正用


  //マーカー観測可能
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(img_dst2, markerCorners, markerIds);//マーカー位置を描画
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);//マーカーの姿勢推定
    for(int i=0;i<markerIds.size();i++){
      std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      //std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化(コーナー時計回り)
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;
      MCx[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][0]+MarkerC[markerIds.at(i)][1][0])/2;//マーカー中心座標(x座標)
      MCy[markerIds.at(i)]=(MarkerC[markerIds.at(i)][0][1]+MarkerC[markerIds.at(i)][2][1])/2;//マーカー中心座標(y座標)
	  cv::circle(img_dst2, cv::Point(MCx[markerIds.at(i)],MCy[markerIds.at(i)]), 3, Scalar(0,255,0),  -1, cv::LINE_AA);//緑点
      cv::aruco::drawAxis(img_dst2,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写
      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

	  cv::circle(img_dst2, cv::Point(MarkerC[markerIds.at(i)][0][0],MarkerC[markerIds.at(i)][0][1]), 5, Scalar(255,255,0),  -1, cv::LINE_AA);//緑点
	  cv::circle(img_dst2, cv::Point(MarkerC[markerIds.at(i)][1][0],MarkerC[markerIds.at(i)][1][1]), 5, Scalar(0,255,255),  -1, cv::LINE_AA);//緑点
	  cv::circle(img_dst2, cv::Point(MarkerC[markerIds.at(i)][3][0],MarkerC[markerIds.at(i)][3][1]), 5, Scalar(255,0,255),  -1, cv::LINE_AA);//緑点

	  cv::circle(img_dst2, cv::Point(MarkerC[markerIds.at(i)][2][0],MarkerC[markerIds.at(i)][2][1]), 5, Scalar(255,0,255),  -1, cv::LINE_AA);//緑点


      //画像→カメラ座標変換(マーカーの中心座標を使用)----------------------------------------------------------------------
      pixel[markerIds.at(i)][0]=MCx[markerIds.at(i)];
      pixel[markerIds.at(i)][1]=MCy[markerIds.at(i)];
      std::cout <<"MCx["<<markerIds.at(i)<<"]="<<MCx[markerIds.at(i)]<<",MCy["<<markerIds.at(i)<<"]="<<MCy[markerIds.at(i)]<< std::endl;
      depth[markerIds.at(i)] = img_depth.at<float>(cv::Point(pixel[markerIds.at(i)][0],pixel[markerIds.at(i)][1]));
      //Depth値を修正
      //depth0=depth[markerIds.at(i)]*0.001;
      //depthFT=39.215*depth0*depth0-27.793*depth0-7.7718;
      //depth[markerIds.at(i)]=depth[markerIds.at(i)]-depthFT;
      std::cout <<"image.cols="<<image.cols<<",image.rows="<<image.rows<< std::endl;
      std::cout <<"img_depth.cols="<<img_depth.cols<<",img_depth.rows="<<img_depth.rows<< std::endl;


      //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
      if(depth[markerIds.at(i)]>0&&depth[markerIds.at(i)]<10000){
        x = (pixel[markerIds.at(i)][0] - image.cols/2) / 615.337;//ここで正規化座標もしてる
        y = (pixel[markerIds.at(i)][1] - image.rows/2) / 615.458;
        //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

        MC_point[markerIds.at(i)][0] = depth[markerIds.at(i)] * x/1000;//メートル表示変換
        MC_point[markerIds.at(i)][1] = depth[markerIds.at(i)] * y/1000+0.3;
        MC_point[markerIds.at(i)][2] = depth[markerIds.at(i)]/1000;
        MC_point[markerIds.at(i)][3] = 1;//データ取得可能なら1

        //ロボット座標系(tf座標系)とカメラ観測座標系ではxとyの符号が逆なので注意(ロボット(tf)座標:Y = カメラ観測座標:-X,ロボット(tf)座標:Z = カメラ観測座標:-Y )
        std::cout << "マーカーのカメラ座標:MC_point["<<markerIds.at(i)<<"]={"<< -MC_point[markerIds.at(i)][0] <<","<<-MC_point[markerIds.at(i)][1]<<","<<MC_point[markerIds.at(i)][2]<<"}"<< std::endl;
        //std::cout <<"X="<< MC_point[markerIds.at(i)][0]/MC_point[markerIds.at(i)][2]<< std::endl;
        //std::cout <<"Y="<< MC_point[markerIds.at(i)][1]/MC_point[markerIds.at(i)][2]<< std::endl;
      }
    }
    double ALLdepthM=0;
    int Mko=0;
    //マーカーのDepth値を調べる
    for(int i=MarkerC[4][0][0];i<=MarkerC[4][1][0];i++){
       for(int j=MarkerC[4][0][1];j<=MarkerC[4][3][1];j++){
            float depthM= img_depth.at<float>(cv::Point(i,j));
            ALLdepthM=ALLdepthM+depthM;
            depth_marker<<"depthM("<<i<<","<<j<<")="<<depthM<<"\n";
	        cv::circle(img_dst2, cv::Point(i,j), 1, Scalar(0,0,255),  -1, cv::LINE_AA);//緑点
	        cv::circle(adjimg_depth, cv::Point(i,j), 1, Scalar(0,0,0),  -1, cv::LINE_AA);//緑点
            Mko=Mko+1;
       }
    }
    ALLdepth_marker<<"ALLdepthM="<<ALLdepthM/Mko<<"\n";

  }
  else{std::cout <<"マーカー観測不能"<< std::endl;}
  //ここまでがマーカー観測----------------------------------------------




  
    // 画面表示
    cv::imshow(win_src, image);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_depth, img_depth);
    cv::imshow("adjimg_depth", adjimg_depth);
    cv::imshow("adjimg_depth500", adjimg_depth2);
	kaisu++;

    cv::waitKey(1);//ros::spinにジャンプする
}



//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker2");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  if(kaisu!=0){ros_begin = ros::Time::now();}
	//subscriber関連
  //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));
  
	ros::spin();//トピック更新待機
			
	return 0;
}