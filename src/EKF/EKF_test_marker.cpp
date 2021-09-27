//rosのヘッダ
//カルマンフィルタはマーカーの位置を推定する
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/CameraInfo.h>//camera_infoを獲得するためのヘッダー
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <random>
#include <functional>
#include <opencv2/aruco/charuco.hpp>//マーカー検出


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_dst = "dst";//カメラ画像


using namespace std;
using namespace cv;
float Ts=1,d=2;
float kaisu;

//マーカー検出
sensor_msgs::CameraInfo camera_info;//CameraInfo受け取り用
float pixel[50][4][2],depth[100],point[50][4][3],x,y,r2,f,ux,uy;//画像→カメラ座標変換
cv::Mat_<float> intrinsic_K= cv::Mat_<float>(3, 3);//カメラ内部パラメータ保管
float marker_prev[50][4][3];

float omegaY,Vx,Vz;
float xEst[3],xEst_prev[3];
cv::Mat Ft,Gt,Vt,xEST,xEST_prev,Mt,Pt,Wt,Ot,Yt,Ht,Ut,Zt,Kt,u_;
cv::Mat delta,Ot_1,Ot_1_T;

cv::Mat img_dst,imageCopy;
 

//コールバック関数

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat depthimage;//opencvの画像
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
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

    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
    image = bridgeImage->image.clone();//image変数に変換した画像データを代入
    camera_info=*cam_info;//CameraInfo受け取り
    //camera_info.K[0]=615.337,camera_info.K[2]=324.473,camera_info.K[4]=615.458,camera_info.K[5]=241.696//内部パラメータ

    
    image.copyTo(imageCopy);//
    if(kaisu==0){
    img_dst = image.clone();//オプティカルフローの描写画面の用意
    img_dst = cv::Scalar(255,255,255);
  }

  //マーカー検出+外部パラメータ推定-------------------------------------------------------------------------------------------  
  //カメラ内部パラメータ読み込み
  cv::Mat cameraMatrix;
  cv::FileStorage fs;
  fs.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fs["intrinsic"]>>cameraMatrix;
  std::cout << "内部パラメータcameraMatrix=\n" << cameraMatrix << std::endl;
  intrinsic_K=cameraMatrix;

  //カメラの歪みパラメータ読み込み
  cv::Mat distCoeffs;
  cv::FileStorage fd;
  fd.open("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/realsense_para.xml", cv::FileStorage::READ);
  fd["distortion"]>>distCoeffs;
  //std::cout << "ねじれパラメータdistCoeffs=\n" << distCoeffs << std::endl;

  //マーカ辞書作成 6x6マスのマーカを250種類生成
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);

  //マーカー検出時メソッドを指定
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

  //マーカー検出
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f> > markerCorners;
  cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);

  std::vector<cv::Vec3d> rvecs,tvecs;//マーカーの姿勢(回転ベクトル、並進ベクトル)
  cv::Mat_<double> rvecs2[50],jacobian[50];
  int MarkerC[50][4][2];//マーカーのコーナー座標を記録する([マーカーID][コーナー番号][XorY])
  float MCx[50],MCy[50];//マーカーの中心座標
  cv::Mat_<float> Screenp[50] = cv::Mat_<float>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<float> Camerap[50] = cv::Mat_<float>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T
  float MLength[50],MAngle[50];//マーカーまでの距離と角度
  int depth_error[50];//各マーカーごとにDepthが取得不可能なコーナー数をカウント
  int depth_ok[50],ALL_depth_ok=0;//各マーカーごとにDepthが取得可能なコーナー数をカウント
  float World_point[50][4][3];//各マーカーの世界座標

  
  if (markerIds.size() > 0) {
    //マーカー位置を描画
    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    //マーカーの姿勢推定
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);

    for(int i=0;i<markerIds.size();i++){
      std::cout <<"マーカーの個数:markerIds.size()="<<markerIds.size() << std::endl;//マーカー個数
      std::cout <<"markerIds("<<i<<")="<< markerIds.at(i) << std::endl;//マーカーID
      MarkerC[markerIds.at(i)][0][0]=markerCorners[i][0].x, MarkerC[markerIds.at(i)][0][1]=markerCorners[i][0].y;//コーナーの画像座標ID対応化
      MarkerC[markerIds.at(i)][1][0]=markerCorners[i][1].x, MarkerC[markerIds.at(i)][1][1]=markerCorners[i][1].y;
      MarkerC[markerIds.at(i)][2][0]=markerCorners[i][2].x, MarkerC[markerIds.at(i)][2][1]=markerCorners[i][2].y;
      MarkerC[markerIds.at(i)][3][0]=markerCorners[i][3].x, MarkerC[markerIds.at(i)][3][1]=markerCorners[i][3].y;

		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][0][0], MarkerC[markerIds.at(i)][0][1]), 3, Scalar(0,255,0),   -1, cv::LINE_AA);//緑点
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][1][0], MarkerC[markerIds.at(i)][1][1]), 3, Scalar(255,255,0), -1, cv::LINE_AA);//水色
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][2][0], MarkerC[markerIds.at(i)][2][1]), 3, Scalar(255,0,0),   -1, cv::LINE_AA);//青
		  cv::circle(imageCopy, cv::Point(MarkerC[markerIds.at(i)][3][0], MarkerC[markerIds.at(i)][3][1]), 3, Scalar(255,0,255), -1, cv::LINE_AA);//紫

      cv::aruco::drawAxis(imageCopy,cameraMatrix,distCoeffs,rvecs[i], tvecs[i], 0.1);//マーカーの姿勢描写

      cv::Rodrigues(rvecs[i],rvecs2[i],jacobian[i]);//回転ベクトルから回転行列への変換

      depth_ok[markerIds.at(i)]=0;//Depth取得可能数初期化

      //画像→カメラ座標変換(コーナすべて変換する)----------------------------------------------------------------------
      for(int j=0;j<4;j++){
        pixel[markerIds.at(i)][j][0]=MarkerC[markerIds.at(i)][j][0];
        pixel[markerIds.at(i)][j][1]=MarkerC[markerIds.at(i)][j][1];
        std::cout <<"MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][0]="<<MarkerC[markerIds.at(i)][j][0]<<",MarkerC[markerIds.at(i)="<<markerIds.at(i)<<"][j="<<j<<"][1]="<<MarkerC[markerIds.at(i)][j][1]<< std::endl;
        depth[j] = depthimage.at<float>(cv::Point(pixel[markerIds.at(i)][j][0],pixel[markerIds.at(i)][j][1]));

        //Depthが取得できないコーナーを削除する+Depthの外れ値を除く
        if(depth[j]>0&&depth[j]<10000){
          x = (pixel[markerIds.at(i)][j][0] - camera_info.K[2]) / camera_info.K[0];
          y = (pixel[markerIds.at(i)][j][1] - camera_info.K[5]) / camera_info.K[4];

          point[markerIds.at(i)][j][0] = depth[j] * x/1000;//メートル表示変換
          point[markerIds.at(i)][j][1] = depth[j] * y/1000;
          point[markerIds.at(i)][j][2] = depth[j]/1000;
          std::cout << "特徴点のカメラ座標:point["<<markerIds.at(i)<<"]["<<j<<"]={"<< point[markerIds.at(i)][j][0] <<","<<point[markerIds.at(i)][j][1]<<","<<point[markerIds.at(i)][j][2]<<"}"<< std::endl;
          depth_ok[markerIds.at(i)]=depth_ok[markerIds.at(i)]+1;//Depth取得可能の個数をカウント
          ALL_depth_ok=ALL_depth_ok+1;//全Depth取得可能の個数をカウント
        }
        if(kaisu==0){
          marker_prev[markerIds.at(i)][j][0]=0;
          marker_prev[markerIds.at(i)][j][1]=0;
          marker_prev[markerIds.at(i)][j][2]=0;
        }
      }
      std::cout <<"depth_ok["<<markerIds.at(i)<<"]="<< depth_ok[markerIds.at(i)]<< std::endl;
    }
  }

  //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
  if(kaisu==0){
    xEST = cv::Mat_<float>::zeros(3, 1);//推定状態
    xEST_prev = cv::Mat_<float>::zeros(3, 1);//推定状態
    xEst[0]=0,xEst[1]=0,xEst[2]=0;//マーカーの推定初期値
    xEst_prev[0]=0,xEst_prev[1]=0,xEst_prev[2]=0;//マーカーの推定初期値
    omegaY=0;//カメラの角速度
    Vx=0,Vz=0;//カメラの速度
    Ft = cv::Mat_<float>::zeros(3, 3);//Ftヤコビアン行列
    delta = cv::Mat_<float>::zeros(2, 2);//共分散行列
    delta = delta*0.12;//数字は仮
    Ot = cv::Mat_<float>::zeros(2, 6);
    Ot_1 = cv::Mat_<float>::zeros(4, 4);
    Ot_1_T = cv::Mat_<float>::zeros(4, 4);
    delta = cv::Mat_<float>::eye(2, 2);	  // 単位行列
    Wt = cv::Mat_<float>::zeros(6, 6);
    Mt = cv::Mat_<float>::zeros(3, 3);//誤差共分散(予測式)
    Pt = cv::Mat_<float>::zeros(3, 3);//誤差共分散(更新式)
    Yt = cv::Mat_<float>::zeros(2, 1);//観測残差
    Ht = cv::Mat_<float>::zeros(2, 3);//Htヤコビアン行列
    Ut = cv::Mat_<float>::zeros(2, 1);//共分散
    Kt = cv::Mat_<float>::zeros(3, 2);//カルマンゲインKt
    u_ = (cv::Mat_<float>(2, 2) <<
      1/12, 0,
      0, 1/12);


    std::cout <<"初期設定"<< std::endl;
    //marker_prev[markerIds.at(0)][2][0]=0;
    //marker_prev[markerIds.at(0)][2][1]=0;
    //marker_prev[markerIds.at(0)][2][2]=0;

  }

  //状態関数を定義する--------------------------------------------------------------------------------------------------------------------------
  std::cout <<"状態方程式"<< std::endl;
  std::cout <<"マーカーの状態モデルの更新(推定式)xEST=\n"<<xEST<< std::endl;
  std::cout <<"マーカーの状態モデルの更新(推定式)xEST_prev=\n"<<xEST_prev<< std::endl;
  std::cout <<"誤差共分散の更新(推定式)Pt=\n"<<Pt<< std::endl;





  //予測ステップ-----------------------------------------------------------------------------------------------------
  std::cout <<"予測ステップ"<< std::endl;
  //マーカーの状態モデルの予測式-------------
  Ft= (cv::Mat_<float>(3,3) << 
      1,      0, -omegaY,
      0,      1, 0,
      omegaY, 0, 1);
  std::cout <<"Ft=\n"<<Ft<< std::endl;
  

  Gt= (cv::Mat_<float>(3,6) << 
    -1, 0, 0, 0, -xEst_prev[2], xEst_prev[1],
    0, -1, 0, xEst_prev[2], 0, -xEst_prev[0],
    0, 0, -1, -xEst_prev[1], xEst_prev[0], 0); 
  std::cout <<"Gt=\n"<<Gt<< std::endl;
  

  Vt= (cv::Mat_<float>(6,1) <<
    Vx,
    0,
    Vz,
    0,
    0,
    0);
  std::cout <<"Vt=\n"<<Vt<< std::endl;
  
  
  //xEST_prev= (cv::Mat_<float>(3,1) <<
  //  xEst_prev[0],
  //  xEst_prev[1],
  //  xEst_prev[2]);

  xEST=Ft*xEST_prev+Gt*Vt;
  std::cout <<"マーカーの状態モデル(予測式)xEST=\n"<<xEST<< std::endl;

  //誤差共分散の予測式-----------------------------------
  float X_p,Y_p;
  //ここのDepthが０になると値がおかしくなるからそれを考慮したものを作る必要がある
  if(kaisu==0){
    X_p=MarkerC[markerIds.at(0)][2][0]-320;//初回は観測値を使う(正規化画像座標系)
    Y_p=MarkerC[markerIds.at(0)][2][1]-240;
  }
  else{
    X_p=camera_info.K[0]*xEst[0]/xEst[2];//二回目以降は推定値を使う
    Y_p=camera_info.K[0]*xEst[1]/xEst[2];
  }

  std::cout <<"X_p="<<X_p<< std::endl;
  std::cout <<"Y_p="<<Y_p<< std::endl;
  
  Ot= (cv::Mat_<float>(2, 6) <<//一つ前の推定結果を使う
    camera_info.K[0], 0, -X_p, -X_p*xEst_prev[1], X_p*xEst_prev[0]+camera_info.K[0]*xEst_prev[2], -camera_info.K[0]*xEst_prev[1],
    0, camera_info.K[0], -Y_p, -Y_p*xEst_prev[1]-camera_info.K[0]*xEst_prev[2], -Y_p*xEst_prev[0],camera_info.K[0]*xEst_prev[0]);

  std::cout <<"Ot=\n"<<Ot<< std::endl;
  
  Ot_1=Ot.t()*Ot;
  std::cout <<"Ot_1=\n"<<Ot_1<< std::endl;

  Ot_1_T=Ot_1.inv();
  std::cout <<"Ot_1_T=\n"<<Ot_1_T<< std::endl;

  Wt=Ot_1_T*Ot.t()*delta*Ot*Ot_1_T.t();
  std::cout <<"Wt=\n"<<Wt<< std::endl;


  Mt=Ft*Pt*Ft.t()+Gt*Wt*Gt.t();
  std::cout <<"誤差共分散(予測式)Mt=\n"<<Mt<< std::endl;

//更新ステップ------------------------------------------------------------------
//観測残差
  if(kaisu==0){
    Ut= (cv::Mat_<float>(2, 1) <<
      0,
      0);
    Ht= (cv::Mat_<float>(2, 3) <<
      0, 0, 0,
      0, 0, 0);
  }
  else{
    Ut= (cv::Mat_<float>(2, 1) <<
      camera_info.K[0]*xEst_prev[0]/xEst_prev[2],
      camera_info.K[0]*xEst_prev[1]/xEst_prev[2]);
    Ht= (cv::Mat_<float>(2, 3) <<
      camera_info.K[0]/xEst_prev[2], 0, -camera_info.K[0]*xEst_prev[0]/xEst_prev[2]*xEst_prev[2],
      0, camera_info.K[0]/xEst_prev[2], -camera_info.K[0]*xEst_prev[1]/xEst_prev[2]*xEst_prev[2]);
  }
  Zt=(cv::Mat_<float>(2, 1) <<//観測データZt
    MarkerC[markerIds.at(0)][2][0]-320,
    MarkerC[markerIds.at(0)][2][1]-240);

  Yt=Zt-(Ht*xEST+Ut);
  std::cout <<"観測残差Yt=\n"<<Yt<< std::endl;
   
  //カルマンゲイン
  Kt=Mt*Ht.t()*u_.inv();
  std::cout <<"カルマンゲインKt=\n"<<Kt<< std::endl;
  std::cout <<"Kt*Yt=\n"<<Kt*Yt<< std::endl;
  //状態モデルの更新
  xEST=xEST+(Kt*Yt);
  std::cout <<"マーカーの状態モデルの更新(推定式)xEST=\n"<<xEST<< std::endl;


  //誤差共分散の更新
  Pt=Mt.inv()+Ht.t()*u_.inv()*Ht;
  Pt=Pt.inv();
  std::cout <<"誤差共分散の更新(推定式)Pt=\n"<<Pt<< std::endl;






  cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
  cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

  cv::imshow(win_src, image);
  //cv::imshow(win_dst, img_dst);
  cv::imshow(win_dst, imageCopy);

	
  if (markerIds.size() > 0) {
    for(int i=0;i<markerIds.size();i++){
      for(int j=0;j<4;j++){
        marker_prev[markerIds.at(i)][j][0]=point[markerIds.at(i)][j][0];
        marker_prev[markerIds.at(i)][j][1]=point[markerIds.at(i)][j][1];
        marker_prev[markerIds.at(i)][j][2]=point[markerIds.at(i)][j][2];
      }
    }
  }
  xEST_prev = xEST.clone();
  std::cout <<"マーカーの状態モデルの更新(推定式)xEST_prev=\n"<<xEST_prev<< std::endl;

  //xEst_prev[0]=xEst[0];
  //xEst_prev[1]=xEst[1];
  //xEst_prev[2]=xEst[2];

  kaisu=kaisu+1;
   //ros::spinにジャンプする
  cv::waitKey(1);
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"marker_EKF");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nhSub, "/camera/color/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::CameraInfo> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub, info_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2, _3));


	ros::spin();//トピック更新待機
			
	return 0;
}






