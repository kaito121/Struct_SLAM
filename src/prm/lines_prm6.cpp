//内部パラメータ、外部パラメータの検出とカメラ座標から世界座標系への座標変換
//FLD化済み
//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
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
#include <Eigen/Dense>//Eigen用
#include <Eigen/Core>//cvMat->Eigen変換用
#include <opencv2/core/core.hpp>//cvMat->Eigen変換用
#include <opencv2/core/eigen.hpp>//cvMat->Eigen変換用



ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher marker_pub;
ros::Publisher marker_pub_W;
// ros::Publisher image_pub;
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_dst2 = "dst2";
std::string win_depth = "depth";
std::string win_line = "line";


using namespace std;
using namespace cv;
using namespace Eigen;
using Eigen::MatrixXd;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 0;
int sec=0;
cv::Mat RGBimage3;//ここで定義する必要がある

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	  cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
    

	//ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示

	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;}

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");}//printと秒数表示
    
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;}
        
    std::cout << "kaisu=" << kaisu << std::endl;
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入


//画像マッチング（パラメータ推定）
  cv::Mat img_prm[2], img_prmw[2], img_match, img_per, img_reg;
  cv::Scalar color[2] = { cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0) };

   // 画像読み込み(初回はこっち)
    if(kaisu==0){
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage;}// 画像読み込み
	//二回目以降
	else{
	img_prm[0] = RGBimage; // 画像読み込み
	img_prm[1] = RGBimage3;} // 画像読み込み
	cv::rectangle(img_prm[0], cv::Point(0, 0), cv::Point(img_prm[0].cols, img_prm[0].rows), color[0], 2); // 外枠
  cv::rectangle(img_prm[1], cv::Point(0, 0), cv::Point(img_prm[1].cols, img_prm[1].rows), color[1], 2); // 外枠
	img_prmw[0] = cv::Mat::zeros(img_prm[0].size() * 2, img_prm[0].type());
  img_prmw[1] = cv::Mat::zeros(img_prm[1].size() * 2, img_prm[1].type());
	cv::Mat roi1 = img_prmw[0](cv::Rect(img_prmw[0].cols / 4, img_prmw[0].rows / 4, img_prm[0].cols, img_prm[0].rows));
  cv::Mat roi2 = img_prmw[1](cv::Rect(img_prmw[1].cols / 4, img_prmw[1].rows / 4, img_prm[1].cols, img_prm[1].rows));
	img_prm[0].copyTo(roi1); // 縦横倍のMatの中央にコピー
  img_prm[1].copyTo(roi2); // 縦横倍のMatの中央にコピー
	
	//cv::imshow("img_prm[0]", img_prmw[0]);
	//cv::imshow("img_prm[1]", img_prmw[1]);

  // (1) 特徴点抽出(ORB)
	cv::Ptr<cv::ORB> detector = cv::ORB::create();// FeatureDetectorオブジェクトの生成
	cv::Ptr<cv::ORB> extractor = cv::ORB::create();// DescriptionExtractorオブジェクトの生成
	cv::BFMatcher matcher(cv::NORM_HAMMING); // DescriptorMatcherオブジェクトの生成
  std::vector<cv::KeyPoint> kpts1, kpts2;// 特徴点情報を格納するための変数
  detector->detect(img_prmw[0], kpts1);// 特徴点抽出の実行
  detector->detect(img_prmw[1], kpts2);
  cv::Mat desc1, desc2;// 画像の特徴情報を格納するための変数
  extractor->compute(img_prmw[0], kpts1, desc1);// 特徴記述の計算を実行
  extractor->compute(img_prmw[1], kpts2, desc2);
  std::vector<cv::DMatch> matches;// 特徴点のマッチング情報を格納する変数
  matcher.match(desc1, desc2, matches);// 特徴点マッチングの実行
	ROS_INFO("maxtutinngu");//printと秒数表示

  std::cout << "best = " << best << std::endl;
	std::cout << "match size = " << matches.size() << std::endl;
	if (matches.size() < best) {
		std::cout << "few matchpoints" << std::endl;
	}
	
	// 上位best個を採用
	std::nth_element(begin(matches), begin(matches) + best - 1, end(matches));
	matches.erase(begin(matches) + best, end(matches));
	std::cout << "matchs size = " << matches.size() << std::endl;
	
	// 特徴点の対応を表示
	cv::drawMatches(img_prmw[0], kpts1, img_prmw[1], kpts2, matches, img_match);
	cv::imshow("matchs", img_match);
	
	// 特徴点をvectorにまとめる
	std::vector<cv::Point2f> points_src, points_dst;
	for (int i = 0; i < matches.size(); i++) {
		points_src.push_back(kpts1[matches[i].queryIdx].pt);
		points_dst.push_back(kpts2[matches[i].trainIdx].pt);
	}
	
	// (3) マッチング結果から，F行列を推定する
	cv::Mat F = cv::findFundamentalMat(points_src, points_dst);
	std::cout << "F=" << F << std::endl;

  // (4) カメラの内部パラメータが既知の場合はE行列を計算し，外部パラメータを推定する
	// カメラ内部パラメータ読み込み
	cv::Mat V;
	cv::FileStorage fs;
  fs.open("realsense_para.xml", cv::FileStorage::READ);
	fs["intrinsic"]>>V;
	std::cout << "V=" << V << std::endl;
	
	// E行列の計算
	cv::Mat E = cv::findEssentialMat(points_src, points_dst, V);
	
	// 外部パラメータ（回転，並進ベクトル）の計算
	cv::Mat R, t;
	cv::recoverPose(E, points_src, points_dst, V, R, t);

	std::cout << "E=" << E << std::endl;
	std::cout << "R=" << R << std::endl;
	std::cout << "t=" << t << std::endl;

	//行列の要素抜き出し（数値)
  std::cout << "R(0,0)=" << R.at<double>(0, 0) << std::endl << std::endl;

   //配列の列の追加方法(斉次化してないバージョン)
  //cv::Mat_<double> Rt_ = cv::Mat_<double>(3, 4);        
  //Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0);
  //cv::Mat Rt = Rt_;
  //std::cout << "Rt=" << Rt << std::endl << std::endl;

  //配列の列の追加方法(斉次化バージョン)
  cv::Mat_<double> Rt_ = cv::Mat_<double>(4, 4);        
  Rt_ << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),t.at<double>(1, 0),R.at<double>(2, 0),R.at<double>(2, 1),R.at<double>(2, 2),t.at<double>(2, 0),0,0,0,1;
  cv::Mat Rt = Rt_;
  std::cout << "Rt=" << Rt << std::endl << std::endl;


//ここに処理項目
	  cv::Mat img_src = bridgeRGBImage->image.clone();
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_dst2;
    cv::Mat img_line,img_line1;
    cv::Mat img_depth2,img_depth3,img_depth4;

    img_src.copyTo(img_dst);
    img_src.copyTo(img_dst2);

    //Depth修正
    img_depth2 = img_depth.clone();//depthの画像をコピーする
    //画像クロップ(中距離でほぼ一致)
    cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
    cv::Mat img_dstdepth = img_depth(roi); // 切り出し画像
    resize(img_dstdepth, img_depth3,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
    img_depth4 = img_depth3.clone();//depth3の画像をコピーする
    
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);


    float dep,dep1[300],dep2[300];
    double theta[300];
    //Y軸との角度(詳しくは2月の研究ノート)
   // theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    //theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)  
    
    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);

    //FLD変換
    std::vector<cv::Vec4f> lines0;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines0);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines0.size(); i++){
       cv::line(img_dst,cv::Point(lines0[i][0],lines0[i][1]),cv::Point(lines0[i][2],lines0[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines0[i][0],lines0[i][1]),cv::Point(lines0[i][2],lines0[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_depth4,cv::Point(lines0[i][0],lines0[i][1]),cv::Point(lines0[i][2],lines0[i][3]),cv::Scalar(0,0,255), 0.5, cv::LINE_AA);
    }
    cv::cvtColor(img_line, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 200, 200);

    //確率的ハフ変換(元画像・lines)
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 80,30,10);

    //確率的ハフ変換座標と三次元距離データの結合と画像描写
    for(int i = 0; i < lines.size(); i++){
        dep1[i]= img_depth3.at<float>(lines[i][0],lines[i][1]);//点の三次元距離データ取得
        dep2[i]= img_depth3.at<float>(lines[i][2],lines[i][3]);

        if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          dep= dep1[i] - dep2[i];
          if(abs(dep)<500){//2点の三次元距離の差が500(50cm)以下だった場合水平か垂直の線
             //img_lineは確率的ハフ変換のラインのみの画像
             cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);
             //cv::line(img_line,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);   
            }
          else{
             cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,255,255), 4, cv::LINE_AA);
             //cv::line(img_line,cv::Point(lines[i][0],lines[i][1]), cv::Point(lines[i][2],lines[i][3]), cv::Scalar(0,0,255), 4, cv::LINE_AA);  
            }

          cv::circle(img_dst,Point(lines[i][0],lines[i][1]),10,Scalar(255,0,0),-1);//青点
          cv::circle(img_dst,Point(lines[i][2],lines[i][3]),10,Scalar(0,255,0),-1);//緑点

         std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<","<<dep1[i]<<")"<< std::endl;
         std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<","<<dep2[i]<<")"<< std::endl;

         //確率的ハフ変換線のy軸との角度を求める
         theta[i]=M_PI-atan2((lines[i][2]-lines[i][0]),(lines[i][3]-lines[i][1]));
         std::cout <<"確率的ハフ変換の傾きl["<<i<<"]("<<theta[i]<< std::endl;
        }
    }


 /* thetaの数値を小さいにソート */
  double tmp,tmp1x,tmp1y,tmp2x,tmp2y,tmpdep1,tmpdep2;
  for (int i=0; i<=lines.size(); ++i) {
     for (int j=i+1;j<lines.size(); ++j) {
         if (theta[i] > theta[j]) {
             tmp =  theta[i];
             tmp1x =  lines[i][0];
             tmp1y =  lines[i][1];
             tmp2x =  lines[i][2];
             tmp2y =  lines[i][3];
             tmpdep1 = dep1[i];
             tmpdep2 = dep2[i];

             theta[i] = theta[j];
             lines[i][0] = lines[j][0];
             lines[i][1] = lines[j][1];
             lines[i][2] = lines[j][2];
             lines[i][3] = lines[j][3];
             dep1[i] = dep1[j];
             dep2[i] = dep2[j];

             theta[j] = tmp;
             lines[j][0] = tmp1x;
             lines[j][1] = tmp1y;
             lines[j][2] = tmp2x;
             lines[j][3] = tmp2y;
             dep1[j] = tmpdep1;
             dep2[j] = tmpdep2;
            }
        }
    }

     std::cout <<"並び替え後"<<std::endl;
    for (int i=0; i< lines.size(); ++i) {
        std::cout <<"pt1["<<i<<"]("<<lines[i][0]<<","<<lines[i][1]<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines[i][2]<<","<<lines[i][3]<<")"<< std::endl;
        std::cout <<"FLD抽出線の傾きl2["<<i<<"]("<<theta[i]<<")"<< std::endl;
        //cv::line(img_line2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
    }
    //角度データテキスト化
    //for (int i=0; i<=lines.size(); ++i) {std::cout <<theta[i]<< std::endl;}

    //線が縦線か横線か見極め２つに分類する（縦線0,横線１)

    int yokot,tatet,yokoj,tatej,p,yokoyouso,tateyouso;
    double yoko[200][20][2][4],tate[200][20][2][4],yokoB,tateB,linesX,linesY,yokolines[200][4],tatelines[200][4],yokotheta[200],tatetheta[200];
    double yokoZ1[200],yokoZ2[200],tateZ1[200],tateZ2[200];
    double naname[200][20][2][4],nanameB;
    double yokothetal[100],tatethetal[100],yokolc[100],tatelc[100],yokol[200][4],tatel[200][4];
    double ynanamethetal[100],tnanamethetal[100],ynanamelc[100],tnanamelc[100],ynanamel[200][4],tnanamel[200][4];
    double A[4][200][2][4];
    int youso[4];
    
    yokot=0,tatet=0,yokoj=1,tatej=1,p=0,yokoyouso=0,tateyouso=0;
    youso[0]=0,youso[1]=0,youso[2]=0,youso[3]=0;

    for (int j=0; j< lines.size(); ++j) {
        linesX=abs(lines[j][0]-lines[j][2]);//傾きを調べる（x成分)
        linesY=abs(lines[j][1]-lines[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(linesX>linesY){
            std::cout <<"yoko(linesX>linesY)="<<linesX<<">"<<linesY<< std::endl;
            yokolines[yokoyouso][0]=lines[j][0];//(x成分)
            yokolines[yokoyouso][1]=lines[j][1];//(y成分)
            yokolines[yokoyouso][2]=lines[j][2];//(x成分)
            yokolines[yokoyouso][3]=lines[j][3];//(y成分)
            yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=dep1[j];
            yokoZ2[yokoyouso]=dep2[j];

            yokoB=yokotheta[0];
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)
        }
        //縦線に分類
        else{
        if(linesY>linesX){
            std::cout <<"tate(linesY>linesX)="<<linesY<<">"<<linesX<< std::endl;
            tatelines[tateyouso][0]=lines[j][0];
            tatelines[tateyouso][1]=lines[j][1];
            tatelines[tateyouso][2]=lines[j][2];
            tatelines[tateyouso][3]=lines[j][3];
            tatetheta[tateyouso]=theta[j];
            tateZ1[tateyouso]=dep1[j];
            tateZ2[tateyouso]=dep2[j];

            tateB=tatetheta[0];
            tateyouso=tateyouso+1;//縦線に分類されたグループ数(tateyouso)
        }}
        }

    //このグルーピングは縦横で分類したあと、縦線と横線の範囲を定め、その範囲で縦線と横線の分類を行っている
    //グルーピング(横線)
    for(int i = 0; i < yokoyouso; i++){
            //横成分の範囲を指定する
            if((yokotheta[i+1]>=1.3962634)&&(1.745329>=yokotheta[i+1])){
            std::cout <<"yokotheta["<<i+1<<"]= 横成分の範囲内（1.3962634〜1.745329) "<< std::endl;
            std::cout <<"yokotheta["<<i+1<<"]="<<yokotheta[i+1]<< std::endl;

            A[0][youso[0]][0][0]=yokolines[i+1][0];//ヨコ線のu1座標
            A[0][youso[0]][0][1]=yokolines[i+1][1];//ヨコ線のv1座標
            A[0][youso[0]][0][2]=yokoZ1[i+1];//ヨコ線のz1座標(三次元距離情報)
            A[0][youso[0]][0][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[0][youso[0]][1][0]=yokolines[i+1][2];//ヨコ線のu2座標
            A[0][youso[0]][1][1]=yokolines[i+1][3];//ヨコ線のv2座標
            A[0][youso[0]][1][2]=yokoZ2[i+1];//ヨコ線のz2座標(三次元距離情報)
            A[0][youso[0]][1][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)


          std::cout <<"チェックA[0]["<<youso[0]<<"][0][0]= "<<A[0][youso[0]][0][0]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][0][1]= "<<A[0][youso[0]][0][1]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][0][2]= "<<A[0][youso[0]][0][2]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][0][3]= "<<A[0][youso[0]][0][3]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]

          std::cout <<"チェックA[0]["<<youso[0]<<"][1][0]= "<<A[0][youso[0]][1][0]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][1][1]= "<<A[0][youso[0]][1][1]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][1][2]= "<<A[0][youso[0]][1][2]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<youso[0]<<"][1][3]= "<<A[0][youso[0]][1][3]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]

            youso[0]=youso[0]+1;
          
          //cv::line(img_line3,cv::Point(yokolines[i+1][0],yokolines[i+1][1]),cv::Point(yokolines[i+1][2],yokolines[i+1][3]),cv::Scalar(0,255,0), 2, cv::LINE_AA);
          //cv::line(img_line4,cv::Point(yokolines[i+1][0],yokolines[i+1][1]),cv::Point(yokolines[i+1][2],yokolines[i+1][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA);
           
            /*//座標から一次関数を引く関数
            yokothetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines[i+1][2]-yokolines[i+1][0]),(yokolines[i+1][3]-yokolines[i+1][1])));
            yokolc[i+1]=(yokolines[i+1][2]-yokolines[i+1][0])*(yokolines[i+1][2]-yokolines[i+1][0])+(yokolines[i+1][3]-yokolines[i+1][1])*(yokolines[i+1][3]-yokolines[i+1][1]);
            yokol[i+1][0]=yokolines[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//X1座標
            yokol[i+1][1]=yokolines[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//Y1座標
            yokol[i+1][2]=yokolines[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//X2座標
            yokol[i+1][3]=yokolines[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< yokothetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< yokol[i+1][0] <<","<<yokol[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< yokol[i+1][2] <<","<<yokol[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(yokol[i+1][0],yokol[i+1][1]),cv::Point(yokol[i+1][2],yokol[i+1][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);*/
            }
        //横成分の範囲に含まれていない斜めの線
        else{ 
             std::cout <<"yokotheta["<<i+1<<"]= 横成分の範囲外（斜め線） "<< std::endl;
             std::cout <<"yokotheta["<<i+1<<"]="<<yokotheta[i+1]<< std::endl;
             std::cout <<"yokoB="<<yokoB<< std::endl;
             std::cout <<"yokotheta[i+1]-yokoB="<<yokotheta[i+1]-yokoB<< std::endl;


            A[2][youso[2]][0][0]=yokolines[i+1][0];//ヨコナナメ線のu1座標
            A[2][youso[2]][0][1]=yokolines[i+1][1];//ヨコナナメ線のv1座標
            A[2][youso[2]][0][2]=yokoZ1[i+1];//ヨコナナメ線のz1座標(三次元距離情報)
            A[2][youso[2]][0][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[2][youso[2]][1][0]=yokolines[i+1][2];//ヨコナナメ線のu2座標
            A[2][youso[2]][1][1]=yokolines[i+1][3];//ヨコナナメ線のv2座標
            A[2][youso[2]][1][2]=yokoZ2[i+1];//ヨコナナメ線のz2座標(三次元距離情報)
            A[2][youso[2]][1][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)


            std::cout <<"チェックA[2]["<<youso[2]<<"][0][0]= "<<A[2][youso[2]][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][0][1]= "<<A[2][youso[2]][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][0][2]= "<<A[2][youso[2]][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][0][3]= "<<A[2][youso[2]][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[2]["<<youso[2]<<"][1][0]= "<<A[2][youso[2]][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][1][1]= "<<A[2][youso[2]][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][1][2]= "<<A[2][youso[2]][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<youso[2]<<"][1][3]= "<<A[2][youso[2]][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
        
             youso[2]=youso[2]+1;//配列繰り上がり、ｊリセット
             //cv::line(img_line3,cv::Point(yokolines[i+1][0],yokolines[i+1][1]),cv::Point(yokolines[i+1][2],yokolines[i+1][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

            /* //座標から一次関数を引く関数
            ynanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines[i+1][2]-yokolines[i+1][0]),(yokolines[i+1][3]-yokolines[i+1][1])));
            ynanamelc[i+1]=(yokolines[i+1][2]-yokolines[i+1][0])*(yokolines[i+1][2]-yokolines[i+1][0])+(yokolines[i+1][3]-yokolines[i+1][1])*(yokolines[i+1][3]-yokolines[i+1][1]);
            ynanamel[i+1][0]=yokolines[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//X1座標
            ynanamel[i+1][1]=yokolines[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//Y1座標
            ynanamel[i+1][2]=yokolines[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//X2座標
            ynanamel[i+1][3]=yokolines[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< ynanamethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< ynanamel[i+1][0] <<","<<ynanamel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< ynanamel[i+1][2] <<","<<ynanamel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(ynanamel[i+1][0],ynanamel[i+1][1]),cv::Point(ynanamel[i+1][2],ynanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);*/
                     
        }
    } 

    //グルーピング(縦線)
    for(int i = 0; i < tateyouso; i++){
            //縦成分の範囲を指定する
            if((tatetheta[i+1]>=0&&0.1745329>=tatetheta[i+1])||(tatetheta[i+1]>=2.9670597&&3.14159265>=tatetheta[i+1])){
            std::cout <<"tatetheta["<<i+1<<"]= 縦成分の範囲内（0〜0.1745329、2.9670597〜3.14159265) "<< std::endl;
            std::cout <<"tatetheta["<<i+1<<"]="<<tatetheta[i+1]<< std::endl;

            A[1][youso[1]][0][0]=tatelines[i+1][0];//タテ線のu1座標
            A[1][youso[1]][0][1]=tatelines[i+1][1];//タテ線のv1座標
            A[1][youso[1]][0][2]=tateZ1[i+1];//タテ線のz1座標(三次元距離情報)
            A[1][youso[1]][0][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[1][youso[1]][1][0]=tatelines[i+1][2];//タテ線のu2座標
            A[1][youso[1]][1][1]=tatelines[i+1][3];//タテ線のv2座標
            A[1][youso[1]][1][2]=tateZ2[i+1];//タテ線のz2座標(三次元距離情報)
            A[1][youso[1]][1][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            std::cout <<"チェックA[1]["<<youso[1]<<"][0][0]= "<<A[1][youso[1]][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][0][1]= "<<A[1][youso[1]][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][0][2]= "<<A[1][youso[1]][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][0][3]= "<<A[1][youso[1]][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[1]["<<youso[1]<<"][1][0]= "<<A[1][youso[1]][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][1][1]= "<<A[1][youso[1]][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][1][2]= "<<A[1][youso[1]][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<youso[1]<<"][1][3]= "<<A[1][youso[1]][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            youso[1]=youso[1]+1;
            //cv::line(img_line3,cv::Point(tatelines[i+1][0],tatelines[i+1][1]),cv::Point(tatelines[i+1][2],tatelines[i+1][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
            //cv::line(img_line4,cv::Point(tatelines[i+1][0],tatelines[i+1][1]),cv::Point(tatelines[i+1][2],tatelines[i+1][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
             /*//座標から一次関数を引く関数
            tatethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines[i+1][2]-tatelines[i+1][0]),(tatelines[i+1][3]-tatelines[i+1][1])));
            tatelc[i+1]=(tatelines[i+1][2]-tatelines[i+1][0])*(tatelines[i+1][2]-tatelines[i+1][0])+(tatelines[i+1][3]-tatelines[i+1][1])*(tatelines[i+1][3]-tatelines[i+1][1]);
            tatel[i+1][0]=tatelines[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//X1座標
            tatel[i+1][1]=tatelines[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//Y1座標
            tatel[i+1][2]=tatelines[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//X2座標
            tatel[i+1][3]=tatelines[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tatethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tatel[i+1][0] <<","<<tatel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tatel[i+1][2] <<","<<tatel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(tatel[i+1][0],tatel[i+1][1]),cv::Point(tatel[i+1][2],tatel[i+1][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);*/
            } 

        //縦成分の範囲に含まれていない斜めの線
        else{
             std::cout <<"tatetheta["<<i+1<<"]= 縦成分の範囲外（斜め線） "<< std::endl;
             std::cout <<"tatetheta["<<i+1<<"]="<<tatetheta[i+1]<< std::endl;
             std::cout <<"tateB="<<tateB<< std::endl;
             std::cout <<"tatetheta[i+1]-tateB="<<tatetheta[i+1]-tateB<< std::endl;

            A[3][youso[3]][0][0]=tatelines[i+1][0];//タテナナメ線のu1座標
            A[3][youso[3]][0][1]=tatelines[i+1][1];//タテナナメ線のv1座標
            A[3][youso[3]][0][2]=tateZ1[i+1];//タテナナメ線のz1座標(三次元距離情報)
            A[3][youso[3]][0][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[3][youso[3]][1][0]=tatelines[i+1][2];//タテナナメ線のu2座標
            A[3][youso[3]][1][1]=tatelines[i+1][3];//タテナナメ線のv2座標
            A[3][youso[3]][1][2]=tateZ2[i+1];//タテナナメ線のz2座標(三次元距離情報)
            A[3][youso[3]][1][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            std::cout <<"チェックA[3]["<<youso[3]<<"][0][0]= "<<A[3][youso[3]][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][0][1]= "<<A[3][youso[3]][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][0][2]= "<<A[3][youso[3]][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][0][3]= "<<A[3][youso[3]][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[3]["<<youso[3]<<"][1][0]= "<<A[3][youso[3]][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][1][1]= "<<A[3][youso[3]][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][1][2]= "<<A[3][youso[3]][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<youso[3]<<"][1][3]= "<<A[3][youso[3]][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

             
        
             youso[3]=youso[3]+1;//配列繰り上がり、ｊリセット
             //cv::line(img_line3,cv::Point(tatelines[i+1][0],tatelines[i+1][1]),cv::Point(tatelines[i+1][2],tatelines[i+1][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

            /*  //座標から一次関数を引く関数
            tnanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines[i+1][2]-tatelines[i+1][0]),(tatelines[i+1][3]-tatelines[i+1][1])));
            tnanamelc[i+1]=(tatelines[i+1][2]-tatelines[i+1][0])*(tatelines[i+1][2]-tatelines[i+1][0])+(tatelines[i+1][3]-tatelines[i+1][1])*(tatelines[i+1][3]-tatelines[i+1][1]);
            tnanamel[i+1][0]=tatelines[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//X1座標
            tnanamel[i+1][1]=tatelines[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//Y1座標
            tnanamel[i+1][2]=tatelines[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//X2座標
            tnanamel[i+1][3]=tatelines[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tnanamethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tnanamel[i+1][0] <<","<<tnanamel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tnanamel[i+1][2] <<","<<tnanamel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(tnanamel[i+1][0],tnanamel[i+1][1]),cv::Point(tnanamel[i+1][2],tnanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA); */       
        }

    } 



    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/robot1/camera_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines_hough";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 2;

    //マーカーの種類
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // 大きさを決める
    // LINE_STRIP/LINE_LIST マーカは、線幅に対してスケールの x 成分のみを使用します。
    line_list.scale.x = 0.1;

    /*//ここで色をつける;
    // Line list is red(ラインリストは赤)
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;*/

    visualization_msgs::Marker line_listW;
    line_listW.header.frame_id = "/robot1/camera_link";
    line_listW.header.stamp = ros::Time::now();
    line_listW.ns = "lines_hough";
    line_listW.action = visualization_msgs::Marker::ADD;
    line_listW.pose.orientation.w = 1.0;

    line_listW.id = 2;

    //マーカーの種類
    line_listW.type = visualization_msgs::Marker::LINE_LIST;

    // 大きさを決める
    // LINE_STRIP/LINE_LIST マーカは、線幅に対してスケールの x 成分のみを使用します。
    line_listW.scale.x = 0.1;

    /*//ここで色をつける;
    // Line list is red(ラインリストは赤)
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;*/

    double C1_D[4][200][3][0],C2_D[4][200][3][0];


    //グルーピングチェック
    for(int j=0;j<=4;j++){
      for(int i=0;i<=youso[j];i++){
          if(A[j][i][0][2]>0 && A[j][i][1][2]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          //Aはthetaの数値セット

          std::cout << "Second=" << sec << std::endl;
          std::cout <<"チェックA["<<j<<"]["<<i<<"][0][0]= "<<A[j][i][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][0][1]= "<<A[j][i][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][0][2]= "<<A[j][i][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][0][3]= "<<A[j][i][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

          std::cout <<"チェックA["<<j<<"]["<<i<<"][1][0]= "<<A[j][i][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][1][1]= "<<A[j][i][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][1][2]= "<<A[j][i][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
          std::cout <<"チェックA["<<j<<"]["<<i<<"][1][3]= "<<A[j][i][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

          geometry_msgs::Point p;
          p.x = A[j][i][0][0]*0.007;
          p.z = A[j][i][0][1]*0.007;
          p.y = A[j][i][0][2]*0.001;
            // ラインリストは、各ラインに2点必要
          line_list.points.push_back(p);
          p.x =A[j][i][1][0]*0.007;
          p.z =A[j][i][1][1]*0.007;
          p.y =A[j][i][1][2]*0.001;
          line_list.points.push_back(p);

          int R,G,B;
          double ro,rox1,rox2;

          ro=A[j][i][0][0]*cos(A[j][i][0][2])+A[j][i][0][1]*sin(A[j][i][0][2]);
          rox1=ro-1000;
          rox2=ro+1000;

          if(j==0){B=255,G=0,R=0;line_list.color.b = 1.0;}
          if(j==1){B=0,G=255,R=0;line_list.color.g = 1.0;}
          if(j==2){B=0,G=0,R=255;line_list.color.r = 1.0;}
          if(j==3){B=255,G=0,R=255;line_list.color.b = 1.0,line_list.color.r = 1.0;}

          //ここで色をつける;
          // Line list is red(ラインリストは赤)
          line_list.color.r = 1.0;
          line_list.color.a = 1.0;

         if(lines.size()!=0){ 
             cv::line(img_dst,cv::Point(A[j][i][0][0],A[j][i][0][1]), cv::Point(A[j][i][1][0],A[j][i][1][1]), cv::Scalar(B,G,R), 4, cv::LINE_AA);
             cv::line(img_line,cv::Point(A[j][i][0][0],A[j][i][0][1]), cv::Point(A[j][i][1][0],A[j][i][1][1]), cv::Scalar(B,G,R), 4, cv::LINE_AA);
             cv::line(img_line,cv::Point(rox1,1000), cv::Point(rox2,1000), cv::Scalar(B,G,R), 2, cv::LINE_AA);
             }

          marker_pub.publish(line_list);
          // image_pub.publish(image_data);

          //Eigenの定義
          MatrixXd u1_E(0,3);
          MatrixXd C1_E(0,3);
          MatrixXd C2_E(0,3);
          MatrixXd C1z_E(0,3);//距離情報を配列に代入
          MatrixXd C2z_E(0,3);//距離情報を配列に代入
          C1z_E(0,1)=0,C1z_E(0,2)=0,C1z_E(0,3)=A[j][i][0][2];
          C2z_E(0,1)=0,C2z_E(0,2)=0,C2z_E(0,3)=A[j][i][1][2];
         

          //座標変換
          //画像座標系の斉次化
          cv::Mat_<double> u1_ = cv::Mat_<double>(3, 1);        
          u1_ << A[j][i][0][0], A[j][i][0][1], 1;//画像座標系（u1,v1,1)※斉次化
          cv::Mat u1 = u1_;
          cv::Mat u1_1 = u1_;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
          std::cout << "u1=" << u1 << std::endl << std::endl;
          cv::cv2eigen(u1_1,u1_E);//cvMatからEigenに座標変換
          std::cout << "u1_E=" << u1_E << std::endl;

          cv::Mat_<double> u2_ = cv::Mat_<double>(3, 1);        
          u2_ << A[j][i][1][0], A[j][i][1][1], 1;//画像座標系（u2,v2,1)※斉次化
          cv::Mat u2 = u2_;
          std::cout << "u2=" << u2 << std::endl << std::endl;

          // 逆行列を求める(内部パラメータ)
	        cv::Mat Vinv = V.inv();
	        std::cout << "Vinv=\n" << Vinv << std::endl;

          // 逆行列を求める(外部パラメータ)
	        cv::Mat Rtinv = Rt.inv();
	        std::cout << "Rtinv=\n" << Rtinv << std::endl;

          //座標変換画像座標→カメラ座標
          cv::Mat C1 = Vinv * u1;
          cv::Mat C2 = Vinv * u2;
          std::cout << "Second=" << sec << std::endl;
          std::cout << "C1["<<j<<"]["<<i<<"]=\n" << C1 << std::endl;
          std::cout << "C2["<<j<<"]["<<i<<"]=\n" << C2 << std::endl;
          cv::Mat C1_1 = C1;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
          cv::Mat C2_1 = C2;//Eigen変換用（変換用を別に用意しないとエラーが発生する）
          cv::cv2eigen(C1_1,C1_E);//cvMatからEigenに座標変換
          cv::cv2eigen(C2_1,C2_E);//cvMatからEigenに座標変換
          std::cout << "C1_E=\n" << C1_E << std::endl;
          std::cout << "C2_E=\n" << C2_E << std::endl;

          //std::cout << "C1_E+C1z_E=\n" << C1_E+C1z_E << std::endl;
          //std::cout << "C2_E+C2z_E=\n" << C2_E+C2z_E << std::endl;

          std::cout << "C2_E(0,0)=\n" << C2_E(0,0) << std::endl;
          int nrow=3,ncol=0;
          double C1_d[nrow][ncol];
          //Map<RowMatrixXi>(&C1_d[0][0],nrow,ncol)=C1_E
     
          //C1_d[0]=C1_E(0,0);
          //C1_d[1]=C1_E(0,1);
          //C1_d[2]=C1_E(0,1);
          //std::cout << "C1_d1=\n" << C1_d[0][0] << std::endl;
          double value=20.3;
          
          C1_D[j][i][0][0]=C1_E(0,0),C1_D[j][i][1][0]=C1_E(1,0),C1_D[j][i][2][0]=C1_E(2,0);
          C2_D[j][i][0][0]=C2_E(0,0),C2_D[j][i][1][0]=C2_E(1,0),C2_D[j][i][2][0]=C2_E(2,0);
    

          //std::cout << "C2_D[0][0]=\n" << C2_D[0][0] << std::endl;

         
          //sprintf_s(C1_c,"C1[%d][%d]=(%f,%f,%f)",j,i,C1_D[0][0],C1_D[1][0],C1_D[2][0]);
          //sprintf_s(C2_c,"C2[%d][%d]=(%f,%f,%f)",j,i,C2_D[0][0],C2_D[1][0],C2_D[2][0]);
          //sprintf(C2_c,"C2[%d][%d]=(%f,%f,%f)\n",j,i,C2_E(0,0),C2_E(0,1),C2_E(0,2));

          //sprintf(value_c,"value=%f,unti=%f",value,value);
          
          //putText(img_line, C1_c,Point(A[j][i][0][0],A[j][i][0][1]),0,0.5,Scalar(0,0,0),1,CV_AA);
          //putText(img_line, C2_c,Point(A[j][i][1][0],A[j][i][1][1]),0,0.5,Scalar(0,0,0),1,CV_AA);

          //putText(img_line, value_c,Point(20,100),0,0.5,Scalar(0,0,255),1,CV_AA);

          putText(img_line,"Second="+std::to_string(sec),Point(10,10),0,0.5,Scalar(0,0,0),1);
          putText(img_line,"C1[j="+std::to_string(j),Point(A[j][i][0][0],A[j][i][0][1]),0,0.5,Scalar(0,0,0),1);
          putText(img_line,"C1[i="+std::to_string(i),Point(A[j][i][0][0],A[j][i][0][1]+30),0,0.5,Scalar(0,0,0),1);
          putText(img_line,"C2[j="+std::to_string(j),Point(A[j][i][1][0],A[j][i][1][1]),0,0.5,Scalar(0,0,0),1);
          putText(img_line,"C2[i="+std::to_string(i),Point(A[j][i][1][0],A[j][i][1][1]+30),0,0.5,Scalar(0,0,0),1);
          

          //カメラ座標系を斉次化
          C1.resize(4);//配列の行要素を増やす（でも０しか入れられない）
          C2.resize(4);
          cv::Mat n0 = (cv::Mat_<double>(4,1)<<0,0,0,1);
          C1 = C1+n0;//拡張した行列に1を足すことで斉次化
          C2 = C2+n0;
          std::cout << "C1_2=\n" << C1 << std::endl;
          std::cout << "C2_2=\n" << C1 << std::endl;

          //座標変換カメラ座標→世界座標系
          cv::Mat W1 = Rtinv * C1;
          cv::Mat W2 = Rtinv * C2;
          std::cout << "W1=\n" << W1 << std::endl;
          std::cout << "W2=\n" << W2 << std::endl;

          

          geometry_msgs::Point Wp;
          Wp.x = W1.at<double>(0, 0);
          Wp.z = W1.at<double>(0, 1);
          Wp.y = A[j][i][0][2]*0.001;
            // ラインリストは、各ラインに2点必要
          line_listW.points.push_back(Wp);
          Wp.x =W2.at<double>(0, 0);
          Wp.z =W2.at<double>(0, 1);
          Wp.y =A[j][i][1][2]*0.001;
          line_listW.points.push_back(Wp);

          if(j==0){B=255,G=0,R=0;line_listW.color.b = 1.0;}
          if(j==1){B=0,G=255,R=0;line_listW.color.g = 1.0;}
          if(j==2){B=0,G=0,R=255;line_listW.color.r = 1.0;}
          if(j==3){B=255,G=0,R=255;line_listW.color.b = 1.0,line_listW.color.r = 1.0;}

          //ここで色をつける;
          // Line list is red(ラインリストは赤)
          line_listW.color.r = 1.0;
          line_listW.color.a = 1.0;

          marker_pub_W.publish(line_listW);

          }
        }
    }

    for(int j=0;j<=4;j++){
      for(int i=0;i<=youso[j];i++){
          if(A[j][i][0][2]>0 && A[j][i][1][2]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          std::cout << "C1["<<j<<"]["<<i<<"]=("<<C1_D[j][i][0][0]<<","<<C1_D[j][i][1][0]<<","<<C1_D[j][i][2][0] <<")" << std::endl;
          std::cout << "C2["<<j<<"]["<<i<<"]=("<<C2_D[j][i][0][0]<<","<<C2_D[j][i][1][0]<<","<<C2_D[j][i][2][0] <<")" << std::endl;
          }
      }
    }


    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    //cv::imshow(win_src, img_src);
    //cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_depth, img_depth4);
    cv::imshow(win_line, img_line);

    RGBimage3 = bridgeRGBImage->image.clone();//一つ前の画像をメモリー保存
    kaisu=kaisu+1;
    sec=sec+1;

  	cv::waitKey(1);
   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_pub_W = n.advertise<visualization_msgs::Marker>("Would_marker", 10);
  // image_pub = n.advertise<sensor_msgs::Image>("maskImageData", 10);//realsenseの画像データをパブリッシュ


	ros::spin();//トピック更新待機
			
	return 0;
}