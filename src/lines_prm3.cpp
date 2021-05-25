//内部パラメータ、外部パラメータの検出とカメラ座標から世界座標系への座標変換
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

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
int best = 30;
int kaisu= 0;
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

    img_src.copyTo(img_dst);
    img_src.copyTo(img_dst2);
    
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);


    float dep,dep1[100],dep2[100];
    double theta[100];
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
       //cv::line(img_line2,cv::Point(lines0[i][0],lines0[i][1]),cv::Point(lines0[i][2],lines0[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }
    cv::cvtColor(img_line, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 200, 200);

    //確率的ハフ変換(元画像・lines)
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img_edge, lines, 1, CV_PI/180, 80,30,10);

    //確率的ハフ変換座標と三次元距離データの結合と画像描写
    for(int i = 0; i < lines.size(); i++){
        dep1[i]= img_depth.at<float>(lines[i][0],lines[i][1]);//点の三次元距離データ取得
        dep2[i]= img_depth.at<float>(lines[i][2],lines[i][3]);

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
 /* 並び替えした数値を出力 */
  //printf("昇順ソートした数値\n");
  //for (int i=0; i<lines1.size(); ++i){ printf("lines[%d][1]=%f\n", i,lines1[i][1]); }
    int c[20],n,j,p;
    double A[20][20][2][4],B;
    c[0]=1,n=0,j=1,p=0;

    if(lines.size()>0){
    A[0][0][0][0]=lines[0][0];
    A[0][0][0][1]=lines[0][1];
    A[0][0][0][2]=dep1[0];
    A[0][0][0][3]=theta[0];

    A[0][0][1][0]=lines[0][2];
    A[0][0][1][1]=lines[0][3];
    A[0][0][1][2]=dep2[0];
    A[0][0][1][3]=theta[0];
    

    B=theta[0];}
    std::cout <<"初期値B[0]= "<<B<< std::endl;
    std::cout <<"初期値C[0]= "<<c[0]<< std::endl;

    
    //グルーピング
    for(int i = 0; i < lines.size(); i++){

        std::cout <<"lines["<<i<<"][0]= "<<lines[i][0]<< std::endl;
        std::cout <<"lines["<<i<<"][1]= "<<lines[i][1]<< std::endl;
        std::cout <<"B["<<i<<"]= "<<B<< std::endl;

        //前の番号と同じ数値
        if( B==theta[i+1]){
            std::cout <<"theta["<<i+1<<"]= 同じ数値 "<< std::endl;
            A[n][j][0][0]=lines[i+1][0];//代入
            A[n][j][0][1]=lines[i+1][1];//代入
            A[n][j][0][2]=dep1[i+1];//代入
            A[n][j][0][3]=theta[i+1];//代入
            A[n][j][1][0]=lines[i+1][2];//代入
            A[n][j][1][1]=lines[i+1][3];//代入
            A[n][j][1][2]=dep2[i+1];//代入
            A[n][j][1][3]=theta[i+1];//代入
            j=j+1;//配列カウント
            c[n]=c[n]+1;//要素数（同じ数値は何個あるか）
        }
        //前の番号と異なる数値
        else{

          if(theta[i+1]-B>0.3){//前の角度との差が0.5より大きい
             std::cout <<"theta["<<i+1<<"]= 異なる数値 "<< std::endl;
             std::cout <<"theta["<<i+1<<"]="<<theta[i+1]<< std::endl;
             std::cout <<"B="<<B<< std::endl;
             std::cout <<"theta[i+1]-B="<<theta[i+1]-B<< std::endl;
        
             n=n+1,j=0;//配列繰り上がり、ｊリセット
             A[n][j][0][0]=lines[i+1][0];//代入
             A[n][j][0][1]=lines[i+1][1];//代入
             A[n][j][0][2]=dep1[i+1];//代入
             A[n][j][0][3]=theta[i+1];//代入
             A[n][j][1][0]=lines[i+1][2];//代入
             A[n][j][1][1]=lines[i+1][3];//代入
             A[n][j][1][2]=dep2[i+1];//代入
             A[n][j][1][3]=theta[i+1];//代入
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[n]=0;//配列要素数初期値
            }

          else{//前の角度との差が0.5以下
             std::cout <<"theta["<<i+1<<"]= ちょっと違う数値= "<<theta[i+1]-B<< std::endl;
             A[n][j][0][0]=lines[i+1][0];//代入
             A[n][j][0][1]=lines[i+1][1];//代入
             A[n][j][0][2]=dep1[i+1];//代入
             A[n][j][0][3]=theta[i+1];//代入
             A[n][j][1][0]=lines[i+1][2];//代入
             A[n][j][1][1]=lines[i+1][3];//代入
             A[n][j][1][2]=dep2[i+1];//代入
             A[n][j][1][3]=theta[i+1];//代
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[n]=c[n]+1;//要素数（同じ数値は何個あるか）
            }
        }
      std::cout <<"C["<<n+1<<"]="<< c[n] << std::endl;
     
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


    //グルーピングチェック
    for(int j=0;j<=n;j++){
        std::cout <<"チェックC["<<j<<"]= "<<c[j]<< std::endl;
        std::cout <<"チェックt= "<<n<< std::endl;
      for(int i=0;i<c[j];i++){
          if(A[j][i][0][2]>0 && A[j][i][1][2]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          //Aはthetaの数値セット

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

          //座標変換
           //画像座標系の斉次化
          cv::Mat_<double> u1_ = cv::Mat_<double>(3, 1);        
          u1_ << A[j][i][0][0], A[j][i][0][1], 1;
          cv::Mat u1 = u1_;
          std::cout << "u1=" << u1 << std::endl << std::endl;

          cv::Mat_<double> u2_ = cv::Mat_<double>(3, 1);        
          u2_ << A[j][i][1][0], A[j][i][1][1], 1;
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
          std::cout << "C1=\n" << C1 << std::endl;
          std::cout << "C2=\n" << C2 << std::endl;

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
    //cv::imshow(win_depth, img_depth);
    cv::imshow(win_line, img_line);

    RGBimage3 = bridgeRGBImage->image.clone();//一つ前の画像をメモリー保存
    kaisu=kaisu+1;

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