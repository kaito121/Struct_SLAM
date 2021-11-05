//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
//20210601
//カメラをつかってリアルタイムで物体の追跡が可能プログラム
//しかし追跡はできるがマッチングはしていないので、配列はバラバラに格納されている
//そのためオプティカルフローなどをする場合はこれにマッチングのプログラムを導入する必要がある
//20210602
//Ver1では特徴点の更新プログラムが入っておらず画面から消えたら特徴点が消えてしまい追跡ができない
//そこで常に更新と追跡を行うように変更したい
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>//PCLのためのヘッダ
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <struct_slam/opt_point.h>//FLD

#include <struct_slam/Depth_pclConfig.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>//reconfig用
#include<algorithm>//並び替え用




ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher  pub;//クラスタリングのためのパプリッシュ
std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像（修正前）
std::string win_depth2 = "depth2";//深度画像（修正前）+FLD線
std::string win_depth3 = "depth3";//深度画像（修正後）
std::string win_depth4 = "depth4";//深度画像（修正後）+FLD線
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_line = "line";//FLDの線を表示
std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_prev = "prev";
std::string win_curr = "curr";
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";//オプティカルフロー軌跡確認用（すぐに消えない奴）
std::string win_dst4 = "dst4";//オプティカルフロー軌跡確認用（すぐに消える奴）


using namespace std;
using namespace cv;

//dynamic Reconfigure
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    double X_wariai;
    double Y_wariai;
    double WINDOW_SIZE;
    double X_pcl;
    double Y_pcl;
    double Z_pcl;
    double Z_Z=1000;
    double CONTRAST_MAX;
    double CONTRAST_MIN;
    double CLOSE_OPEN;
    double OPEN_CLOSE;
    double NITIKA;
    double OPEN;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_dst3,img_dst4;
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;
vector<cv::Point2f> mappoint;

cv::Mat_<double>World0,WorldC,WorldC0;//世界座標系設定
cv::Mat_<double>Camera0;//カメラ座標系設定
cv::Mat_<double>WorldP;//特徴点の世界座標系設定

//カメラの内部パラメータ情報
cv::Mat_<double>K;//内部パラメータ（詳しくはM2 6月研究ノート)
cv::Mat_<double>K_;//内部パラメータの逆行列

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
cv::Mat FeaturePoint[100];//特徴点周囲の切り取った画像

int kaisu;//行列初期設定用変数




//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat image,depthimage,img_depth2,img_depth3,img_depth4;//opencvの画像
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

  image = bridgeImage->image.clone();//image変数に変換した画像データを代入
  depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
	cv::imshow(win_src, image);
  //cv::imshow(win_depth, depthimage);
	int req;

  // pointcloud を作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointCloud->points.reserve(image.size().width*image.size().height);//点の数

  //Depth修正----------------------------------------------------------------------------------------------------
    img_depth2 = depthimage.clone();//depthの画像をコピーする
    //画像クロップ(中距離でほぼ一致)
    cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
    cv::Mat img_dstdepth = depthimage(roi); // 切り出し画像
    resize(img_dstdepth, img_depth3,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
    img_depth4 = img_depth3.clone();//depth3の画像をコピーする
    cv::imshow(win_depth4, img_depth4);//クロップされたdepth画像

  //行列初期設定-------------------------------------------------------------------------------------------------------------------
  if(kaisu==0){
  World0 = cv::Mat_<double>::zeros(3, 1);  // 世界座標系原点(零行列)
  Camera0 = cv::Mat_<double>::zeros(3, 1);  //カメラ座標系原点(零行列)
  WorldC = cv::Mat_<double>::zeros(3, 1);	  // 世界座標系でのカメラの座標(単位行列)
  WorldC0 = cv::Mat_<double>::zeros(3, 1);	  // 世界座標系でのカメラの座標の初期値(単位行列)
  K = (cv::Mat_<double>(3, 3) << 1.93/0.003, 0, 326.15115000485503, 0, 1.93/0.003, 244.02271379203739,0,0,1);//内部パラメータ（詳しくはM2 6月研究ノート)
  //K = (cv::Mat_<double>(3, 3) << 615.3367309570312, 0, 324.47296142578125, 0, 615.4581298828125, 241.69638061523438,0,0,1);//内部パラメータ(ros_dev/ORB_SLAM2/D435.yaml)
  d0 = (cv::Mat_<double>(3, 4) << 1,0,0,0,0,1,0,0,0,0,1,0);//カメラ座標系と正規化画像座標系の変換に使用する行列
  K_ = cv::Mat_<double>(3, 3);
  K_=K.inv();//内部パラメータの逆行列
  }
  // 行列表示
  std::cout << "世界座標原点:World0=\n" << World0 << std::endl; 
  std::cout << "カメラの座標(世界座標系):WorldC=\n" << WorldC << std::endl; 
  std::cout << "内部パラメータ: K=\n" << K << std::endl;

  //画像処理------------------------------------------------------------------------------------------------------------------------
  image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(img_dst);//ここ間違えていたので注意
  img_dst4 = image.clone();
  img_dst4 = cv::Scalar(0,0,0);
  if(kaisu==0){
    img_dst2 = image.clone();
    img_dst2 = cv::Scalar(0,0,0);
    img_dst3 = image.clone();
    img_dst3 = cv::Scalar(0,0,0);
  }
 

	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化
	
	if (reset == true) {
		std::cout <<"初回検出プログラム"<< std::endl;//最初のフレーム
		// 特徴点検出(グレースケール画像から特徴点検出)
		cv::goodFeaturesToTrack(image_curr, points_curr, 100, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピー
		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      cv::circle(img_dst2, points_curr[i], 3, Scalar(0,255,255), -1, cv::LINE_AA);//追跡記録に描写
      cv::rectangle(img_dst, cv::Point(points_curr[i].x-15,points_curr[i].y+15), 
      cv::Point(points_curr[i].x+15,points_curr[i].y-15), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);//四角形を描写
		}
		reset = false;//if文切り替え
	} 
	else {
		std::cout <<"二回目オプティカルフロー"<< std::endl;// 特徴点追跡(二回目のフレーム)
		vector<uchar> status;//特徴点の数
		vector<float> err;

    std::cout <<"二回目オプティカルフローpoints_curr=\n"<<points_curr<< std::endl;
    std::cout <<"二回目オプティカルフローpoints_prev=\n"<<points_prev<< std::endl;

		cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev, points_curr, status, err);//オプティカルフロー
		std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

		// 追跡できなかった特徴点をリストから削除する
		int i, k,n;
		req=1;
		for (i = k = n =0; i < status.size(); i++)
		{
			std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
			//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
			//statusが0以外の時値を更新する
			if (status[i] != 0) {	
			points_prev[k]   = points_prev[i];
			points_curr[k++] = points_curr[i];
			std::cout <<"status["<<i<<"]がゼロ以外の時"<< std::endl;//0以外の時は変なマークがでる
			}
		}
		std::cout <<"k="<<k<< std::endl;
		points_curr.resize(k);//ここでkの個数でリサイズされている
		points_prev.resize(k);
		//特徴点が100個以下になったら再び特徴点を検出する
		if(status.size()<50){reset = true;}
	}
  //配列定義---------------------------------------------------------------------------------------------------
  float Zs[points_curr.size()];//距離データ定義
  cv::Mat_<double> Screenp[points_curr.size()] = cv::Mat_<double>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<double> Camerap[points_curr.size()] = cv::Mat_<double>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T
  cv::Mat_<double> Worldp[points_curr.size()] = cv::Mat_<double>(3, 1);//点Pの世界座標系（xw,yw,zw)T
  //cv::Mat_<double> CameraP= cv::Mat_<double>(points_curr.size(), 4);//すべての点のカメラ座標(斉次化)
  //cv::Mat_<double> WorldP= cv::Mat_<double>(points_curr.size(), 3);//すべての点の世界座標
  cv::Mat_<double> Rtc= cv::Mat_<double>(3, 4);//外部パラメータ(値は転置されているのに注意)
  cv::Mat_<double> Tc= cv::Mat_<double>(3, 1);//外部パラメータ(並進ベクトル) 
  cv::Mat_<double> Rc= cv::Mat_<double>(3, 3);//外部パラメータ(回転行列) 
  double optx,opty,pointnorm[points_curr.size()],normgoukei=0,normheikin=0;//オプティカルフローノイズ問題対策
  double maxpointnorm=0,minpointnorm=1000,middlepointnorm;
  double optcurrx,optcurry,optprevx,optprevy;
  int NotZeroL=-1;

  
  

	// 特徴点を丸で描く-------------------------------------------------------------------------------------------
	for (int i = 0; i < points_curr.size(); i++) {
		cv::Scalar c(0, 255, 0);//緑
    pointnorm[i]=cv::norm(points_prev[i] - points_curr[i]);//オプティカルフローノイズ問題対策
    normgoukei=normgoukei+pointnorm[i];
    if(pointnorm[i]>=maxpointnorm){maxpointnorm=pointnorm[i];}//オプティカルフローの最大を求める
    if(minpointnorm>=pointnorm[i]){minpointnorm=pointnorm[i];}//オプティカルフローの最小を求める
    std::cout <<"cv::norm(points_prev["<<i<<"] - points_curr["<<i<<"])="<<pointnorm[i]<< std::endl;//特徴点の座標
		if (cv::norm(points_prev[i] - points_curr[i]) > 0.5) {
			c = cv::Scalar(0, 100, 255);//今の特徴点と一つ前の特徴点との差が0.5以上の時オレンジ
		}
		if(i==0){cv::circle(img_dst, points_curr[i], 8, Scalar(0,255,255), -1, cv::LINE_AA);
    }
		if(i==1){cv::circle(img_dst, points_curr[i], 8, Scalar(0,250,0), -1, cv::LINE_AA);}
		if(i==2){cv::circle(img_dst, points_curr[i], 8, Scalar(0,200,255), -1, cv::LINE_AA);}
		if(i==3){cv::circle(img_dst, points_curr[i], 8, Scalar(0,150,255), -1, cv::LINE_AA);}
		if(i==4){cv::circle(img_dst, points_curr[i], 8, Scalar(100,125,0), -1, cv::LINE_AA);}
		if(i==5){cv::circle(img_dst, points_curr[i], 8, Scalar(0,100,255), -1, cv::LINE_AA);}
		if(i==6){cv::circle(img_dst, points_curr[i], 8, Scalar(0,75,255), -1, cv::LINE_AA);}
		if(i==7){cv::circle(img_dst, points_curr[i], 8, Scalar(100,50,255), -1, cv::LINE_AA);}
		if(i==8){cv::circle(img_dst, points_curr[i], 8, Scalar(50,25,255), -1, cv::LINE_AA);}
		if(i==9){cv::circle(img_dst, points_curr[i], 8, Scalar(0,0,255), -1, cv::LINE_AA);}
		if(i==10){cv::circle(img_dst, points_curr[i], 8, Scalar(100,255,200), -1, cv::LINE_AA);}
		else{
			cv::circle(img_dst, points_curr[i], 4, c, -1, cv::LINE_AA);//今の座標情報
			cv::circle(img_dst, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
			cv::line(img_dst,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      cv::line(img_dst,cv::Point(points_curr[i].x,points_prev[i].y),cv::Point(points_prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      cv::line(img_dst,cv::Point(points_prev[i].x,points_curr[i].y),cv::Point(points_prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
      //cv::arrowedLine(img_dst,points_curr[i],points_prev[i],cv::Scalar(0,255,255),1,8,0,1.0);//矢印の描写
      //cv::arrowedLine(img_dst,cv::Point(points_curr[i].x,points_prev[i].y),points_prev[i],cv::Scalar(255,0,0),1,8,0,1.0);//矢印の描写
      //cv::arrowedLine(img_dst,cv::Point(points_prev[i].x,points_curr[i].y),points_prev[i],cv::Scalar(0,255,0),1,8,0,1.0);//矢印の描写
      //cv::circle(img_dst2, points_curr[i], 1, c, -1, cv::LINE_AA);//今の座標情報
      if((points_curr[i].x-points_prev[i].x)>(points_curr[i].y-points_prev[i].y)){
        cv::arrowedLine(img_dst2,points_curr[i],points_prev[i],cv::Scalar(255,255,0),1,8,0,1.0);//矢印の描写
      }
      else{
        cv::arrowedLine(img_dst2,points_curr[i],points_prev[i],cv::Scalar(0,0,255),1,8,0,1.0);//矢印の描写
      }
		}
      cv::arrowedLine(img_dst2,cv::Point(100,100),cv::Point(150,100),cv::Scalar(0,255,255),1,8,0,1.0);//矢印の描写
      cv::arrowedLine(img_dst2,cv::Point(100,200),cv::Point(300,200),cv::Scalar(0,255,255),1,8,0,1.0);//矢印の描写

    	std::cout <<"特徴点の画像座標(curr)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
		  std::cout <<"特徴点の画像座標(prev)["<<i<<"]="<<points_prev[i]<< std::endl;//特徴点の座標

      Zs[i]= img_depth3.at<float>(points_curr[i]);//点の三次元距離データ取得
      std::cout <<"特徴点の距離Zs["<<i<<"]="<<Zs[i]<< std::endl;

      if(Zs[i]!=0){//depthデータが0の時を考慮
        NotZeroL=NotZeroL+1;
        Zs[NotZeroL]=Zs[i];
        }
      else{continue;}//depthデータがゼロの時下の処理をスキップ
      //Zs[i]=1;
      std::cout <<"特徴点の距離Zs(ノイズ除去後)["<<NotZeroL<<"]="<<Zs[NotZeroL]<< std::endl;

      //Screenp[i] << points_curr[i].x, points_curr[i].y, 1;//斉次化
      Screenp[NotZeroL] << round(points_curr[i].x), round(points_curr[i].y), 1;//斉次化(整数化四捨五入)
      std::cout << "斉次画像座標:Screenp["<<NotZeroL<<"]=\n" << Screenp[NotZeroL] << std::endl;

      Camerap[NotZeroL]=Zs[NotZeroL]*K_*Screenp[NotZeroL];//画像座標系→カメラ座標系座標変換
      std::cout << "特徴点のカメラ座標:Camerap["<<NotZeroL<<"]=\n" << Camerap[NotZeroL] << std::endl;

      /*CameraP(NotZeroL,0)=Camerap[NotZeroL](0,0);//特徴点Pのカメラ座標を一つの行列にまとめる(斉次化)
      CameraP(NotZeroL,1)=Camerap[NotZeroL](1,0);
      CameraP(NotZeroL,2)=Camerap[NotZeroL](2,0);
      CameraP(NotZeroL,3)=1;*/

	}
  if(kaisu%50==0){img_dst2 = cv::Scalar(0,0,0),img_dst3 = cv::Scalar(0,0,0);}//100回で描写記録画像をリセット
  std::cout <<"元データ個数NotZeroL="<<points_curr.size()<< std::endl;
  std::cout <<"Depthノイズを除いたデータ個数NotZeroL="<<NotZeroL<< std::endl;
  cv::Mat_<double> CameraP= cv::Mat_<double>(NotZeroL, 4);//すべての点のカメラ座標(斉次化)
  cv::Mat_<double> WorldP= cv::Mat_<double>(NotZeroL, 4);//すべての点の世界座標
  for (int i = 0; i < NotZeroL; i++) {
      CameraP(i,0)=Camerap[i](0,0);//特徴点Pのカメラ座標を一つの行列にまとめる(斉次化)
      CameraP(i,1)=Camerap[i](1,0);
      CameraP(i,2)=Camerap[i](2,0);
      CameraP(i,3)=1;
	}
  //オプティカルフローノイズ問題についてーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
  normheikin=normgoukei/points_curr.size();
  std::cout <<"平均距離="<<normheikin<< std::endl;//特徴点の座標
  middlepointnorm=(maxpointnorm+minpointnorm)/2;
 


for (int i = 0; i < points_curr.size(); i++) {
  pointnorm[i]=cv::norm(points_prev[i] - points_curr[i]);
  std::cout <<"cv::norm(points_prev["<<i<<"] - points_curr["<<i<<"])="<<pointnorm[i]<< std::endl;//特徴点の座標
  if(pointnorm[i]<normheikin+20){
    /*if((points_curr[i].x-points_prev[i].x)>(points_curr[i].y-points_prev[i].y)){
      //cv::arrowedLine(img_dst3,points_curr[i],points_prev[i],cv::Scalar(255,255,0),1,8,0,1.0);//矢印の描写(水色)(矢印の向きはカメラの動きの向き)
      cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(255,255,0),1,8,0,1.0);//(矢印の向きは特徴点の動きの向き)
    }
    else{
      //cv::arrowedLine(img_dst3,points_curr[i],points_prev[i],cv::Scalar(0,0,255),1,8,0,1.0);//矢印の描写(赤色)(矢印の向きはカメラの動きの向き)
      cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(0,0,255),1,8,0,1.0);//(矢印の向きは特徴点の動きの向き)
    }*/

    if(pointnorm[i] >= (maxpointnorm+middlepointnorm)/2){ //変化
      cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(0,0,255),1,8,0,1.0);//赤色
      cv::arrowedLine(img_dst4,points_prev[i],points_curr[i],cv::Scalar(0,0,255),1,8,0,1.0);//赤色
      }
    else{
      if(pointnorm[i]<(maxpointnorm+middlepointnorm)/2&&pointnorm[i]>=middlepointnorm){
        cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(0,100,255),1,8,0,1.0);//オレンジ色
        cv::arrowedLine(img_dst4,points_prev[i],points_curr[i],cv::Scalar(0,100,255),1,8,0,1.0);//オレンジ色
      }
      else{
        if(pointnorm[i]<middlepointnorm&&pointnorm[i]>=(middlepointnorm+minpointnorm)/2){
          cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(0,255,255),1,8,0,1.0);//黄色  
          cv::arrowedLine(img_dst4,points_prev[i],points_curr[i],cv::Scalar(0,255,255),1,8,0,1.0);//黄色  
        }
        else{
          if((middlepointnorm+minpointnorm)/2 > pointnorm[i]){
            cv::arrowedLine(img_dst3,points_prev[i],points_curr[i],cv::Scalar(0,255,100),1,8,0,1.0);//緑
            cv::arrowedLine(img_dst4,points_prev[i],points_curr[i],cv::Scalar(0,255,100),1,8,0,1.0);//緑

            pcl::PointXYZRGB jk;

            //XYはimageのデータなのでpclにそのままもって行くとでかい そこである定数で割ることで食らうタリング座標に変換する-------------------------(1)
             jk.x=(float)points_curr[i].x/X_wariai;//ピクセル距離をクラスタリング距離に変換
             jk.y=(float)points_curr[i].y/Y_wariai;

             jk.z=0;//ZはDepthデータなのでそのままで行ける

             pointCloud->points.emplace_back(jk);//ポイントクラウドに座標データを移動
           
          }
        }
      }
    }    
  }
}

  //pointcloudサイズ設定
  pointCloud -> width = pointCloud -> points.size();
  pointCloud -> height = 1;
  pointCloud -> is_dense = true;

   // クラスタリングの設定
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (pointCloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  //ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  //ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  //ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
  ec.setClusterTolerance (1000);//同じクラスタとみなす距離
  ec.setMinClusterSize (3);//クラスタを構成する最小の点数
  ec.setMaxClusterSize (30);//クラスタを構成する最大の点数

  ec.setSearchMethod (tree);//s探索方法
  ec.setInputCloud (pointCloud);//クラスタリングするポイントクラウドの指定

  // クラスタリング実行
  std::vector<pcl::PointIndices> indices;
  ec.extract (indices);//結果

  int R[12]={255,255,255,125,125,125,50,50,50,0,0,0};
  int G[12]={50,125,0,50,125,0,255,50,125,0,50,125};
  int B[12]={50,50,50,125,125,125,255,255,255,0,0,0};
  int j=0;
  double MAXPX,MINPX,MAXPY,MINPY,CENTPY,PZ,leftz,rightz,centerz,centerx,centerleftx,centerleftz,centerrightx,centerrightz;

   
    //ROS_INFO("いちばん");//printと秒数表示

     // クラスタリング の結果を色々できるところ(配列にアクセス)
    for (std::vector<pcl::PointIndices>::iterator it = indices.begin (); it != indices.end (); ++it){ // グループにアクセスするループ(it=グループ番号)
        std::sort(it->indices.begin (),it->indices.end (),[&] (int const& a,int const& b){//並び替え
        return pointCloud -> points[a].x > pointCloud -> points[b].x;//並び替えの条件
        });
    }
        //ROS_INFO("なか ");//printと秒数表示
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){ 
        j=j+1; MAXPX=-10000,MINPX=10000,MAXPY=-10000,MINPY=10000;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // グループ中の点にアクセスするループ(pit=グループ内の点番号)
            // 点へのアクセス
      
            pointCloud -> points[*pit].r = R[j%12];//ポイントクラウドのマスの色付け
            pointCloud -> points[*pit].g = G[j%12];
            pointCloud -> points[*pit].b = B[j%12];

            //rectangle(img_dst3, Rect((int)(pointCloud -> points[*pit].x*X_wariai), (int)(pointCloud -> points[*pit].y*Y_wariai),W,H), Scalar(B[j%12],G[j%12],R[j%12]),1);//四角作成
			      cv::circle(img_dst4, cv::Point((int)(pointCloud -> points[*pit].x*X_wariai), (int)(pointCloud -> points[*pit].y*Y_wariai)), 3, Scalar(B[j%12],G[j%12],R[j%12]), -1, cv::LINE_AA);//一つ前の画像の座標

            
            if(MAXPY<=(pointCloud -> points[*pit].y * Y_wariai)){MAXPY = pointCloud -> points[*pit].y * Y_wariai;}//yの最大値(ピクセル座標)
            if(MINPY>=(pointCloud -> points[*pit].y * Y_wariai)){MINPY = pointCloud -> points[*pit].y * Y_wariai;}//pointsをwariaiでかけるとピクセル座標になる(ピクセル座標)
            CENTPY=(MAXPY+MINPY)/2;
            
            //image座標からpcl座標に移行（image座標の原点は左上,pcl座標の原点は画像の中心
            //ピクセルX-Xの高さ半分の値
            //pointsはクラスタリング座標なのである定数をかけることでピクセル座標に変換している、ピクセル座標をpcl定数で割ることでpcl座標に変換している
            pointCloud -> points[*pit].x = ((pointCloud -> points[*pit].x*X_wariai)-(image.size().width/2))/X_pcl; //ピクセル原点からpcl上のX原点変換(pcl座標)
            pointCloud -> points[*pit].y = ((pointCloud -> points[*pit].y*Y_wariai)-(image.size().height/2))/Y_pcl;//ピクセル原点からpcl上のY原点変換(pcl座標)
            pointCloud -> points[*pit].z *= Z_pcl;
            //ここの段階でpointsがPCL座標になる
        }
        //ROS_INFO("おわり");//printと秒数表示
    //itの回数が大枠の個数

     double CENPX,CENPXMAX,CENPXMIN,cenpx=1000,cenpxmin=1000,cenpxmax=1000;
        //pcl座標にpclをかけるとピクセル座標に変換される
         MINPX = pointCloud -> points[*(it->indices.end ()-1)].x * X_pcl +(image.size().width/2);//並び替えたのでitの最後の値がXの最小値となる(leftX)(ピクセル座標)
         MAXPX = pointCloud -> points[*(it->indices.begin ())].x * X_pcl +(image.size().width/2);//(RightX)(ピクセル座標)
         CENPX = (pointCloud -> points[*(it->indices.end ()-1)].x+pointCloud -> points[*(it->indices.begin ())].x)/2;//(CenterX)（pcl座標）
         CENPXMIN = (pointCloud -> points[*(it->indices.end ()-1)].x+CENPX)/2;//LeftXとCenterXの間のX（pcl座標）
         CENPXMAX = (pointCloud -> points[*(it->indices.begin ())].x+CENPX)/2;//RightXとCenterXの間のX（pcl座標）

        std::cout <<"MINPX["<<MINPX<<"]  MINPY["<<MINPY<<"]"<< std::endl;
        std::cout <<"MAXPX["<<MAXPX<<"]  MAXPY["<<MAXPY<<"]"<< std::endl;
        std::cout <<"CENPX["<<CENPX<<"]"<< std::endl;

    //rectangle(img_dst3,Rect(MINPX,MINPY,MAXPX-MINPX+W,MAXPY-MINPY+H),Scalar(24,248,159),1.5);//四角作成
    rectangle(img_dst4,Rect(MINPX,MINPY,MAXPX-MINPX,MAXPY-MINPY),Scalar(24,248,159),1.5);//四角作成
    rectangle(img_dst,Rect(MINPX,MINPY,MAXPX-MINPX,MAXPY-MINPY),Scalar(24,248,159),1.5);//四角作成
    }
  

    


		cv::imshow("win_dst", img_dst);
    cv::imshow("win_dst2", img_dst2);
    cv::imshow("win_dst3", img_dst3);
    cv::imshow("win_dst4", img_dst4);
		cv::imshow("win_curr", image_curr);//今の画像


		if (req == 1) {//初回は直前の画像がないため考慮
		cv::imshow("win_prev", image_prev);//一つ前の画像像
		std::cout <<"でてるよ"<< std::endl;//特徴点の座標
		}


  
   


		int key = cv::waitKey(30);
		if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
		
		cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
		cv::swap(points_curr, points_prev);// points_curr を points_prev に移す（交換する）
		kaisu=kaisu+1;//行列初期設定用変数
 
	cv::waitKey(1);//ros::spinにジャンプする
}

void dynamicParamsCB(struct_slam::Depth_pclConfig &cfg, uint32_t level){
    CLUSTER_TOLERANCE = cfg.cluster_tolerance;
    MIN_CLUSTER_SIZE = cfg.min_cluster_size;
    MAX_CLUSTER_SIZE = cfg.max_cluster_size;
    X_wariai = cfg.X_wariai;
    Y_wariai = cfg.Y_wariai;
    WINDOW_SIZE=cfg.window_size;
    X_pcl = cfg.X_pcl;
    Y_pcl = cfg.Y_pcl;
    Z_pcl = cfg.Z_pcl;
    CONTRAST_MIN = cfg.contrast_min;
    CONTRAST_MAX = cfg.contrast_max;
    OPEN_CLOSE = cfg.open_close;
    CLOSE_OPEN = cfg.close_open;
    NITIKA = cfg.nitika;
    OPEN = cfg.open;

    }



//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"Vpoint_1");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    //reconfigure設定
    dynamic_reconfigure::Server<struct_slam::Depth_pclConfig> drs_;
    drs_.setCallback(boost::bind(&dynamicParamsCB, _1, _2));

    //クラスタリングのためのパブリッシュ設定
    ros::NodeHandle nhPub;
    pub=nhPub.advertise<sensor_msgs::PointCloud2>("depth_pcl", 1000);


	ros::spin();//トピック更新待機
			
	return 0;
}