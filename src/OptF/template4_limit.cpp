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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_dst = "dst";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
std::string win_FeaturePoint = "FeaturePoint";//特徴点を検出したら特徴点の周りの画像をクロップした画像
std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//今のテンプレートを表示したカメラ画像
std::string win_Prev_img_1 = "win_Prev_img_1";//一つ前のテンプレートを表示したカメラ画像

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180
// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1,Prev_img_1;
cv::Mat img_template1,img_template2;
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;
vector<cv::Point2f> mappoint;
vector<cv::Point2f> points_prev_limited, points_curr_limited;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)

cv::Mat_<double>World0,WorldC,WorldC0;//世界座標系設定
cv::Mat_<double>Camera0;//カメラ座標系設定
cv::Mat_<double>WorldP;//特徴点の世界座標系設定

//カメラの内部パラメータ情報
cv::Mat_<double>K;//内部パラメータ（詳しくはM2 6月研究ノート)
cv::Mat_<double>K_;//内部パラメータの逆行列

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
cv::Mat FeaturePoint[100];//特徴点周囲の切り取った画像
cv::Mat Prev_FeaturePoint[100];//特徴点周囲の切り取った画像(一つ前に取得した画像)

int kaisu,optkaisu;//行列初期設定用変数
int req=0;//テンプレートマッチング用カウント数
double mamama;


//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
  cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
  cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
  cv::Mat RGBimage,depthimage,image,img_depth2,img_depth3,img_depth4;//opencvの画像
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
  if(kaisu==0){
    img_dst2 = image.clone();
    img_dst2 = cv::Scalar(0,0,0);
  }
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化

	//初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset == true) {
    optkaisu=0;//オプティカルフロー回数リセット
		std::cout <<"初回検出プログラム["<<optkaisu<<"]"<< std::endl;//最初のフレーム
    image.copyTo(img_1);
		// 特徴点検出(グレースケール画像から特徴点検出)
    //cv::goodFeaturesToTrack(今画像, 前画像, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 3, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    points_curr_limited = points_curr;//points_curr_limitedの定義
    points_prev_limited = points_curr;//points_curr_limitedの定義
    int limited=0;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)

		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      cv::circle(img_dst2, points_curr[i], 3, Scalar(0,255,255), -1, cv::LINE_AA);//追跡記録に描写
      cv::rectangle(img_dst, cv::Point(points_curr[i].x-15,points_curr[i].y+15), 
      cv::Point(points_curr[i].x+15,points_curr[i].y-15), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);//四角形を描写
    
    //特徴点を検出したら特徴点の周りの画像をクロップして保存→その後保存した画像でBoWを行う予定
    //テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
    if(points_curr[i].y<480-30&&points_curr[i].x<640-30){
        points_curr_limited[limited] = points_curr[i];//新しいpoint_vecterに書き換える(外枠15Pixelの特徴点を検出しない)

      std::cout <<"特徴点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
      cv::Rect roi2(cv::Point(points_curr[i].x,points_curr[i].y), cv::Size(30, 30));//特徴点を中心とした15☓15pixelの画像を切り取る
      FeaturePoint[i] = image(roi2); // 切り出し画像
      //cv::rectangle(img_1, cv::Point(points_curr[i].x,points_curr[i].y), cv::Point(points_curr[i].x+30,points_curr[i].y+30), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写

      std::cout <<"特徴点画像クロッププログラムlimited["<<limited<<"]"<< std::endl;
      //cv::Rect roi2(cv::Point(points_curr_limited[i].x,points_curr_limited[i].y), cv::Size(30, 30));//特徴点を中心とした15☓15pixelの画像を切り取る
      //FeaturePoint[i] = image(roi2); // 切り出し画像
      cv::rectangle(img_1, cv::Point(points_curr_limited[i].x,points_curr_limited[i].y), 
      cv::Point(points_curr_limited[i].x+30,points_curr_limited[i].y+30), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写

    
      cv::rectangle(img_1, cv::Point(points_curr_limited[i].x,points_curr_limited[i].y), 
      cv::Point(points_curr_limited[i].x+30,points_curr_limited[i].y+30), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写

      std::cout <<"特徴点の画像座標(curr_if)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
      std::cout <<"特徴点の画像座標(limited)["<<limited<<"]="<<points_curr_limited[limited]<< std::endl;//特徴点の座標
      limited=limited+1;
      }
		}

    points_prev_limited = points_curr_limited;
    points_curr_limited.resize(limited);
    points_prev_limited.resize(limited);
    std::cout <<"制限した特徴点の個数(limited.size)="<<points_curr_limited.size()<< std::endl;
		reset = false;//if文切り替え
  }//------------------------------------------------------------------------------------------------------------------------ 

  //二回目オプティカルフロー--------------------------------------------------------------------------------------------
	else {
    optkaisu=optkaisu+1;
		std::cout <<"二回目オプティカルフロー["<<optkaisu<<"]"<< std::endl;// 特徴点追跡(二回目のフレーム)
		vector<uchar> status;//特徴点の数
		vector<float> err;

		cv::calcOpticalFlowPyrLK(image_prev, image_curr, points_prev_limited, points_curr_limited, status, err);//オプティカルフロー
		std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

		// 追跡できなかった特徴点をリストから削除する
		int i, k,n;
		for (i = k = n =0; i < status.size(); i++){
			std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
			//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
			//statusが0以外の時値を更新する
			if (status[i] != 0) {	
			points_prev_limited[k]   = points_prev_limited[i];
			points_curr_limited[k++] = points_curr_limited[i];
			std::cout <<"status["<<i<<"]がゼロ以外の時"<< std::endl;//0以外の時は変なマークがでる
			}
		}
		std::cout <<"k="<<k<< std::endl;
		points_curr_limited.resize(k);//ここでkの個数でリサイズされている
		points_prev_limited.resize(k);
		
		if(status.size()<3){reset = true;}//特徴点が100個以下になったら再び特徴点を検出する
    
	}//else文ここまで-------------------------------------------------------------------------------------------

  //配列定義---------------------------------------------------------------------------------------------------
  float Zs[points_curr_limited.size()];//距離データ定義
  cv::Mat_<double> Screenp[points_curr_limited.size()] = cv::Mat_<double>(3, 1);//点Pの画像座標系（u1,v1,1)T※斉次化
  cv::Mat_<double> Camerap[points_curr_limited.size()] = cv::Mat_<double>(3, 1);//点Pのカメラ座標系（xc,yc,zc)T
  cv::Mat_<double> Worldp[points_curr_limited.size()] = cv::Mat_<double>(3, 1);//点Pの世界座標系（xw,yw,zw)T
  cv::Mat_<double> CameraP= cv::Mat_<double>(points_curr_limited.size(), 4);//すべての点のカメラ座標(斉次化)
  cv::Mat_<double> WorldP= cv::Mat_<double>(points_curr_limited.size(), 3);//すべての点の世界座標
  cv::Mat_<double> Rtc= cv::Mat_<double>(3, 4);//外部パラメータ(値は転置されているのに注意)
  cv::Mat_<double> Tc= cv::Mat_<double>(3, 1);//外部パラメータ(並進ベクトル) 
  cv::Mat_<double> Rc= cv::Mat_<double>(3, 3);//外部パラメータ(回転行列) 
  double optx,opty,pointnorm[points_curr_limited.size()],normgoukei=0,normheikin=0;//オプティカルフローノイズ問題対策
  double optcurrx,optcurry,optprevx,optprevy;
  int NotZeroL=-1;
  
	// 特徴点を丸で描く-------------------------------------------------------------------------------------------
	for (int i = 0; i < points_curr_limited.size(); i++) {
    std::cout << "特徴点描写for文["<<i<<"]"<< std::endl;
		cv::Scalar c(0, 255, 0);//緑
    //pointnorm[i]=cv::norm(points_prev[i] - points_curr[i]);//オプティカルフローノイズ問題対策
    //normgoukei=normgoukei+pointnorm[i];
    //std::cout <<"cv::norm(points_prev["<<i<<"] - points_curr["<<i<<"])="<<pointnorm[i]<< std::endl;//オプティカルフローの長さ

		if (cv::norm(points_prev_limited[i] - points_curr_limited[i]) > 0.5) {
			c = cv::Scalar(0, 100, 255);}//今の特徴点と一つ前の特徴点との差が0.5以上の時オレンジ
		//if(i==0){cv::circle(img_dst, points_curr_limited[i], 8, Scalar(0,255,255), -1, cv::LINE_AA);}
		//else{
			cv::circle(img_dst, points_curr_limited[i], 4, c, -1, cv::LINE_AA);//今の座標情報
			cv::circle(img_dst, points_prev_limited[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
			cv::line(img_dst,cv::Point(points_curr_limited[i]),cv::Point(points_prev_limited[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
      cv::line(img_dst,cv::Point(points_curr_limited[i].x,points_prev_limited[i].y),cv::Point(points_prev_limited[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
      cv::line(img_dst,cv::Point(points_prev_limited[i].x,points_curr_limited[i].y),cv::Point(points_prev_limited[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する
      
      //cv::rectangle(img_dst, cv::Rect(points_curr_limited[i].x,points_curr_limited[i].y, 30+abs(points_curr_limited[i].x-points_prev_limited[i].x)*5, 30+abs(points_curr_limited[i].y-points_prev_limited[i].y)*5), cv::Scalar(0, 0, 255), 2);
      

      if((points_curr_limited[i].x-points_prev_limited[i].x)>(points_curr_limited[i].y-points_prev_limited[i].y)){
        cv::arrowedLine(img_dst2,points_curr_limited[i],points_prev_limited[i],cv::Scalar(255,255,0),1,8,0,1.0);//矢印の描写(水色)
      }
      else{
        cv::arrowedLine(img_dst2,points_curr_limited[i],points_prev_limited[i],cv::Scalar(0,0,255),1,8,0,1.0);//矢印の描写(赤)
      }
		//}
    //テンプレートマッチング----------------------------------------------------------------------------------------
    if(kaisu >= 1){
      //if(points_curr[i].y<480-30&&points_curr[i].x<640-30){
      std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
      //cv::Rect roi1(cv::Point(points_prev[i].x,points_curr[i].y), cv::Size(10, 10));
      FeaturePoint[i].copyTo(img_template1); // 切り出し画像(FeaturePoint[i]→img_template1)

      cv::Mat img_minmax1;
      // テンプレートマッチング
      std::cout << "templ1" << std::endl;
      cv::matchTemplate(image, img_template1, img_minmax1, cv::TM_CCORR_NORMED);
      std::cout << "templ2" << std::endl;

      // 最大位置に矩形描画
      cv::Point min_pt1, max_pt1;
      double min_val1, max_val1;

      cv::minMaxLoc(img_minmax1, &min_val1, &max_val1, &min_pt1, &max_pt1);
      std::cout << "min_val1(白)["<<i<<"]=" << min_val1 << std::endl;//一致度が上がると値が大きくなる
      std::cout << "max_val1(白)["<<i<<"]=" << max_val1 << std::endl;
      if(max_val1>0.9){
      cv::rectangle(img_dst, cv::Rect(max_pt1.x, max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(kaisu, 255, 255), 3);}//白
      //else{cv::rectangle(img_dst, cv::Rect(max_pt1.x, max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);}
      //}

    }

    //テンプレートマッチング2----------------------------------------------------------------------------------------
    if(req >= 1){
      std::cout << "テンプレートマッチングプログラム2["<<i<<"]"<< std::endl;
      Prev_FeaturePoint[i].copyTo(img_template2); // 切り出し画像(FeaturePoint[i]→img_template1)

      cv::Mat img_minmax2;
      // テンプレートマッチング
      std::cout << "templ1" << std::endl;
      cv::matchTemplate(image, img_template2, img_minmax2, cv::TM_CCORR_NORMED);
      std::cout << "templ2" << std::endl;

      // 最大位置に矩形描画
      cv::Point min_pt2, max_pt2;
      double min_val2, max_val2;

      cv::minMaxLoc(img_minmax2, &min_val2, &max_val2, &min_pt2, &max_pt2);
      std::cout << "min_val2(紫)["<<i<<"]=" << min_val2 << std::endl;//一致度が上がると値が大きくなる
      std::cout << "max_val2(紫)["<<i<<"]=" << max_val2 << std::endl;
      if(max_val2>0.9){
      cv::rectangle(img_dst, cv::Rect(max_pt2.x, max_pt2.y, img_template2.cols, img_template2.rows), cv::Scalar(255, 0, 255), 2);}//白
      //else{cv::rectangle(img_dst, cv::Rect(max_pt1.x, max_pt1.y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 255), 3);}
      //}
    }
    
   //----------------------------------------------------------------------------------------------------------------------

    std::cout <<"特徴点の画像座標(curr)["<<i<<"]="<<points_curr_limited[i]<< std::endl;//特徴点の座標
		std::cout <<"特徴点の画像座標(prev)["<<i<<"]="<<points_prev_limited[i]<< std::endl;//特徴点の座標

    if(kaisu%50==0){img_dst2 = cv::Scalar(0,0,0);}//100回で描写記録画像をリセット

    /*Zs[i]= img_depth3.at<float>(points_curr[i]);//点の三次元距離データ取得
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

    //CameraP(NotZeroL,0)=Camerap[NotZeroL](0,0);//特徴点Pのカメラ座標を一つの行列にまとめる(斉次化)
    //CameraP(NotZeroL,1)=Camerap[NotZeroL](1,0);
    //CameraP(NotZeroL,2)=Camerap[NotZeroL](2,0);
    //CameraP(NotZeroL,3)=1;

    if(kaisu%50==0){img_dst2 = cv::Scalar(0,0,0);}//100回で描写記録画像をリセット
    std::cout <<"元データ個数NotZeroL="<<points_curr.size()<< std::endl;
    std::cout <<"Depthノイズを除いたデータ個数NotZeroL="<<NotZeroL<< std::endl;
    cv::Mat_<double> CameraP= cv::Mat_<double>(NotZeroL, 4);//すべての点のカメラ座標(斉次化)
    cv::Mat_<double> WorldP= cv::Mat_<double>(NotZeroL, 4);//すべての点の世界座標
    for (int i = 0; i < NotZeroL; i++) {
        CameraP(i,0)=Camerap[i](0,0);//特徴点Pのカメラ座標を一つの行列にまとめる(斉次化)
        CameraP(i,1)=Camerap[i](1,0);
        CameraP(i,2)=Camerap[i](2,0);
        CameraP(i,3)=1;
	    }*/
  }
  //特徴点描写for文終わり--------------------------------------------------------------------------------------------------

  //オプティカルフローノイズ問題についてーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
  /*for (int i = 0; i < points_curr.size(); i++) {
      std::cout <<"オプティカルフローX["<<i<<"]="<<optx<< std::endl;//オプティカルフローの差
      //std::cout <<"optcurrx["<<i<<"]="<<optcurrx<< std::endl;
    }*/

  /*normheikin=normgoukei/points_curr.size();
  std::cout <<"平均距離="<<normheikin<< std::endl;//特徴点の座標

  for (int i = 0; i < points_curr.size(); i++) {
  pointnorm[i]=cv::norm(points_prev[i] - points_curr[i]);
  std::cout <<"cv::norm(points_prev["<<i<<"] - points_curr["<<i<<"])="<<pointnorm[i]<< std::endl;//特徴点の座標
  if(pointnorm[i]>normheikin){
    cv::circle(img_dst, points_prev[i], 3, Scalar(255,0,0), -1, cv::LINE_AA);//一つ前の画像の座
  }
  }*/
  //-------------------------------------------------------------------------------------------------------------------    

  /*std::cout << "すべての点のカメラ座標:CameraP=\n" << CameraP << std::endl;
  //if(kaisu==0){//開始時は世界座標とカメラ座標の原点は一致しているので世界座標から見た点Pの座標とカメラ座標から見た点Pの座標は一致
  std::cout << "初回動作=" << kaisu << std::endl;
  Rtc=((CameraP.t()*CameraP).inv())*(CameraP.t()*CameraP);
  std::cout << "外部パラメータ:Rtc=\n" << Rtc << std::endl;

  Rc(0,0)=Rtc(0,0),Rc(0,1)=Rtc(0,1),Rc(0,2)=Rtc(0,2);
  Rc(1,0)=Rtc(1,0),Rc(1,1)=Rtc(1,1),Rc(1,2)=Rtc(1,2);
  Rc(2,0)=Rtc(2,0),Rc(2,1)=Rtc(2,1),Rc(2,2)=Rtc(2,2);
  Tc(0,0)=Rtc(3,0),Tc(1,0)=Rtc(3,1),Tc(2,0)=Rtc(3,2);

  std::cout << "外部パラメータ(回転行列):Rc=\n" << Rc << std::endl;//転置行列
  std::cout << "外部パラメータ(並進ベクトル):Tc=\n" << Tc << std::endl;
  for (int i = 0; i < NotZeroL; i++) {
    //Worldp[i]=Rc*(Camerap[i]-Tc);
    Worldp[i]=Rc.inv()*(Camerap[i]+Tc);
    std::cout << "特徴点の世界座標:Worldp["<<i<<"]=\n" << Worldp[i] << std::endl; 
    WorldP(i,0)=Worldp[i](0,0);//特徴点Pのカメラ座標を一つの行列にまとめる(斉次化)
    WorldP(i,1)=Worldp[i](1,0);
    WorldP(i,2)=Worldp[i](2,0);
    WorldP(i,3)=1;
  }
  std::cout << "すべての点の世界座標:WorldP=\n" << WorldP << std::endl;*/
    

  //------------------------------------------------------------------------------------------
  // (4)結果の表示(Vizを使用する）
  /*cv::Matx33d K1 = cv::Matx33d(1.93/0.003, 0, 326.15115000485503,0, 1.93/0.003,244.02271379203739,0, 0, 1);
  //...Windowを生成
  cv::viz::Viz3d window("Coordinate Frame");
  window.setWindowSize(cv::Size(800, 600));
  window.setBackgroundColor(); // 指定しないと背景は黒

  //...推定された特徴点の３次元位置をセット
  std::vector<cv::Vec3f> point_cloud_est;
  for (int i = 0; i < NotZeroL; i++) {
    point_cloud_est.push_back(cv::Vec3f(Worldp[i]));}

  //...カメラ位置のセット
  std::vector<cv::Affine3d> path;
  for (int i = 0; i < NotZeroL; i++){
    path.push_back(cv::Affine3d(Rc.inv(), Tc));}

  //...３次元座標（点での）の表示
  cv::viz::WCloud cloud_widget(point_cloud_est, cv::viz::Color::green());
  window.showWidget("point_cloud", cloud_widget);

  //...カメラ位置の表示
  window.showWidget("cameras_frames_and_lines", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::green()));
  window.showWidget("cameras_frustums", cv::viz::WTrajectoryFrustums(path, K1, 0.1, cv::viz::Color::yellow()));
  window.setViewerPose(path[0]);*/
  //---------------------------------------------------------------------------------------------------------------
     
  cv::imshow("win_dst", img_dst);
  cv::imshow("win_dst2", img_dst2);
  cv::imshow("win_curr", image_curr);//今の画像
  std::cout << "win1" << std::endl;
  cv::imshow("win_FeaturePoint", FeaturePoint[1]);//黄色の特徴点を中心としたクロップ画像
  std::cout << "win2" << std::endl;
  cv::imshow("win_img_1", img_1);//今のテンプレート表示画像
  std::cout << "win3" << std::endl;
  if(req>=1){cv::imshow("win_Prev_img_1", Prev_img_1);}//一つ前のテンプレート表示画像

  if (kaisu >= 1) {//初回は直前の画像がないため考慮
  cv::imshow("win_prev", image_prev);//一つ前の画像像
  std::cout <<"でてるよ"<< std::endl;//特徴点の座標
  }

  //---------------------------------------------------------------------------------------------

  int key = cv::waitKey(30);
  if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
  
  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
  cv::swap(points_curr_limited, points_prev_limited);// points_curr を points_prev に移す（交換する）
  kaisu=kaisu+1;//行列初期設定用変数
  if(reset == true){
    cv::swap(FeaturePoint, Prev_FeaturePoint);//今のテンプレート画像をPrev_FeaturePointに保存する(再度特徴点を検出時)
    cv::swap(img_1, Prev_img_1);
    req=req+1;
    std::cout <<"swap(FeaturePoint, Prev_FeaturePoint----------------------------------------------------------------------------"<< std::endl;
    std::cout <<"req="<<req<< std::endl;
    }

  cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数-----------------------------------------------------------------------------------
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

	ros::spin();//トピック更新待機
			
	return 0;
}