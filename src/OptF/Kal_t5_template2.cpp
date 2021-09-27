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
std::string win_img_1 = "img_1";//カメラ画像
std::string win_RoiKalman = "RoiKalman";//カルマンフィルタで予測したマッチング予測範囲クロップ画像

using namespace std;
using namespace cv;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)
bool kalkal = false;//カルマンフィルタのりサイズ用
bool swap_on = true;//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)
bool roiroi = false;//カルマンクロップ用
bool kal3 = false;//3回目マッチング結果カルマン

cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1;
cv::Mat img_template1,img_template2[10];//テンプレート画像
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;
vector<cv::Point2f> mappoint;
vector<cv::Point2f> points_prev_limited, points_curr_limited;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
vector<cv::Point2f> Template_Points_Prev,Template_Points_Curr;//テンプレートの中心座標

cv::Mat_<double>World0,WorldC,WorldC0;//世界座標系設定
cv::Mat_<double>Camera0;//カメラ座標系設定
cv::Mat_<double>WorldP;//特徴点の世界座標系設定

//カメラの内部パラメータ情報
cv::Mat_<double>K;//内部パラメータ（詳しくはM2 6月研究ノート)
cv::Mat_<double>K_;//内部パラメータの逆行列

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
cv::Mat FeaturePoint[100];//特徴点周囲の切り取った画像

int kaisu,optkaisu;//行列初期設定用変数

int karuman=5;//カルマンフィルタの個数=特徴点の個数
cv::KalmanFilter KF(4, karuman);//(状態ベクトルの次元数,観測ベクトルの次元数)
cv::KalmanFilter KF1(4, karuman);//カルマンフィルタデータ保持用

cv::Point min_pt1[10], max_pt1[10] ,min_pt2, max_pt2, min_pt3, max_pt3;//テンプレートマッチング用変数
double min_val1[10], max_val1[10], min_val2[10], max_val2[10], min_val3[10], max_val3[10];//テンプレートマッチング結果変数

double templateX[10],templateY[10],templateRows[10],templateCols[10];//カルマンフィルタで予測したマッチング予測範囲
cv::Mat RoiKalman[20],RoiKalman2[20];//カルマンフィルタで予測したマッチング予測範囲クロップ画像
cv::Point roi_min_pt1[10], roi_max_pt1[10];
double roi_min_val1[10], roi_max_val1[10];


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
	int req;

  //Depth修正----------------------------------------------------------------------------------------------------
    img_depth2 = depthimage.clone();//depthの画像をコピーする
    //画像クロップ(中距離でほぼ一致)
    cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
    cv::Mat img_dstdepth = depthimage(roi); // 切り出し画像
    resize(img_dstdepth, img_depth3,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
    img_depth4 = img_depth3.clone();//depth3の画像をコピーする
    cv::imshow(win_depth4, img_depth4);//クロップされたdepth画像

  //行列初期設定-------------------------------------------------------------------------------------------------------------------
  /*if(kaisu==0){
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
  std::cout << "内部パラメータ: K=\n" << K << std::endl;*/

  //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
  float yosokuX[karuman],yosokuY[karuman],yosoku_haniX[karuman],yosoku_haniY[karuman];//カルマンフィルタ出力変数
  if(kaisu==0){
  KF.statePre = cv::Mat_<float>::zeros(4, karuman); //状態の推定値(x'(k))
  KF.statePost = cv::Mat_<float>::zeros(4, karuman);// 更新された状態の推定値 (x(k))
  // 運動モデル(システムの時間遷移に関する線形モデル)(A)
  KF.transitionMatrix = (cv::Mat_<float>(4, 4) << // 等速直線運動（速度利用あり）
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1);
  
  KF.measurementMatrix = (cv::Mat_<float>(2, 4) << //観測行列 (H)
    1, 0, 0, 0,
    0, 1, 0, 0);

  KF.measurementNoiseCov = (cv::Mat_<float>(2, 2) << //観測ノイズの共分散行列 (R)
    0.1, 0,
    0, 0.1);

  KF.gain = cv::Mat_<float>::zeros(4, 2);//カルマンゲイン(K)
  setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-1));//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
  setIdentity(KF.errorCovPost, cv::Scalar::all(1e-1));//前回更新された誤差共分散(P'(k))

  //カルマンの一つ前のデータ保持用
  KF1.statePre = cv::Mat_<float>::zeros(4, karuman); //状態の推定値(x'(k))
  KF1.statePost = cv::Mat_<float>::zeros(4, karuman);// 更新された状態の推定値 (x(k))
  KF1.transitionMatrix = cv::Mat_<float>::zeros(4, 4); // 運動モデル(システムの時間遷移に関する線形モデル)(A)
  KF1.measurementMatrix = cv::Mat_<float>::zeros(2,4);//観測行列 (H)
  KF1.measurementNoiseCov = cv::Mat_<float>::zeros(2,2);//観測ノイズの共分散行列 (R)
  KF1.gain = cv::Mat_<float>::zeros(4, 2);//カルマンゲイン(K)
  KF1.processNoiseCov= cv::Mat_<float>::zeros(4, 4);//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
  KF1.errorCovPre= cv::Mat_<float>::zeros(4, 4);//現在の誤差共分散(P'(k))
  KF1.errorCovPost= cv::Mat_<float>::zeros(4, 4);//更新された誤差共分散(P(k))

  }

  //画像処理------------------------------------------------------------------------------------------------------------------------
  image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(img_dst);//ここ間違えていたので注意
  if(kaisu==0){
    img_dst2 = image.clone();//オプティカルフローの描写画面の用意
    img_dst2 = cv::Scalar(0,0,0);
  }
	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化

	//初回検出プログラム-----------------------------------------------------------------------------------------------------
  if (reset == true) {
    swap_on=false;
    optkaisu=0;//オプティカルフロー回数リセット
		std::cout <<"初回検出プログラム["<<optkaisu<<"]"<< std::endl;//最初のフレーム
    image.copyTo(img_1);
		// 特徴点検出(グレースケール画像から特徴点検出)
    //cv::goodFeaturesToTrack(今画像, 前画像, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, karuman, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    points_curr_limited = points_curr;//points_curr_limitedの定義
    points_prev_limited = points_curr;//points_prev_limitedの定義
    Template_Points_Curr = points_curr;//Template_Points_Currの定義
    Template_Points_Prev = points_curr;//Template_Points_Prevの定義
    int limited=0;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)

		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      cv::circle(img_dst2, points_curr[i], 3, Scalar(0,255,255), -1, cv::LINE_AA);//追跡記録に描写
      cv::rectangle(img_dst, cv::Point(points_curr[i].x-15,points_curr[i].y+15), 
      cv::Point(points_curr[i].x+15,points_curr[i].y-15), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);//四角形を描写(青)
    
    //特徴点を検出したら特徴点の周りの画像をクロップして保存→その後保存した画像でBoWを行う予定
    //テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
    if(15<points_curr[i].y&&points_curr[i].y<480-15){
      if(15<points_curr[i].x&&points_curr[i].x<640-15){

      points_curr_limited[limited] = points_curr[i];//新しいpoint_vecterに書き換える(外枠15Pixelの特徴点を検出しない

      //テンプレート作成----------------------------------------------------------------------------------------
      std::cout <<"特徴点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
      //cv::Rect roi2(cv::Point(points_curr[i].x-15,points_curr[i].y-15), cv::Size(30, 30));//特徴点を中心とした15☓15pixelの画像を切り取る
      //FeaturePoint[i] = image(roi2); // 切り出し画像

      std::cout <<"特徴点画像クロッププログラムlimited["<<limited<<"]"<< std::endl;
      std::cout <<"1特徴点の画像座標(curr_if)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標(範囲制限前)
      std::cout <<"1特徴点の画像座標(limited)["<<limited<<"]="<<points_curr_limited[limited]<< std::endl;//特徴点の座標(範囲制限後)

      cv::rectangle(img_1, cv::Point(points_curr_limited[i].x-15,points_curr_limited[i].y+15), 
      cv::Point(points_curr_limited[i].x+15,points_curr_limited[i].y-15), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)

      cv::Rect roi2(cv::Point(points_curr[i].x-15,points_curr[i].y-15), cv::Size(30, 30));//特徴点を中心とした15☓15pixelの画像を切り取る
      FeaturePoint[limited] = image(roi2); // 切り出し画像

      std::cout <<"特徴点の画像座標(curr_if)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標(範囲制限前)
      std::cout <<"特徴点の画像座標(limited)["<<limited<<"]="<<points_curr_limited[limited]<< std::endl;//特徴点の座標(範囲制限後)
      limited=limited+1;//範囲制限後の特徴点の個数
      }}
		}

    points_prev_limited = points_curr_limited;
    points_curr_limited.resize(limited);
    points_prev_limited.resize(limited);
    std::cout <<"制限した特徴点の個数(limited.size)="<<points_curr_limited.size()<< std::endl;
    //Template_Points_Prev = points_curr_limited.clone();//テンプレートの中心座標
    //points_curr_limited.copyTo(Template_Points_Prev);
    
    Template_Points_Prev = points_curr_limited;//テンプレートの中心座標
    Template_Points_Prev.resize(limited);
    std::cout <<"制限した特徴点の個数(Template_Points_Prev.size)="<<Template_Points_Prev.size()<< std::endl;

    KF.statePre = cv::Mat_<float>::zeros(4, Template_Points_Prev.size()); //特徴点の数でリサイズ
    KF.statePost = cv::Mat_<float>::zeros(4, Template_Points_Prev.size());//特徴点の数でリサイズ

    reset = false;//if文切り替え
  }//------------------------------------------------------------------------------------------------------------------------ 

  //二回目動作プログラム--------------------------------------------------------------------------------------------
	else {
    swap_on = true;
    std::cout << "二回目動作プログラム"<< std::endl;
    int templP=0;//マッチングの中心座標個数
    std::cout <<"制限した特徴点の個数(Template_Points_Prev.size)="<<Template_Points_Prev.size()<< std::endl;
    
    //マッチング範囲限定クロップ------------------------------------------------------------------------------------
    if(roiroi==true){
      std::cout << "マッチング範囲限定クロッププログラム"<< std::endl;
      for (int i = 0; i < Template_Points_Prev.size(); i++) {
        //カルマンフィルタの予測範囲が画像外に行かないように処理(一回ではうまく除去できないので２回行う)
        if(templateX[i]+templateCols[i]>=640){templateCols[i]=640-templateX[i];}
        if(templateY[i]+templateRows[i]>=480){templateRows[i]=480-templateY[i];}
        if(templateX[i]<=0){templateX[i]=0;}
        if(templateY[i]<=0){templateY[i]=0;}
        cv::Rect RoiKal(cv::Point(templateX[i],templateY[i]), cv::Size(templateCols[i], templateRows[i]));//特徴点を中心とした15☓15pixelの画像を切り取る
        RoiKalman[i] = image(RoiKal); // 切り出し画像
        RoiKalman[i].copyTo(RoiKalman2[i]);
        cv::rectangle(img_dst, cv::Rect(templateX[i], templateY[i], templateCols[i],templateRows[i]),cv::Scalar(255, 0, 100), 2);//マッチング予測範囲(赤枠)
      
    
      //マッチング範囲限定マッチング--------------------------------------------------------------
        FeaturePoint[i].copyTo(img_template2[i]); // 切り出し画像
        cv::Mat img_roimax1[Template_Points_Prev.size()];
        cv::matchTemplate(RoiKalman[i], img_template2[i], img_roimax1[i], cv::TM_CCORR_NORMED);// テンプレートマッチング
        cv::minMaxLoc(img_roimax1[i], &roi_min_val1[i], &roi_max_val1[i], &roi_min_pt1[i], &roi_max_pt1[i]);
        //std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が大きくなる
        //std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
        if(max_val1[i]>0.9){
          cv::rectangle(RoiKalman2[i], cv::Rect(roi_max_pt1[i].x, roi_max_pt1[i].y, img_template2[i].cols, img_template2[i].rows), cv::Scalar(0, 255, 0), 3);
          cv::circle(RoiKalman2[i], cv::Point(roi_max_pt1[i].x+img_template2[i].cols/2, roi_max_pt1[i].y+img_template2[i].rows/2), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
          cv::rectangle(img_dst, cv::Rect(roi_max_pt1[i].x+templateX[i], roi_max_pt1[i].y+templateY[i], img_template2[i].cols, img_template2[i].rows), cv::Scalar(0, 255, 0), 2);
          cv::circle(img_dst, cv::Point(roi_max_pt1[i].x+(img_template2[i].cols/2)+templateX[i], roi_max_pt1[i].y+(img_template2[i].rows/2)+templateY[i]), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
        }
      }
      cv::imshow("win_RoiKalman", RoiKalman2[0]);//カルマンフィルタで予測したマッチング予測範囲クロップ画像
    }

    if(roiroi==false){
      for (int i = 0; i < Template_Points_Prev.size(); i++) {
       //テンプレートマッチング(二回目)----------------------------------------------------------------------------------------
          std::cout << "テンプレートマッチングプログラム(二回目）["<<i<<"]"<< std::endl;
          FeaturePoint[i].copyTo(img_template1); // 切り出し画像

          cv::Mat img_minmax1;
          cv::matchTemplate(image, img_template1, img_minmax1, cv::TM_CCORR_NORMED);// テンプレートマッチング
          cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);

          //std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が大きくなる
          //std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
          if(max_val1[i]>0.9){
            cv::rectangle(img_dst, cv::Rect(max_pt1[i].x, max_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 255, 255), 3);//白枠
            cv::circle(img_dst, cv::Point(max_pt1[i].x+img_template1.cols/2, max_pt1[i].y+img_template1.rows/2), 5, cv::Scalar(255, 255, 255), -1);//テンプレートの中心座標
          }
            //std::cout << "マッチング座標max_pt1["<<i<<"]=" << max_pt1[i] << std::endl;
            Template_Points_Curr[i].x=max_pt1[i].x+img_template1.cols/2;
            Template_Points_Curr[i].y=max_pt1[i].y+img_template1.rows/2;
            templP=templP+1;//マッチングの中心座標個数

            std::cout <<"マッチングの中心座標<Template_Points_Curr["<<i<<"]="<<Template_Points_Curr[i]<< std::endl;
            std::cout <<"一つ前の座標Template_Points_Prev["<<i<<"]="<<Template_Points_Prev[i]<< std::endl;

      }//テンプレートマッチング終わり------------------------------------------------------------------------
    }
    //三回目動作プログラム----------------------------------------------------------------------------------------
    else{
      for (int i = 0; i < Template_Points_Prev.size(); i++) {
        std::cout << "テンプレートマッチングプログラム(三回目）["<<i<<"]"<< std::endl;
         Template_Points_Curr[i].x=roi_max_pt1[i].x+(img_template2[i].cols/2)+templateX[i];
         Template_Points_Curr[i].y=roi_max_pt1[i].y+(img_template2[i].rows/2)+templateY[i];
         templP=templP+1;//マッチングの中心座標個数
         std::cout <<"マッチングの中心座標<Template_Points_Curr["<<i<<"]="<<Template_Points_Curr[i]<< std::endl;
         std::cout <<"一つ前の座標Template_Points_Prev["<<i<<"]="<<Template_Points_Prev[i]<< std::endl;
      }
      roiroi= false;
    }


    std::cout <<"リサイズ前マッチングの中心座標<Template_Points_Curr=\n"<<Template_Points_Curr<< std::endl;
    std::cout <<"リサイズ前一つ前の座標Template_Points_Prev=\n"<<Template_Points_Prev<< std::endl;
    Template_Points_Curr.resize(templP);//配列リサイズ
    std::cout <<"マッチングの中心座標の個数="<<Template_Points_Curr.size()<< std::endl;

    std::cout <<"リサイズ後マッチングの中心座標<Template_Points_Curr=\n"<<Template_Points_Curr<< std::endl;
    std::cout <<"リサイズ後一つ前の座標Template_Points_Prev=\n"<<Template_Points_Prev<< std::endl;

    //オプティカルフロー-------------------------------------------------------------------------------------
    optkaisu=optkaisu+1;
		std::cout <<"オプティカルフロー["<<optkaisu<<"]"<< std::endl;// 特徴点追跡(二回目のフレーム)
		vector<uchar> status;//特徴点の数
		vector<float> err;

		//cv::calcOpticalFlowPyrLK(image_prev, image_curr, Template_Points_Prev, Template_Points_Curr, status, err);//オプティカルフロー
    cv::calcOpticalFlowPyrLK(image_prev, image_curr, Template_Points_Curr, Template_Points_Prev, status, err);//オプティカルフロー
		std::cout <<"status.size()="<<status.size()<< std::endl;//特徴点の個数

		// 追跡できなかった特徴点をリストから削除する
		int i, k,n,knot[karuman],j=0;
		req=1;
		for (i = k = n =0; i < status.size(); i++){
			std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
			//ここで検索出来なかったものを消し探索の範囲を狭めている(statusが0の時)
			//statusが0以外の時値を更新する
			if (status[i] != 0) {	
			  Template_Points_Prev[k]   = Template_Points_Prev[i];
			  Template_Points_Curr[k++] = Template_Points_Curr[i];
			  std::cout <<"status["<<i<<"]がゼロ以外の時"<< std::endl;//0以外の時は変なマークがでる
			}
      else{
        j=j+1;//何回0が来るのか
        kalkal = true;//カルマン再定義
        knot[j]=i;//何番目で0になるのか
        std::cout <<"status["<<i<<"]がゼロの時(knot[j="<<j<<"]="<<knot[j]<< std::endl;
        std::cout <<"fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"<< std::endl;
        }
    }
		std::cout <<"k="<<k<< std::endl;
		Template_Points_Curr.resize(k);//ここでkの個数でリサイズされている
		Template_Points_Prev.resize(k);
    std::cout <<"(オプティカルフロー後）Template_Points_Curr.size()="<<Template_Points_Curr.size()<< std::endl;
    std::cout <<"(オプティカルフロー後）Template_Points_Prev.size()="<<Template_Points_Prev.size()<< std::endl;
		
		if(status.size()<3||k<2){//特徴点が100個以下になったら再び特徴点を検出する
      reset = true;
      kalkal = false;
      std::cout <<" 特徴点再検出リセット"<<std::endl;}

    else{
      //カルマンフィルタのリサイズ処理-------------------------------------------------------------------------------------------
      //リサイズが必要なのは状態方程式のみ。それ以外は一つ前のデータをコピー
      if(kalkal == true){
           std::cout <<"カルマンフィルタのリサイズj="<<j<<"---------------------------------------------------------------"<< std::endl;
           std::cout <<"リサイズ前_KF.statePre=\n"<< KF.statePre<<std::endl;
           std::cout <<"リサイズ前_KF.statePost=\n"<< KF.statePost<<std::endl;
           std::cout <<"status.size()="<< status.size()<<std::endl;

            KF.statePre = cv::Mat_<float>::zeros(4, status.size()-j); //状態の推定値(x'(k))
            KF.statePost = cv::Mat_<float>::zeros(4, status.size()-j);// 更新された状態の推定値 (x(k))

            for (i = k = n =0; i < status.size(); i++){
		  	      std::cout <<"status["<<i<<"]="<<status[i]<< std::endl;//0以外の時は変なマークがでる
		  	      //statusが0以外の時値を更新する
		  	      if (status[i] != 0) {	
                KF1.statePre.col(i).copyTo(KF.statePre.col(k));
                KF1.statePost.col(i).copyTo(KF.statePost.col(k++));
              }
            }

            KF1.transitionMatrix.copyTo(KF.transitionMatrix);// 運動モデル(システムの時間遷移に関する線形モデル)(A)
            KF1.measurementMatrix.copyTo(KF.measurementMatrix);//観測行列 (H)
            KF1.measurementNoiseCov.copyTo(KF.measurementNoiseCov);//観測ノイズの共分散行列 (R)
            KF1.gain.copyTo(KF.gain);//カルマンゲイン(K)
            KF1.processNoiseCov.copyTo(KF.processNoiseCov);//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
            KF1.errorCovPre.copyTo(KF1.errorCovPre);//現在の誤差共分散(P'(k))
            KF1.errorCovPost.copyTo(KF1.errorCovPost);//更新された誤差共分散(P(k))

            KF1.statePre = cv::Mat_<float>::zeros(4, karuman-j); //状態の推定値(x'(k))
            KF1.statePost = cv::Mat_<float>::zeros(4, karuman-j);// 更新された状態の推定値 (x(k))

           std::cout <<"リサイズ後_KF.statePre=\n"<< KF.statePre<<std::endl;
           std::cout <<"リサイズ後_KF.statePost=\n"<< KF.statePost<<std::endl;
           kalkal = false;
      }

	    // 特徴点を丸で描く-------------------------------------------------------------------------------------------
	    for (int i = 0; i < Template_Points_Curr.size(); i++) {
        std::cout << "特徴点描写for文["<<i<<"]"<< std::endl;
        std::cout <<"OPT後マッチングの中心座標["<<i<<"]="<<Template_Points_Curr[i]<< std::endl;
        std::cout <<"OPT後マッチングの中心座標["<<i<<"].x="<<Template_Points_Curr[i].x<< std::endl;
        std::cout <<"OPT後マッチングの中心座標["<<i<<"].y="<<Template_Points_Curr[i].y<< std::endl;

		  	cv::circle(img_dst, Template_Points_Curr[i], 4, Scalar(0, 255, 0), -1, cv::LINE_AA);//今の座標情報
		  	cv::circle(img_dst, Template_Points_Prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
		  	cv::line(img_dst,cv::Point(Template_Points_Curr[i]),cv::Point(Template_Points_Prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する
        cv::line(img_dst,cv::Point(Template_Points_Curr[i].x,Template_Points_Prev[i].y),cv::Point(Template_Points_Prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
        cv::line(img_dst,cv::Point(Template_Points_Prev[i].x,Template_Points_Curr[i].y),cv::Point(Template_Points_Prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する

        if((Template_Points_Curr[i].x-Template_Points_Prev[i].x)>(Template_Points_Curr[i].y-Template_Points_Prev[i].y)){
          cv::arrowedLine(img_dst2,Template_Points_Curr[i],Template_Points_Prev[i],cv::Scalar(255,255,0),1,8,0,1.0);//矢印の描写
          cv::line(img_dst,cv::Point(Template_Points_Curr[i].x,Template_Points_Prev[i].y),cv::Point(Template_Points_Prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
        }
        else{
          cv::arrowedLine(img_dst2,Template_Points_Curr[i],Template_Points_Prev[i],cv::Scalar(0,0,255),1,8,0,1.0);//矢印の描写
          cv::line(img_dst,cv::Point(Template_Points_Curr[i].x,Template_Points_Prev[i].y),cv::Point(Template_Points_Prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する
        }
      }

      //カルマンフィルタ0----------------------------------------------------------------------------------------
      // 観測
      std::cout <<"カルマンフィルタプログラム["<<i<<"]\n"<<std::endl;//観測データ(z(k))
      int karuix=0,karuiy=0,karuj_X=0,karuj_Y=0,karuj_HX=0,karuj_HY=0;//変数定義

      cv::Mat measurement(2, Template_Points_Curr.size(), CV_32F);
      for(int karui=0;karui<Template_Points_Curr.size()*2;karui++){
        if(karui<Template_Points_Curr.size()){measurement.at<float>(karui) = Template_Points_Curr[karuix].x;
          std::cout <<"観測座標_pos["<<karui<<"].x="<<measurement.at<float>(karui)<<std::endl;
          karuix=karuix+1;}
        if(karui>=Template_Points_Curr.size()){measurement.at<float>(karui) = Template_Points_Curr[karuiy].y;
          std::cout <<"観測座標_pos["<<karui<<"].y="<<measurement.at<float>(karui)<<std::endl;
          karuiy=karuiy+1;}
      }
      


      std::cout <<"観測データ(z(k))_measurement=\n"<<measurement<<std::endl;//観測データ(z(k))

      std::cout <<" 現在の状態の推定値(x'(k))_KF.statePre=\n"<< KF.statePre<<std::endl;//現在の状態の推定値(x(k)):x(k)=A*x(k-1)+B*u(k)
      std::cout <<" 更新された状態の推定値(x(k))_KF.statePost=\n"<< KF.statePost<<std::endl;//更新された状態の推定値(x(k)):x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
      //std::cout <<"運動モデル(A)_KF.transitionMatrix=\n"<<KF.transitionMatrix<<std::endl;//運動モデル(システムの時間遷移に関する線形モデル)(A)
      //std::cout <<"観測行列 (H)_KF.measurementMatrix=\n"<<KF.measurementMatrix<<std::endl;//観測行列 (H)
      //std::cout <<"時間遷移に関するノイズの共分散行列 (Q)_KF.processNoiseCov=\n"<<KF.processNoiseCov<<std::endl;//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
      //std::cout <<"観測ノイズの共分散行列 (R)_KF.measurementNoiseCov=\n"<<KF.measurementNoiseCov<<std::endl;//観測ノイズの共分散行列 (R)
      std::cout <<"今の誤差共分散(P'(k))_KF.errorCovPre=\n"<<KF.errorCovPre<<std::endl;//今の誤差共分散(P'(k)):P'(k)=A*P(k-1)*At + Q
      //std::cout <<"カルマンゲイン(K)_KF.gain=\n"<<KF.gain<<std::endl;//カルマンゲイン(K):K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
      std::cout <<"更新された誤差共分散(P(k))_KF.errorCovPost=\n"<<KF.errorCovPost<<std::endl;//更新された誤差共分散(P(k)):P(k)=(I-K(k)*H)*P'(k)

      // 修正（カルマンフィルタ関数）ここで測定結果から予測された状態の更新を行っている
      cv::Mat correction = KF.correct(measurement);//観測値をカルマンフィルタに代入

      // 予測（カルマンフィルタ関数）
      cv::Mat prediction = KF.predict();//予測状態の計算
      std::cout <<"prediction=\n"<<prediction<<std::endl;

      for(int karui=0;karui<Template_Points_Curr.size()*4;karui++){
       if(karui<Template_Points_Curr.size()){
         std::cout <<"予測座標x["<<karui<<"]="<<prediction.at<float>(karui)<<std::endl;
         yosokuX[karuj_X]=prediction.at<float>(karui);
         karuj_X = karuj_X + 1;}

       if(Template_Points_Curr.size()<=karui && karui<Template_Points_Curr.size()*2){
         std::cout <<"予測座標y["<<karui<<"]="<< prediction.at<float>(karui)<<std::endl;
          yosokuY[karuj_Y]=prediction.at<float>(karui);
          karuj_Y = karuj_Y + 1;}  

       if(Template_Points_Curr.size()*2<=karui && karui<Template_Points_Curr.size()*3){
         std::cout <<"予測範囲x["<<karui<<"]="<<prediction.at<float>(karui)<<std::endl;
         yosoku_haniX[karuj_HX]=prediction.at<float>(karui);
         karuj_HX = karuj_HX + 1;}

       if(Template_Points_Curr.size()*3<=karui && karui<=Template_Points_Curr.size()*4){
         std::cout <<"予測範囲y["<<karui<<"]="<<prediction.at<float>(karui)<<std::endl;
         yosoku_haniY[karuj_HY]=prediction.at<float>(karui);
         karuj_HY = karuj_HY + 1;}
      }

      for(int i=0;i<Template_Points_Curr.size();i++){
        std::cout <<"karuj="<<i<<std::endl;//yの予測範囲
        std::cout <<"予測座標yosokuX["<<i<<"]="<<yosokuX[i]<<std::endl;
        std::cout <<"予測座標yosokuY["<<i<<"]="<< yosokuY[i]<<std::endl;
        std::cout <<"予測範囲yosoku_haniX["<<i<<"]="<<yosoku_haniX[i]<<std::endl;
        std::cout <<"予測範囲yosoku_haniY["<<i<<"]="<<yosoku_haniY[i]<<std::endl;
        cv::circle(img_dst, cv::Point(yosokuX[i], yosokuY[i]), 5, cv::Scalar(0, 255, 255), -1);//予測の点（黄色）
        //cv::ellipse(img_dst, cv::Point(yosokuX[i], yosokuY[i]),
        //cv::Size(abs(yosoku_haniX[i]*5), abs(yosoku_haniY[i])*5),0.0, 0.0, 360.0, cv::Scalar(0, 255, 255), 3);//予測の円（黄色

        templateX[i]=yosokuX[i]-img_template1.cols/2-abs(yosoku_haniX[i])*5;//カルマンフィルタで予測したマッチング予測範囲
        templateY[i]=yosokuY[i]-img_template1.rows/2-abs(yosoku_haniY[i])*5;
        templateCols[i]=img_template1.cols+abs(yosoku_haniX[i])*10;
        templateRows[i]=img_template1.rows+abs(yosoku_haniY[i])*10;

        //カルマンフィルタの予測範囲が画像外に行かないように処理
        if(templateX[i]+templateCols[i]>=640){templateCols[i]=640-templateX[i];}
        if(templateY[i]+templateRows[i]>=480){templateRows[i]=480-templateY[i];}
        if(templateX[i]<=0){templateX[i]=0;}
        if(templateY[i]<=0){templateY[i]=0;}
   

         cv::rectangle(img_dst, cv::Rect(templateX[i], templateY[i], templateCols[i],templateRows[i]),cv::Scalar(0, 0, 255), 2);//マッチング予測範囲(赤枠)
         std::cout <<"カルマン後のTemplate_Points_Curr["<<i<<"]="<<Template_Points_Curr[i]<< std::endl;
         std::cout <<"カルマン後のTemplate_Points_Prev["<<i<<"]="<<Template_Points_Prev[i]<< std::endl;        
      }  
      //カルマンフィルタの情報保持
      KF.statePre.copyTo(KF1.statePre);//状態の推定値(x'(k))
      KF.statePost.copyTo(KF1.statePost);// 更新された状態の推定値 (x(k))
      KF.transitionMatrix.copyTo(KF1.transitionMatrix);// 運動モデル(システムの時間遷移に関する線形モデル)(A)
      KF.measurementMatrix.copyTo(KF1.measurementMatrix);//観測行列 (H)
      KF.measurementNoiseCov.copyTo(KF1.measurementNoiseCov);//観測ノイズの共分散行列 (R)
      KF.gain.copyTo(KF1.gain);//カルマンゲイン(K)
      KF.processNoiseCov.copyTo(KF1.processNoiseCov);//プロセスノイズ（時間遷移に関するノイズ）の共分散行列 (Q)
      KF.errorCovPre.copyTo(KF1.errorCovPre);//現在の誤差共分散(P'(k))
      KF.errorCovPost.copyTo(KF1.errorCovPost);//更新された誤差共分散(P(k))

      roiroi=true;//カルマン予測範囲クロップon

       
        //std::cout <<"特徴点の画像座標(curr)["<<i<<"]="<<points_curr_limited[i]<< std::endl;//特徴点の座標
		    //std::cout <<"特徴点の画像座標(prev)["<<i<<"]="<<points_prev_limited[i]<< std::endl;//特徴点の座標

        if(kaisu%50==0){img_dst2 = cv::Scalar(0,0,0);}//100回で描写記録画像をリセット

    }//特徴点再検出リセット
  }//else文ここまで-------------------------------------------------------------------------------------------

      //配列定義---------------------------------------------------------------------------------------------------
    /*float Zs[points_curr_limited.size()];//距離データ定義
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
    int NotZeroL=-1;*/

     

  cv::imshow("win_curr", image_curr);//今の画像
  cv::imshow("win_FeaturePoint", FeaturePoint[0]);//黄色の特徴点を中心としたクロップ画像
  cv::imshow("win_img_1", img_1);//今の画像
  cv::imshow("win_dst", img_dst);
  cv::imshow("win_dst2", img_dst2);
  

  if (req == 1) {//初回は直前の画像がないため考慮
    cv::imshow("win_prev", image_prev);//一つ前の画像像
    std::cout <<"でてるよ"<< std::endl;//特徴点の座標
  }

  //---------------------------------------------------------------------------------------------

  int key = cv::waitKey(30);
  if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
  
  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
  if(swap_on ==true){cv::swap(Template_Points_Curr, Template_Points_Prev);}//初回プログラム実行時はswapしない(データの浅いコピーの影響を考慮)


  kaisu=kaisu+1;//行列初期設定用変数
  
  cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数-----------------------------------------------------------------------------------
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

	ros::spin();//トピック更新待機
			
	return 0;
}