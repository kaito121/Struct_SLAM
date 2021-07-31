//Depth調整バージョン(D435_test.cppを参照)
//rosのヘッダ
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
#include <Eigen/Dense>

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像（修正前）
std::string win_depth2 = "depth2";//深度画像（修正前）+FLD線
std::string win_depth3 = "depth3";//深度画像（修正後）
std::string win_depth4 = "depth4";//深度画像（修正後）+FLD線
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_line = "line";//FLDの線を表示
std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_line3 = "line3";//分類分けしたタテ、ヨコ、ナナメの線を表示
std::string win_line4 = "line4";//分類分けしたタテ、ヨコの線を表示

std::string win_dstP = "dstP";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
std::string win_FeaturePoint = "FeaturePoint";//特徴点を検出したら特徴点の周りの画像をクロップした画像
//std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像

using namespace std;
using namespace cv;
using Eigen::MatrixXd;


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
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1;
cv::Mat img_template1;
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;
vector<cv::Point2f> mappoint;
vector<cv::Point2f> points_prev_limited, points_curr_limited;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
vector<cv::Point2f> Template_Points_Prev,Template_Points_Curr;//テンプレートの中心座標
vector<cv::Point2f> points_prev_yoko, points_curr_yoko;
vector<cv::Point2f> points_prev_tate, points_curr_tate;

cv::Mat_<double>World0,WorldC,WorldC0;//世界座標系設定
cv::Mat_<double>Camera0;//カメラ座標系設定
cv::Mat_<double>WorldP;//特徴点の世界座標系設定

//カメラの内部パラメータ情報
cv::Mat_<double>K;//内部パラメータ（詳しくはM2 6月研究ノート)
cv::Mat_<double>K_;//内部パラメータの逆行列

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
cv::Mat FeaturePoint[100];//特徴点周囲の切り取った画像

int kaisu,optkaisu;//行列初期設定用変数

int karuman=4;//カルマンフィルタの個数=特徴点の個数
cv::KalmanFilter KF(4, karuman);//(状態ベクトルの次元数,観測ベクトルの次元数)

cv::Point min_pt1[10], max_pt1[10] ,min_pt2, max_pt2, min_pt3, max_pt3;//テンプレートマッチング用変数
double min_val1[10], max_val1[10], min_val2[10], max_val2[10], min_val3[10], max_val3[10];

//コールバック関数
void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
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

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

 //カルマンフィルタ初期設定---------------------------------------------------------------------------------------------------------
  /*float yosokuX[karuman],yosokuY[karuman],yosoku_haniX[karuman],yosoku_haniY[karuman];//カルマンフィルタ出力変数
  double templateX[karuman],templateY[karuman],templateRows[karuman],templateCols[karuman];//カルマンフィルタで予測したマッチング予測範囲
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
  }*/


//ここに処理項目
	cv::Mat img_src = image;
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_line,img_line2,img_line3,img_line4;
    cv::Mat img_depth2,img_depth3,img_depth4;
    double theta[1000],theta0,theta90;
    float dep,dep1[100],dep2[100];

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);
    img_line2 = img_src.clone();
    img_line2 = cv::Scalar(255,255,255);
    img_line3 = img_src.clone();
    img_line3 = cv::Scalar(255,255,255);
    img_line4 = img_src.clone();
    img_line4 = cv::Scalar(255,255,255);

    //Depth修正
    img_depth2 = img_depth.clone();//depthの画像をコピーする
    //画像クロップ(中距離でほぼ一致)
    cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
    cv::Mat img_dstdepth = img_depth(roi); // 切り出し画像
    resize(img_dstdepth, img_depth3,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
    img_depth4 = img_depth3.clone();//depth3の画像をコピーする
    

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    //cv::line(img_dst,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青水平
    //cv::line(img_dst,cv::Point(100,0),cv::Point(100,200),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑垂直

    //FLD変換
    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines.size(); i++){
       //cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
      // cv::line(img_line2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
      // cv::line(img_depth2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
       //cv::line(img_depth4,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }

    cv::cvtColor(img_line, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 200, 200);

    //確率的ハフ変換(元画像・lines)
    std::vector<cv::Vec4i> lines2;
    cv::HoughLinesP(img_edge, lines2, 1, CV_PI/180, 80,30,10);

    
    //FLD＋確率ハフ変換抽出線とY軸との角度を求める+三次元距離データの結合
    std::cout <<"並び替え前"<<std::endl;
    for(int i = 0; i < lines2.size(); i++){

     dep1[i]= img_depth3.at<float>(lines2[i][0],lines2[i][1]);//点の三次元距離データ取得
     dep2[i]= img_depth3.at<float>(lines2[i][2],lines2[i][3]);
     //cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

     if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)

      //cv::circle(img_line2,Point(lines2[i][0],lines2[i][1]),5,Scalar(255,0,0),-1);//青点
      //cv::circle(img_line2,Point(lines2[i][2],lines2[i][3]),5,Scalar(0,255,0),-1);//緑点

      std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<","<<dep1[i]/1000<<"[m])"<< std::endl;
      std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<","<<dep2[i]/1000<<"[m])"<< std::endl;

       //FLD抽出線のy軸との角度を求める
       theta[i]=M_PI-atan2((lines2[i][2]-lines2[i][0]),(lines2[i][3]-lines2[i][1]));
       std::cout <<"FLD+確率ハフ変換の線の傾きl["<<i<<"]("<<theta[i]<<")"<< std::endl;
       //cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(0,255,255), 2, cv::LINE_AA);//黄色の線（距離データ取得可能）
      }
    }

  //thetaの数値を小さいにソート
  double tmp,tmp1x,tmp1y,tmp2x,tmp2y,tmpdep1,tmpdep2;
  for (int i=0; i<=lines2.size(); ++i) {
     for (int j=i+1;j<lines2.size(); ++j) {
         if (theta[i] > theta[j]) {
             tmp =  theta[i];
             tmp1x =  lines2[i][0];
             tmp1y =  lines2[i][1];
             tmp2x =  lines2[i][2];
             tmp2y =  lines2[i][3];
             tmpdep1 = dep1[i];
             tmpdep2 = dep2[i];
            
             theta[i] = theta[j];
             lines2[i][0] = lines2[j][0];
             lines2[i][1] = lines2[j][1];
             lines2[i][2] = lines2[j][2];
             lines2[i][3] = lines2[j][3];
             dep1[i] = dep1[j];
             dep2[i] = dep2[j];
             
             theta[j] = tmp;
             lines2[j][0] = tmp1x;
             lines2[j][1] = tmp1y;
             lines2[j][2] = tmp2x;
             lines2[j][3] = tmp2y;
             dep1[j] = tmpdep1;
             dep2[j] = tmpdep2;
            }
        }
    }
    std::cout <<"水平線の傾きl("<<theta0<<")"<< std::endl;
    std::cout <<"垂直線の傾きl("<<theta90<<")"<< std::endl;

    std::cout <<"並び替え後"<<std::endl;
    for (int i=0; i< lines2.size(); ++i) {
        std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<")"<< std::endl;
        std::cout <<"FLD抽出線の傾きl2["<<i<<"]("<<theta[i]<<")"<< std::endl;
        //cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
    }
    //角度データテキスト化
    //for (int i=0; i<=lines2.size(); ++i) {std::cout <<theta[i]<< std::endl;}

    //線が縦線か横線か見極め２つに分類する（縦線0,横線１)

    int yokot,tatet,yokoj,tatej,p,yokoyouso,tateyouso;
    double yoko[200][20][2][4],tate[200][20][2][4],yokoB,tateB,lines2X,lines2Y,yokolines2[200][4],tatelines2[200][4],yokotheta[200],tatetheta[200];
    double yokoZ1[200],yokoZ2[200],tateZ1[200],tateZ2[200];
    double naname[200][20][2][4],nanameB;
    double yokothetal[100],tatethetal[100],yokolc[100],tatelc[100],yokol[200][4],tatel[200][4];
    double ynanamethetal[100],tnanamethetal[100],ynanamelc[100],tnanamelc[100],ynanamel[200][4],tnanamel[200][4];
    double A[3][500][2][4];
    int yokoi,tatei,yokonanamei,tatenanamei;
    double datat[tatei],datay[yokoi];//ヒストグラムデータ用
    
    yokot=0,tatet=0,yokoj=1,tatej=1,p=0,yokoyouso=0,tateyouso=0;
    yokoi=0,tatei=0,yokonanamei=0,tatenanamei=0;

    for (int j=0; j< lines2.size(); ++j) {
        lines2X=abs(lines2[j][0]-lines2[j][2]);//傾きを調べる（x成分)
        lines2Y=abs(lines2[j][1]-lines2[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(lines2X>lines2Y){
            std::cout <<"yoko(lines2X>lines2Y)="<<lines2X<<">"<<lines2Y<< std::endl;
            yokolines2[yokoyouso][0]=lines2[j][0];//(x成分)
            yokolines2[yokoyouso][1]=lines2[j][1];//(y成分)
            yokolines2[yokoyouso][2]=lines2[j][2];//(x成分)
            yokolines2[yokoyouso][3]=lines2[j][3];//(y成分)
            yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=dep1[j];
            yokoZ2[yokoyouso]=dep2[j];

            yokoB=yokotheta[0];
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)
        }
        //縦線に分類
        else{
        if(lines2Y>lines2X){
            std::cout <<"tate(lines2Y>lines2X)="<<lines2Y<<">"<<lines2X<< std::endl;
            tatelines2[tateyouso][0]=lines2[j][0];
            tatelines2[tateyouso][1]=lines2[j][1];
            tatelines2[tateyouso][2]=lines2[j][2];
            tatelines2[tateyouso][3]=lines2[j][3];
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

            A[0][yokoi][0][0]=yokolines2[i+1][0];//ヨコ線のu1座標
            A[0][yokoi][0][1]=yokolines2[i+1][1];//ヨコ線のv1座標
            A[0][yokoi][0][2]=yokoZ1[i+1];//ヨコ線のz1座標(三次元距離情報)
            A[0][yokoi][0][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[0][yokoi][1][0]=yokolines2[i+1][2];//ヨコ線のu2座標
            A[0][yokoi][1][1]=yokolines2[i+1][3];//ヨコ線のv2座標
            A[0][yokoi][1][2]=yokoZ2[i+1];//ヨコ線のz2座標(三次元距離情報)
            A[0][yokoi][1][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)


          std::cout <<"チェックA[0]["<<yokoi<<"][0][0]= "<<A[0][yokoi][0][0]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][0][1]= "<<A[0][yokoi][0][1]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][0][2]= "<<A[0][yokoi][0][2]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][0][3]= "<<A[0][yokoi][0][3]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]

          std::cout <<"チェックA[0]["<<yokoi<<"][1][0]= "<<A[0][yokoi][1][0]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][1][1]= "<<A[0][yokoi][1][1]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][1][2]= "<<A[0][yokoi][1][2]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]
          std::cout <<"チェックA[0]["<<yokoi<<"][1][3]= "<<A[0][yokoi][1][3]<< std::endl;//A[ヨコ0,タテ1,ヨコナナメ2,タテナナメ3][個数番号][点1or点2][要素]


          
            cv::line(img_line3,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(img_line4,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA);
            cv::line(img_dst,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA);
           
            //座標から一次関数を引く関数
            yokothetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines2[i+1][2]-yokolines2[i+1][0]),(yokolines2[i+1][3]-yokolines2[i+1][1])));
            yokolc[i+1]=(yokolines2[i+1][2]-yokolines2[i+1][0])*(yokolines2[i+1][2]-yokolines2[i+1][0])+(yokolines2[i+1][3]-yokolines2[i+1][1])*(yokolines2[i+1][3]-yokolines2[i+1][1]);
            yokol[i+1][0]=yokolines2[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//X1座標
            yokol[i+1][1]=yokolines2[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//Y1座標
            yokol[i+1][2]=yokolines2[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//X2座標
            yokol[i+1][3]=yokolines2[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< yokothetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< yokol[i+1][0] <<","<<yokol[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< yokol[i+1][2] <<","<<yokol[i+1][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(yokol[i+1][0],yokol[i+1][1]),cv::Point(yokol[i+1][2],yokol[i+1][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(yokol[i+1][0],yokol[i+1][1]),cv::Point(yokol[i+1][2],yokol[i+1][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);

            datay[yokoi]=yokotheta[i+1];

            yokoi=yokoi+1;
        }

        //横成分の範囲に含まれていない斜めの線
        else{ 
             std::cout <<"yokotheta["<<i+1<<"]= 横成分の範囲外（斜め線） "<< std::endl;
             std::cout <<"yokotheta["<<i+1<<"]="<<yokotheta[i+1]<< std::endl;
             std::cout <<"yokoB="<<yokoB<< std::endl;
             std::cout <<"yokotheta[i+1]-yokoB="<<yokotheta[i+1]-yokoB<< std::endl;


            A[2][yokonanamei][0][0]=yokolines2[i+1][0];//ヨコナナメ線のu1座標
            A[2][yokonanamei][0][1]=yokolines2[i+1][1];//ヨコナナメ線のv1座標
            A[2][yokonanamei][0][2]=yokoZ1[i+1];//ヨコナナメ線のz1座標(三次元距離情報)
            A[2][yokonanamei][0][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[2][yokonanamei][1][0]=yokolines2[i+1][2];//ヨコナナメ線のu2座標
            A[2][yokonanamei][1][1]=yokolines2[i+1][3];//ヨコナナメ線のv2座標
            A[2][yokonanamei][1][2]=yokoZ2[i+1];//ヨコナナメ線のz2座標(三次元距離情報)
            A[2][yokonanamei][1][3]=yokotheta[i+1];//代入([3]なのはθで[2]はZだから)


            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][0]= "<<A[2][yokonanamei][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][1]= "<<A[2][yokonanamei][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][2]= "<<A[2][yokonanamei][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][3]= "<<A[2][yokonanamei][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][0]= "<<A[2][yokonanamei][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][1]= "<<A[2][yokonanamei][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][2]= "<<A[2][yokonanamei][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][3]= "<<A[2][yokonanamei][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
        
             yokonanamei=yokonanamei+1;//配列繰り上がり、ｊリセット
             cv::line(img_line3,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

            //座標から一次関数を引く関数
            ynanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines2[i+1][2]-yokolines2[i+1][0]),(yokolines2[i+1][3]-yokolines2[i+1][1])));
            ynanamelc[i+1]=(yokolines2[i+1][2]-yokolines2[i+1][0])*(yokolines2[i+1][2]-yokolines2[i+1][0])+(yokolines2[i+1][3]-yokolines2[i+1][1])*(yokolines2[i+1][3]-yokolines2[i+1][1]);
            ynanamel[i+1][0]=yokolines2[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//X1座標
            ynanamel[i+1][1]=yokolines2[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//Y1座標
            ynanamel[i+1][2]=yokolines2[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//X2座標
            ynanamel[i+1][3]=yokolines2[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< ynanamethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< ynanamel[i+1][0] <<","<<ynanamel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< ynanamel[i+1][2] <<","<<ynanamel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(ynanamel[i+1][0],ynanamel[i+1][1]),cv::Point(ynanamel[i+1][2],ynanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(ynanamel[i+1][0],ynanamel[i+1][1]),cv::Point(ynanamel[i+1][2],ynanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                     
        }
    } 


    yokonanamei=yokonanamei-1;//配列繰り上がり、ｊリセット
    //グルーピング(縦線)
    for(int i = 0; i < tateyouso; i++){
            //縦成分の範囲を指定する
            //if((tatetheta[i+1]>=0&&0.1745329>=tatetheta[i+1])||(tatetheta[i+1]>=2.9670597&&3.14159265>=tatetheta[i+1])){
            //if(tatetheta[i+1]>=0&&0.1745329>=tatetheta[i+1]){//0〜10[deg]
            if(tatetheta[i+1]>=0&&0.26179938>=tatetheta[i+1]){//0〜15[deg]
                tatetheta[i+1]=tatetheta[i+1]*M_PI;
            }
            //if(tatetheta[i+1]>=2.9670597&&3.14159265>=tatetheta[i+1]){//170〜180[deg]
            if(tatetheta[i+1]>=2.87979326&&3.14159265>=tatetheta[i+1]){//165〜180[deg]

            std::cout <<"tatetheta["<<i+1<<"]= 縦成分の範囲内（0〜0.1745329、2.9670597〜3.14159265) "<< std::endl;
            std::cout <<"tatetheta["<<i+1<<"]="<<tatetheta[i+1]<< std::endl;

            A[1][tatei][0][0]=tatelines2[i+1][0];//タテ線のu1座標
            A[1][tatei][0][1]=tatelines2[i+1][1];//タテ線のv1座標
            A[1][tatei][0][2]=tateZ1[i+1];//タテ線のz1座標(三次元距離情報)
            A[1][tatei][0][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[1][tatei][1][0]=tatelines2[i+1][2];//タテ線のu2座標
            A[1][tatei][1][1]=tatelines2[i+1][3];//タテ線のv2座標
            A[1][tatei][1][2]=tateZ2[i+1];//タテ線のz2座標(三次元距離情報)
            A[1][tatei][1][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            std::cout <<"チェックA[1]["<<tatei<<"][0][0]= "<<A[1][tatei][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][0][1]= "<<A[1][tatei][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][0][2]= "<<A[1][tatei][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][0][3]= "<<A[1][tatei][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[1]["<<tatei<<"][1][0]= "<<A[1][tatei][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][1][1]= "<<A[1][tatei][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][1][2]= "<<A[1][tatei][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[1]["<<tatei<<"][1][3]= "<<A[1][tatei][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]



            cv::line(img_line3,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
            
            cv::line(img_line4,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            cv::line(img_dst,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);

             //座標から一次関数を引く関数
            tatethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines2[i+1][2]-tatelines2[i+1][0]),(tatelines2[i+1][3]-tatelines2[i+1][1])));
            tatelc[i+1]=(tatelines2[i+1][2]-tatelines2[i+1][0])*(tatelines2[i+1][2]-tatelines2[i+1][0])+(tatelines2[i+1][3]-tatelines2[i+1][1])*(tatelines2[i+1][3]-tatelines2[i+1][1]);
            tatel[i+1][0]=tatelines2[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//X1座標
            tatel[i+1][1]=tatelines2[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//Y1座標
            tatel[i+1][2]=tatelines2[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//X2座標
            tatel[i+1][3]=tatelines2[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tatethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tatel[i+1][0] <<","<<tatel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tatel[i+1][2] <<","<<tatel[i+1][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(tatel[i+1][0],tatel[i+1][1]),cv::Point(tatel[i+1][2],tatel[i+1][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(tatel[i+1][0],tatel[i+1][1]),cv::Point(tatel[i+1][2],tatel[i+1][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            datat[tatei]=tatetheta[i+1];
            tatei=tatei+1;//縦線に分類された線の個数

        }

        //縦成分の範囲に含まれていない斜めの線
        else{
             std::cout <<"tatetheta["<<i+1<<"]= 縦成分の範囲外（斜め線） "<< std::endl;
             std::cout <<"tatetheta["<<i+1<<"]="<<tatetheta[i+1]<< std::endl;
             std::cout <<"tateB="<<tateB<< std::endl;
             std::cout <<"tatetheta[i+1]-tateB="<<tatetheta[i+1]-tateB<< std::endl;

            A[2][yokonanamei][0][0]=tatelines2[i+1][0];//タテナナメ線のu1座標
            A[2][yokonanamei][0][1]=tatelines2[i+1][1];//タテナナメ線のv1座標
            A[2][yokonanamei][0][2]=tateZ1[i+1];//タテナナメ線のz1座標(三次元距離情報)
            A[2][yokonanamei][0][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[2][yokonanamei][1][0]=tatelines2[i+1][2];//タテナナメ線のu2座標
            A[2][yokonanamei][1][1]=tatelines2[i+1][3];//タテナナメ線のv2座標
            A[2][yokonanamei][1][2]=tateZ2[i+1];//タテナナメ線のz2座標(三次元距離情報)
            A[2][yokonanamei][1][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][0]= "<<A[2][yokonanamei][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][1]= "<<A[2][yokonanamei][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][2]= "<<A[2][yokonanamei][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][0][3]= "<<A[2][yokonanamei][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][0]= "<<A[2][yokonanamei][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][1]= "<<A[2][yokonanamei][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][2]= "<<A[2][yokonanamei][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[2]["<<yokonanamei<<"][1][3]= "<<A[2][yokonanamei][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

             yokonanamei=yokonanamei+1;//配列繰り上がり、ｊリセット
             cv::line(img_line3,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

             //座標から一次関数を引く関数
            tnanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines2[i+1][2]-tatelines2[i+1][0]),(tatelines2[i+1][3]-tatelines2[i+1][1])));
            tnanamelc[i+1]=(tatelines2[i+1][2]-tatelines2[i+1][0])*(tatelines2[i+1][2]-tatelines2[i+1][0])+(tatelines2[i+1][3]-tatelines2[i+1][1])*(tatelines2[i+1][3]-tatelines2[i+1][1]);
            tnanamel[i+1][0]=tatelines2[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//X1座標
            tnanamel[i+1][1]=tatelines2[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//Y1座標
            tnanamel[i+1][2]=tatelines2[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//X2座標
            tnanamel[i+1][3]=tatelines2[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tnanamethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tnanamel[i+1][0] <<","<<tnanamel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tnanamel[i+1][2] <<","<<tnanamel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(tnanamel[i+1][0],tnanamel[i+1][1]),cv::Point(tnanamel[i+1][2],tnanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);        
            cv::line(img_dst,cv::Point(tnanamel[i+1][0],tnanamel[i+1][1]),cv::Point(tnanamel[i+1][2],tnanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);

        }
    }



    std::cout <<"ナナメ線総数yokonanamei="<< yokonanamei <<std::endl;
    MatrixXd N(2,yokonanamei);//Nは2行L列の行列
    MatrixXd PC(2,yokonanamei);//推定交点
    int fx=100,fy=200;
    //一次関数を描写するための関数
    double thetal[yokonanamei],l[yokonanamei][4],lc[yokonanamei];

    //直線の作成(消失点と任意の点を端点とした線を作成する)
   for(int j=0;yokonanamei>j;j++){
    std::cout <<"j="<< j << std::endl;
      N(0,j)=A[2][j][0][0];//直線の端点の座標x
      N(1,j)=A[2][j][0][1];//直線の端点の座標y
      std::cout <<"直線の端点の座標x_N(0,"<<j<<")="<< N(0,j) << std::endl;
      std::cout <<"直線の端点の座標y_N(1,"<<j<<")="<< N(1,j) << std::endl;

      PC(0,j)=A[2][j][1][0];//消失点真値(x座標)_観測データ=真値±ノイズ
      PC(1,j)=A[2][j][1][1];//消失点真値(y座標)
      std::cout <<"消失点真値(x座標)_PC(0,"<<j<<")="<< PC(0,j) << std::endl;
      std::cout <<"消失点真値(y座標)_PC(1,"<<j<<")="<< PC(1,j) << std::endl;
    
  
      //座標から一次関数を引く関数
      thetal[j]=(M_PI/2)-(M_PI-atan2((PC(0,j)-N(0,j)),(PC(1,j)-N(1,j))));
      lc[j]=((PC(0,j)-N(0,j))*(PC(0,j)-N(0,j)))+((PC(1,j)-N(1,j))*(PC(1,j)-N(1,j)));
      l[j][0]=N(0,j)+(cos(-thetal[j])*sqrt(lc[j]))*10;//X1座標
      l[j][1]=N(1,j)+(sin(-thetal[j])*sqrt(lc[j]))*10;//Y1座標
      l[j][2]=N(0,j)+(cos(-thetal[j])*sqrt(lc[j]))*-10;//X2座標
      l[j][3]=N(1,j)+(sin(-thetal[j])*sqrt(lc[j]))*-10;//Y2座標
      //std::cout <<"直線の角度1θ="<< thetal[j] << std::endl;
      //std::cout <<"直線座標1=("<< l[j][0] <<","<<l[j][1]<<")"<< std::endl;
      //std::cout <<"直線座標2=("<< l[j][2] <<","<<l[j][3]<<")"<< std::endl;

      cv::line(img_line3,cv::Point(l[j][0],l[j][1]),cv::Point(l[j][2],l[j][3]),cv::Scalar(150,50,0), 1, cv::LINE_AA);
      cv::line(img_line3,cv::Point(PC(0,j),PC(1,j)),cv::Point(N(0,j),N(1,j)),cv::Scalar(150,50,0), 4, cv::LINE_AA);//直線の描写(青線)
      cv::circle(img_line3,Point(N(0,j),N(1,j)),5,Scalar(255,150,0),-1);
      cv::circle(img_line3,Point(PC(0,j),PC(1,j)),5,Scalar(0,150,255),-1);
    }


  //法線ベクトルを求める
  //法線ベクトルは直線の90度なので90度回転させる
  //ただし90度回転させただけなので、そのベクトルを単位ベクトル化することで法線ベクトルを作る(M2 4月研究参照)
  MatrixXd R(2,2);
  R(0,0)=cos(M_PI/2);
  R(0,1)=-sin(M_PI/2);
  R(1,0)=sin(M_PI/2);
  R(1,1)=cos(M_PI/2);

  //std::cout <<"回転行列R=\n"<< R << std::endl;

  MatrixXd n(2,yokonanamei);
  n=R*(PC-N);//直線を90度回転させたベクトルn
  //for(int t=0;yokonanamei>t;t++){
  //cv::line(img_line3,cv::Point(PC(0,t),PC(1,t)),cv::Point(PC(0,t)-n(0,t),PC(1,t)-n(1,t)),cv::Scalar(0,255,0), 4, cv::LINE_AA);//90度回転した直線(PC-nでベクトルから座標変換)(緑の線)
  //}
  
  //std::cout <<"直線を90度回転させたベクトルn=\n"<< n << std::endl;

  //法線ベクトルの大きさを１にする
  MatrixXd na(2,yokonanamei);
  MatrixXd na1(yokonanamei,0);
  na=n.transpose()*n;//na=n^T*n（ルートの中身を計算）
  na1=na.diagonal();//naを対角化することで要素の二乗の和を求める(例:a1^2+a2^2);
  //std::cout <<"na=\n"<< na << std::endl;
  //std::cout <<"naの対角行列=\n"<< na.diagonal() << std::endl;
  MatrixXd n1(yokonanamei,1);
  MatrixXd n2(2,yokonanamei);
  MatrixXd n22(yokonanamei,1);
  MatrixXd p0(yokonanamei,yokonanamei);
  MatrixXd n3(2,2);
  MatrixXd n30(yokonanamei,yokonanamei);
  MatrixXd n4(2,1);
  MatrixXd n40(2,1);
  MatrixXd X(2,1);
  MatrixXd X0(2,1);
    
  for(int t=0;yokonanamei>t;t++){
    n1(t,0)=sqrt(na1(t,0));//ベクトルnの大きさ（二乗の和に対しルートをかける）
    n2(0,t)=n(0,t)/n1(t,0);//法線ベクトル
    n2(1,t)=n(1,t)/n1(t,0);//法線ベクトル
    
    //cv::line(img_line3,cv::Point(PC(0,t),PC(1,t)),cv::Point((PC(0,t)-n2(0,t)*100),(PC(1,t)-n2(1,t)*100)),cv::Scalar(0,0,255), 4, cv::LINE_AA);//法線ベクトル(描写時に100倍してる）(赤の線)
    }

  std::cout <<"n1=\n"<< n1 << std::endl;
  std::cout <<"法線ベクトルn2=\n"<< n2 << std::endl;

  //最小二乗法の計算
  p0=n2.transpose()*N;//P0=nT*Nを計算//ここ要チェック------------------------------------------------
  n3=n2*n2.transpose();//逆行列の内部を計算

  //最小二乗法の計算
  n4=n2*p0.diagonal();//nにP0の対角行列をかけている 
  X=n3.inverse()*n4;//逆行列と法線ベクトルの転置を掛け算
  
  std::cout <<"p0=\n"<< p0 << std::endl;
  std::cout <<"n3=\n"<< n3 << std::endl;
  std::cout <<"n3の逆行列=\n"<< n3.inverse() << std::endl;
  std::cout <<"n4=\n"<< n4 << std::endl;
  std::cout <<"p0.diagonal()=\n"<< p0.diagonal() << std::endl;
  std::cout <<"X=\n"<< X << std::endl;//推定交点

  cv::circle(img_line3,Point(X(0,0),X(1,0)),8,Scalar(0,0,255),-1);
  cv::circle(img_dst,Point(X(0,0),X(1,0)),8,Scalar(0,0,255),-1);

  //cv::circle(img_line3,Point(X0(0,0),X0(1,0)),5,Scalar(255,0,255),-1);


    //縦線の角度のヒストグラムから平行線を見つける-----------------------------------------------------------------------
    // 変数の初期化
    int histot_i = 0;
    int histot_j = 0;
    int histot_tmp = 0;
    double histot_med = 0;
    double histot_renge = 0;
    
    //データを大きさの順に並べ替え
    for(int i = 1; i < tatei; i++){
        for(int j = 0; j < tatei - i; j++){
            if(datat[j] > datat[j + 1]){
                histot_tmp = datat[j];
                datat[j] = datat[j + 1];
                datat[j + 1] = histot_tmp;
            }
        }
    }

    // メジアンを求める
    // データ数が奇数個の場合
    if(tatei % 2 == 1){ histot_med = datat[(tatei - 1) / 2]; }  // メジアン

    // データ数が偶数の場合
    else{ histot_med = (datat[(tatei / 2) - 1] + datat[tatei / 2]) / 2.0; } // メジアン

    // レンジを求める
    histot_renge = datat[tatei - 1] - datat[0] + 0.0;  // 範囲

    double modet;              // これまでに調べた中での最頻値
    int count_maxt = 0;     // これまでに調べた登場回数の中で最大のもの
    for(int i = 1; i < tatei; i++){
        std::cout << A[1][i][1][3] << std::endl;

        double valuet =  A[1][i][1][3];
        int countt = 1;
    
        // ソートされているのだから、
        // 同じ値があるとすれば、後続に連続して並んでいるはず。
        // その回数をカウントする。
        for (i = i + 1; i < tatei; ++i) {
            if (valuet ==  A[1][i][1][3]) {
                ++countt;
            }
            else {
                // 違う値が登場したら終わり
                break;
            }
        };

        // これまでの最大の登場回数よりも多かったら、更新する
        if (count_maxt < countt) {
            count_maxt = countt;
            modet = valuet;
        }
    }
    for(int i = 1; i < tatei; i++){
        std::cout << A[1][i][1][3] << std::endl;}
    std::cout <<"タテ線_  最瀕値は"<<modet<< std::endl;
    printf("タテ線＿メジアンは %0.2f\n", histot_med);
    printf("タテ線＿範囲    は %0.2f\n", histot_renge);

    cv::line(img_line3,cv::Point(100,380),cv::Point(100-100*sin(modet),380+100*cos(modet)),cv::Scalar(0,0,255), 3, cv::LINE_AA);
    cv::line(img_line3,cv::Point(100,380),cv::Point(100-100*sin(histot_med),380+100*cos(histot_med)),cv::Scalar(0,100,255), 3, cv::LINE_AA);


    //外積処理
    cv::Mat tate_Mat = (cv::Mat_<double>(3, 1) << 
    -100*sin(modet),
    100*cos(modet), 
    1);


   
    //縦線の角度のヒストグラムから平行線を見つける---------------------------------------------------
    // 変数の初期化
    double histoy_i = 0;
    double histoy_j = 0;
    double histoy_tmp = 0;
    double histoy_med = 0;
    double histoy_renge = 0;
    double  histogram[2 + 1];  // ヒストグラム
    
    //データを大きさの順に並べ替え
    for(int i = 1; i <  yokoi; i++){
        for(int j = 0; j < yokoi - i; j++){
            if(datay[j] > datay[j + 1]){
                histoy_tmp = datay[j];
                datay[j] = datay[j + 1];
                datay[j + 1] = histoy_tmp;
            }
        }
    }

    // メジアンを求める
    // データ数が奇数個の場合
    if(yokoi % 2 == 1){ histoy_med = datay[(yokoi - 1) / 2]; }  // メジアン

    // データ数が偶数の場合
    else{ histoy_med = (datay[(yokoi / 2) - 1] + datay[yokoi / 2]) / 2.0; } // メジアン

    // レンジを求める
    histoy_renge = datay[yokoi - 1] - datay[0] + 0.0;  // 範囲

    double modey;              // これまでに調べた中での最頻値
    int count_maxy = 0;     // これまでに調べた登場回数の中で最大のもの
    for(int i = 1; i < yokoi; i++){
        std::cout << A[0][i][1][3] << std::endl;

        double valuey =  datay[i];
        //double valuey =  A[0][i][1][3];
        int county = 1;
    
        // ソートされているのだから、
        // 同じ値があるとすれば、後続に連続して並んでいるはず。
        // その回数をカウントする。
        for (i = i + 1; i < yokoi; ++i) {
            if (valuey ==  datay[i]) {
            //if (valuey ==  A[0][i][1][3]) {
                ++county;
            }
            else {
                // 違う値が登場したら終わり
                break;
            }
        };

        // これまでの最大の登場回数よりも多かったら、更新する
        if (count_maxy < county) {
            count_maxy = county;
            modey = valuey;
        }
    }
    for(int i = 1; i < yokoi; i++){
        std::cout << A[0][i][1][3] << std::endl;}
    std::cout <<"ヨコ線_  最瀕値は"<<modey<< std::endl;
    std::cout <<"ヨコ線_メジアンは"<<histoy_med<< std::endl;
    std::cout <<"ヨコ線_範囲    は"<<histoy_renge<< std::endl;

    cv::line(img_line3,cv::Point(100,380),cv::Point(100+100*sin(modey),380-100*cos(modey)),cv::Scalar(0,255,0), 3, cv::LINE_AA);
    cv::line(img_line3,cv::Point(100,380),cv::Point(100+100*sin(histoy_med),380-100*cos(histoy_med)),cv::Scalar(0,255,100), 3, cv::LINE_AA);


    //外積処理
    cv::Mat yoko_Mat = (cv::Mat_<double>(3, 1) << 
    100*sin(modey),
    -100*cos(modey), 
    1);

    cv::Mat naname_Mat = yoko_Mat.cross(tate_Mat);
	cout << "yoko_Matとtate_Matの外積 = " << naname_Mat << endl;

    cv::line(img_line3,cv::Point(100,380),cv::Point(naname_Mat.at<double>(0)/naname_Mat.at<double>(2)+100,
    naname_Mat.at<double>(1)/naname_Mat.at<double>(2)+380),cv::Scalar(255,0,0), 3, cv::LINE_AA);





   
    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth3, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth4, cv::WINDOW_AUTOSIZE);
   
    cv::imshow(win_src, img_src);
    //cv::imshow(win_depth, img_depth);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_line, img_line);
    //cv::imshow(win_line2, img_line2);
    cv::imshow(win_line3, img_line3);
    cv::imshow(win_line4, img_line4);
    //cv::imshow(win_depth2, img_depth2);
    //cv::imshow(win_depth3, img_depth3);
    //cv::imshow(win_depth4, img_depth4);
 
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


	ros::spin();//トピック更新待機
			
	return 0;
}