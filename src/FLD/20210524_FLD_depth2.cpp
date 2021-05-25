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

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180

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

    cv::line(img_dst,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青水平
    cv::line(img_dst,cv::Point(100,0),cv::Point(100,200),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑垂直

    //FLD変換
    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines.size(); i++){
       cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
       cv::line(img_depth2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
       cv::line(img_depth4,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
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
     cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

     if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)

      cv::circle(img_line2,Point(lines2[i][0],lines2[i][1]),5,Scalar(255,0,0),-1);//青点
      cv::circle(img_line2,Point(lines2[i][2],lines2[i][3]),5,Scalar(0,255,0),-1);//緑点

      std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<","<<dep1[i]/1000<<"[m])"<< std::endl;
      std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<","<<dep2[i]/1000<<"[m])"<< std::endl;

       //FLD抽出線のy軸との角度を求める
       theta[i]=M_PI-atan2((lines2[i][2]-lines2[i][0]),(lines2[i][3]-lines2[i][1]));
       std::cout <<"FLD+確率ハフ変換の線の傾きl["<<i<<"]("<<theta[i]<<")"<< std::endl;
       cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(0,255,255), 2, cv::LINE_AA);//黄色の線（距離データ取得可能）
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
    double A[4][200][2][4];
    int yokoi,tatei,yokonanamei,tatenanamei;
    
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

            yokoi=yokoi+1;
          
            cv::line(img_line3,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(img_line4,cv::Point(yokolines2[i+1][0],yokolines2[i+1][1]),cv::Point(yokolines2[i+1][2],yokolines2[i+1][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA);
           
            //座標から一次関数を引く関数
            yokothetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines2[i+1][2]-yokolines2[i+1][0]),(yokolines2[i+1][3]-yokolines2[i+1][1])));
            yokolc[i+1]=(yokolines2[i+1][2]-yokolines2[i+1][0])*(yokolines2[i+1][2]-yokolines2[i+1][0])+(yokolines2[i+1][3]-yokolines2[i+1][1])*(yokolines2[i+1][3]-yokolines2[i+1][1]);
            yokol[i+1][0]=yokolines2[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//X1座標
            yokol[i+1][1]=yokolines2[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*10;//Y1座標
            yokol[i+1][2]=yokolines2[i+1][0]+(cos(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//X2座標
            yokol[i+1][3]=yokolines2[i+1][1]+(sin(-yokothetal[i+1])*sqrt(yokolc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< yokothetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< yokol[i+1][0] <<","<<yokol[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< yokol[i+1][2] <<","<<yokol[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(yokol[i+1][0],yokol[i+1][1]),cv::Point(yokol[i+1][2],yokol[i+1][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
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

            /* //座標から一次関数を引く関数
            ynanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((yokolines2[i+1][2]-yokolines2[i+1][0]),(yokolines2[i+1][3]-yokolines2[i+1][1])));
            ynanamelc[i+1]=(yokolines2[i+1][2]-yokolines2[i+1][0])*(yokolines2[i+1][2]-yokolines2[i+1][0])+(yokolines2[i+1][3]-yokolines2[i+1][1])*(yokolines2[i+1][3]-yokolines2[i+1][1]);
            ynanamel[i+1][0]=yokolines2[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//X1座標
            ynanamel[i+1][1]=yokolines2[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*10;//Y1座標
            ynanamel[i+1][2]=yokolines2[i+1][0]+(cos(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//X2座標
            ynanamel[i+1][3]=yokolines2[i+1][1]+(sin(-ynanamethetal[i+1])*sqrt(ynanamelc[i+1]))*-10;//Y2座標
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

            tatei=tatei+1;
            cv::line(img_line3,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
            cv::line(img_line4,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
             //座標から一次関数を引く関数
            tatethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines2[i+1][2]-tatelines2[i+1][0]),(tatelines2[i+1][3]-tatelines2[i+1][1])));
            tatelc[i+1]=(tatelines2[i+1][2]-tatelines2[i+1][0])*(tatelines2[i+1][2]-tatelines2[i+1][0])+(tatelines2[i+1][3]-tatelines2[i+1][1])*(tatelines2[i+1][3]-tatelines2[i+1][1]);
            tatel[i+1][0]=tatelines2[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//X1座標
            tatel[i+1][1]=tatelines2[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*10;//Y1座標
            tatel[i+1][2]=tatelines2[i+1][0]+(cos(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//X2座標
            tatel[i+1][3]=tatelines2[i+1][1]+(sin(-tatethetal[i+1])*sqrt(tatelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tatethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tatel[i+1][0] <<","<<tatel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tatel[i+1][2] <<","<<tatel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(tatel[i+1][0],tatel[i+1][1]),cv::Point(tatel[i+1][2],tatel[i+1][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            } 

        //縦成分の範囲に含まれていない斜めの線
        else{
             std::cout <<"tatetheta["<<i+1<<"]= 縦成分の範囲外（斜め線） "<< std::endl;
             std::cout <<"tatetheta["<<i+1<<"]="<<tatetheta[i+1]<< std::endl;
             std::cout <<"tateB="<<tateB<< std::endl;
             std::cout <<"tatetheta[i+1]-tateB="<<tatetheta[i+1]-tateB<< std::endl;

            A[3][tatenanamei][0][0]=tatelines2[i+1][0];//タテナナメ線のu1座標
            A[3][tatenanamei][0][1]=tatelines2[i+1][1];//タテナナメ線のv1座標
            A[3][tatenanamei][0][2]=tateZ1[i+1];//タテナナメ線のz1座標(三次元距離情報)
            A[3][tatenanamei][0][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            A[3][tatenanamei][1][0]=tatelines2[i+1][2];//タテナナメ線のu2座標
            A[3][tatenanamei][1][1]=tatelines2[i+1][3];//タテナナメ線のv2座標
            A[3][tatenanamei][1][2]=tateZ2[i+1];//タテナナメ線のz2座標(三次元距離情報)
            A[3][tatenanamei][1][3]=tatetheta[i+1];//代入([3]なのはθで[2]はZだから)

            std::cout <<"チェックA[3]["<<tatenanamei<<"][0][0]= "<<A[3][tatenanamei][0][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][0][1]= "<<A[3][tatenanamei][0][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][0][2]= "<<A[3][tatenanamei][0][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][0][3]= "<<A[3][tatenanamei][0][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

            std::cout <<"チェックA[3]["<<tatenanamei<<"][1][0]= "<<A[3][tatenanamei][1][0]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][1][1]= "<<A[3][tatenanamei][1][1]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][1][2]= "<<A[3][tatenanamei][1][2]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]
            std::cout <<"チェックA[3]["<<tatenanamei<<"][1][3]= "<<A[3][tatenanamei][1][3]<< std::endl;//A[グループ番号][個数番号][点1or点2][要素]

             
        
             tatenanamei=tatenanamei+1;//配列繰り上がり、ｊリセット
             cv::line(img_line3,cv::Point(tatelines2[i+1][0],tatelines2[i+1][1]),cv::Point(tatelines2[i+1][2],tatelines2[i+1][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

            /*  //座標から一次関数を引く関数
            tnanamethetal[i+1]=(M_PI/2)-(M_PI-atan2((tatelines2[i+1][2]-tatelines2[i+1][0]),(tatelines2[i+1][3]-tatelines2[i+1][1])));
            tnanamelc[i+1]=(tatelines2[i+1][2]-tatelines2[i+1][0])*(tatelines2[i+1][2]-tatelines2[i+1][0])+(tatelines2[i+1][3]-tatelines2[i+1][1])*(tatelines2[i+1][3]-tatelines2[i+1][1]);
            tnanamel[i+1][0]=tatelines2[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//X1座標
            tnanamel[i+1][1]=tatelines2[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*10;//Y1座標
            tnanamel[i+1][2]=tatelines2[i+1][0]+(cos(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//X2座標
            tnanamel[i+1][3]=tatelines2[i+1][1]+(sin(-tnanamethetal[i+1])*sqrt(tnanamelc[i+1]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< tnanamethetal[i+1] << std::endl;
            std::cout <<"直線座標1=("<< tnanamel[i+1][0] <<","<<tnanamel[i+1][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< tnanamel[i+1][2] <<","<<tnanamel[i+1][3]<<")"<< std::endl;

            cv::line(img_line4,cv::Point(tnanamel[i+1][0],tnanamel[i+1][1]),cv::Point(tnanamel[i+1][2],tnanamel[i+1][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA); */       
        }

    } 



    /* //グルーピングチェック
    std::cout <<"総グループ数= "<<t+1<< std::endl;
    double va[100][10],vb[100][10],P[100][20][2][4];

    for(int j=0;j<=t;j++){
        std::cout <<"線Aの総数["<<j<<"]= "<<c[j]<< std::endl;
        std::cout <<"グループ番号= "<<j<< std::endl;
        int i2=0;
        std::cout <<"初期化i2= "<<i2<< std::endl;
      for(int i=0;i<c[j];i++){
        if(A[j][i][0][0]>=0 && A[j][i][0][1]>=0){//画像座標uの値が0以上の時のみに実行(画像座標データの損失を考慮)
        if(A[j][i][1][0]>=0 && A[j][i][1][1]>=0){//画像座標vの値が0以上の時のみに実行(画像座標データの損失を考慮)
          //if(i!=0){i2=i2+1;}//Pの線番号の更新
          std::cout <<"カウントアップi2= "<<i2<< std::endl;

          //上のが要素によって覗かれるので連番にならない、そこで次の工程で連番にする
          P[j][i2][0][0]=A[j][i][0][0],P[j][i2][1][0]=A[j][i][1][0];
          P[j][i2][0][1]=A[j][i][0][1],P[j][i2][1][1]=A[j][i][1][1];
          P[j][i2][0][3]=A[j][i][0][3],P[j][i2][1][3]=A[j][i][1][3];

          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][0]= "<<P[j][i2][0][0]<< std::endl;//P[グループ番号][個数番号][点1or点2][x]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][1]= "<<P[j][i2][0][1]<< std::endl;//P[グループ番号][個数番号][点1or点2][y]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][3]= "<<P[j][i2][0][3]<< std::endl;//P[グループ番号][個数番号][点1or点2][θ]

          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][0]= "<<P[j][i2][1][0]<< std::endl;//P[グループ番号][個数番号][点1or点2][x]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][1]= "<<P[j][i2][1][1]<< std::endl;//P[グループ番号][個数番号][点1or点2][y]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][3]= "<<P[j][i2][1][3]<< std::endl;//P[グループ番号][個数番号][点1or点2][θ]
          //ここでi2回回しているつまり、i2個線が存在するということである  
          i2=i2+1;
          }}//if文last
        }//for文 i最後

        for(int i=0;i<i2;i++){
          //一次関数のaとbの要素を求めている
          va[j][i]=(P[j][i][0][1]-P[j][i][1][1])/(P[j][i][0][0]-P[j][i][1][0]);
          vb[j][i]=P[j][i][0][1]-(P[j][i][0][0]*((P[j][i][0][1]-P[j][i][1][1])/(P[j][i][0][0]-P[j][i][1][0])));

          //一次関数の極座標形式化

          //消失点ラインの描写
          double u0,v0,v640;
        
            v0=P[j][i][0][1]-(P[j][i][0][0]*((P[j][i][0][1]-P[j][i][1][1])/(P[j][i][0][0]-P[j][i][1][0])));//v軸との交点(0,v0)
            u0=P[j][i][0][0]-(P[j][i][0][1]*((P[j][i][0][0]-P[j][i][1][0])/(P[j][i][0][1]-P[j][i][1][1])));//u軸との交点(u0,0)
            if(v0<=0||u0<=0){
              //uとvがマイナスになってしまう場合
              //(640,v640)画面端との交点
              v640=va[j][i]*640+vb[j][i];
              //v640=((A[j][i][0][1]-A[j][i][1][1])/(A[j][i][0][0]-A[j][i][1][0]))*640+A[j][i][0][1]-(A[j][i][0][0]*((A[j][i][0][1]-A[j][i][1][1])/(A[j][i][0][0]-A[j][i][1][0])));
              }
          
          int R,G,B;
          double ro,rox1,rox2;

          ro=P[j][i][0][0]*cos(P[j][i][0][2])+P[j][i][0][1]*sin(P[j][i][0][2]);
          rox1=ro-1000;
          rox2=ro+1000;

          if(j==0){B=255,G=0,R=0;}
          if(j==1){B=0,G=255,R=0;}
          if(j==2){B=0,G=0,R=255;}
          if(j==3){B=255,G=0,R=255;}


         if(lines2.size()!=0){ 
             cv::line(img_line3,cv::Point(P[j][i][0][0],P[j][i][0][1]), cv::Point(P[j][i][1][0],P[j][i][1][1]), cv::Scalar(B,G,R), 2, cv::LINE_AA);
             //cv::line(img_line3,cv::Point(rox1,1000), cv::Point(rox2,1000), cv::Scalar(B,G,R), 2, cv::LINE_AA);
         
             //ラインが垂直の時
            if(P[j][i][0][0]==P[j][i][1][0]){
            //cv::line(img_line3,cv::Point(u0,0), cv::Point(u0,480), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            }
            //ラインが水平の時
            else if(P[j][i][0][1]==P[j][i][1][1]){
            //cv::line(img_line3,cv::Point(0,v0), cv::Point(640,v0), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            }
            else{//ラインがそれ以外の時
            //cv::line(img_line3,cv::Point(0,v0), cv::Point(u0,0), cv::Scalar(B,G,R), 1, cv::LINE_AA);
            }

             if(u0<=0){
            //u0がマイナスになってしまう場合
             //cv::line(img_line3,cv::Point(0,v0), cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA);
             }
             if(v0<=0){
            //v0がマイナスになってしまう場合
             //cv::line(img_line3,cv::Point(u0,0), cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA);
             }
             }

         
        }


    }//for文 j最後*/

    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth3, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_depth4, cv::WINDOW_AUTOSIZE);
   
    cv::imshow(win_src, img_src);
    cv::imshow(win_depth, img_depth);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_line, img_line);
    cv::imshow(win_line2, img_line2);
    cv::imshow(win_line3, img_line3);
    cv::imshow(win_line4, img_line4);
    cv::imshow(win_depth2, img_depth2);
    cv::imshow(win_depth3, img_depth3);
    cv::imshow(win_depth4, img_depth4);
 
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