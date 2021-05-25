//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
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


ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher marker_pub;
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
    
    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入


//ここに処理項目
	  cv::Mat img_src = RGBimage;
    cv::Mat img_depth = depthimage;
    cv::Mat img_gray,img_edge,img_dst,img_dst2,img_dst3;
    cv::Mat img_line,img_line1;

    img_src.copyTo(img_dst);
    //img_src.copyTo(img_dst2);
    
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);
    cv::equalizeHist(img_gray,img_dst2);//ヒストグラム均一化

    //膨張縮小処理(一回目)
    Mat img_tmp,img_ab,img_tmpp;
    Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
    morphologyEx(img_dst2,img_tmp,MORPH_CLOSE,element8,Point(-1,-1),1);
    morphologyEx(img_tmp,img_ab,MORPH_OPEN,element8,Point(-1,-1),1);
    
    morphologyEx(img_ab,img_tmpp,MORPH_OPEN,element8,Point(-1,-1),1);
    morphologyEx(img_tmpp,img_dst3,MORPH_CLOSE,element8,Point(-1,-1),1);

    cv::Canny(img_dst3, img_edge, 200, 200);

    float dep,dep1[100],dep2[100];
    double theta[100];
    
    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);

    //標準的ハフ変換1(元画像・lines0)
    std::vector<cv::Vec2f> lines0;
    cv::HoughLines(img_edge, lines0, 1, CV_PI/180, 120);

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
    int c[20],t,j,p;
    double A[20][20][2][4],B;
    c[0]=1,t=0,j=1,p=0;

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
            A[t][j][0][0]=lines[i+1][0];//代入
            A[t][j][0][1]=lines[i+1][1];//代入
            A[t][j][0][2]=dep1[i+1];//代入
            A[t][j][0][3]=theta[i+1];//代入
            A[t][j][1][0]=lines[i+1][2];//代入
            A[t][j][1][1]=lines[i+1][3];//代入
            A[t][j][1][2]=dep2[i+1];//代入
            A[t][j][1][3]=theta[i+1];//代入
            j=j+1;//配列カウント
            c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
        }
        //前の番号と異なる数値
        else{

          if(theta[i+1]-B>0.3){//前の角度との差が0.5より大きい
             std::cout <<"theta["<<i+1<<"]= 異なる数値 "<< std::endl;
             std::cout <<"theta["<<i+1<<"]="<<theta[i+1]<< std::endl;
             std::cout <<"B="<<B<< std::endl;
             std::cout <<"theta[i+1]-B="<<theta[i+1]-B<< std::endl;
        
             t=t+1,j=0;//配列繰り上がり、ｊリセット
             A[t][j][0][0]=lines[i+1][0];//代入
             A[t][j][0][1]=lines[i+1][1];//代入
             A[t][j][0][2]=dep1[i+1];//代入
             A[t][j][0][3]=theta[i+1];//代入
             A[t][j][1][0]=lines[i+1][2];//代入
             A[t][j][1][1]=lines[i+1][3];//代入
             A[t][j][1][2]=dep2[i+1];//代入
             A[t][j][1][3]=theta[i+1];//代入
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[t]=0;//配列要素数初期値
            }

          else{//前の角度との差が0.5以下
             std::cout <<"theta["<<i+1<<"]= ちょっと違う数値= "<<theta[i+1]-B<< std::endl;
             A[t][j][0][0]=lines[i+1][0];//代入
             A[t][j][0][1]=lines[i+1][1];//代入
             A[t][j][0][2]=dep1[i+1];//代入
             A[t][j][0][3]=theta[i+1];//代入
             A[t][j][1][0]=lines[i+1][2];//代入
             A[t][j][1][1]=lines[i+1][3];//代入
             A[t][j][1][2]=dep2[i+1];//代入
             A[t][j][1][3]=theta[i+1];//代
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
            }
        }
      std::cout <<"C["<<t+1<<"]="<< c[t] << std::endl;
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



    //グルーピングチェック
    std::cout <<"総グループ数= "<<t+1<< std::endl;
    double va[10][10],vb[10][10],P[3][20][2][4];

    for(int j=0;j<=t;j++){
        std::cout <<"線Aの総数["<<j<<"]= "<<c[j]<< std::endl;
        std::cout <<"グループ番号= "<<j<< std::endl;
        int i2=0;
        std::cout <<"初期化i2= "<<i2<< std::endl;
      for(int i=0;i<c[j];i++){
        if(A[j][i][0][0]>=0 && A[j][i][0][1]>=0){//画像座標uの値が0以上の時のみに実行(画像座標データの損失を考慮)
        if(A[j][i][1][0]>=0 && A[j][i][1][1]>=0){//画像座標vの値が0以上の時のみに実行(画像座標データの損失を考慮)
          if(A[j][i][0][2]>0 && A[j][i][1][2]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
          //if(i!=0){i2=i2+1;}//Pの線番号の更新
          std::cout <<"カウントアップi2= "<<i2<< std::endl;

          //上のが要素によって覗かれるので連番にならない、そこで次の工程で連番にする
          P[j][i2][0][0]=A[j][i][0][0],P[j][i2][1][0]=A[j][i][1][0];
          P[j][i2][0][1]=A[j][i][0][1],P[j][i2][1][1]=A[j][i][1][1];
          P[j][i2][0][2]=A[j][i][0][2],P[j][i2][1][2]=A[j][i][1][2];
          P[j][i2][0][3]=A[j][i][0][3],P[j][i2][1][3]=A[j][i][1][3];

          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][0]= "<<P[j][i2][0][0]<< std::endl;//P[グループ番号][個数番号][点1or点2][x]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][1]= "<<P[j][i2][0][1]<< std::endl;//P[グループ番号][個数番号][点1or点2][y]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][2]= "<<P[j][i2][0][2]<< std::endl;//P[グループ番号][個数番号][点1or点2][z]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][0][3]= "<<P[j][i2][0][3]<< std::endl;//P[グループ番号][個数番号][点1or点2][θ]

          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][0]= "<<P[j][i2][1][0]<< std::endl;//P[グループ番号][個数番号][点1or点2][x]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][1]= "<<P[j][i2][1][1]<< std::endl;//P[グループ番号][個数番号][点1or点2][y]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][2]= "<<P[j][i2][1][2]<< std::endl;//P[グループ番号][個数番号][点1or点2][z]
          std::cout <<"チェックP["<<j<<"]["<<i2<<"][1][3]= "<<P[j][i2][1][3]<< std::endl;//P[グループ番号][個数番号][点1or点2][θ]
          //ここでi2回回しているつまり、i2個線が存在するということである  
          i2=i2+1;
          }}}//if文last
        }//for文 i最後
        std::cout <<"有効線Pの総数["<<j<<"]= "<<i2<< std::endl;

        for(int i=0;i<i2;i++){
          //一次関数のaとbの要素を求めている
          va[j][i]=(P[j][i][0][1]-P[j][i][1][1])/(P[j][i][0][0]-P[j][i][1][0]);
          vb[j][i]=P[j][i][0][1]-(P[j][i][0][0]*((P[j][i][0][1]-P[j][i][1][1])/(P[j][i][0][0]-P[j][i][1][0])));

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
          

          geometry_msgs::Point p;
          p.x = P[j][i][0][0]*0.007;
          p.z = P[j][i][0][1]*0.007;
          p.y = P[j][i][0][2]*0.01;
            // ラインリストは、各ラインに2点必要
          line_list.points.push_back(p);
          p.x =P[j][i][1][0]*0.007;
          p.z =P[j][i][1][1]*0.007;
          p.y =P[j][i][1][2]*0.01;
          line_list.points.push_back(p);

          int R,G,B;
          double ro,rox1,rox2;

          ro=P[j][i][0][0]*cos(P[j][i][0][2])+P[j][i][0][1]*sin(P[j][i][0][2]);
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
             cv::line(img_dst,cv::Point(P[j][i][0][0],P[j][i][0][1]), cv::Point(P[j][i][1][0],P[j][i][1][1]), cv::Scalar(B,G,R), 4, cv::LINE_AA);
             cv::line(img_line,cv::Point(P[j][i][0][0],P[j][i][0][1]), cv::Point(P[j][i][1][0],P[j][i][1][1]), cv::Scalar(B,G,R), 4, cv::LINE_AA);
             cv::line(img_line,cv::Point(rox1,1000), cv::Point(rox2,1000), cv::Scalar(B,G,R), 2, cv::LINE_AA);
         
             //ラインが垂直の時
            if(P[j][i][0][0]==P[j][i][1][0]){
            cv::line(img_dst,cv::Point(u0,0), cv::Point(u0,480), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            cv::line(img_line,cv::Point(u0,0), cv::Point(u0,480), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            }
            //ラインが水平の時
            else if(P[j][i][0][1]==P[j][i][1][1]){
            cv::line(img_dst,cv::Point(0,v0), cv::Point(640,v0), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            cv::line(img_line,cv::Point(0,v0), cv::Point(640,v0), cv::Scalar(0,0,0), 1, cv::LINE_AA);
            }
            else{//ラインがそれ以外の時
            cv::line(img_line,cv::Point(0,v0), cv::Point(u0,0), cv::Scalar(B,G,R), 1, cv::LINE_AA);
            cv::line(img_dst, cv::Point(u0,0),cv::Point(0,v0), cv::Scalar(B,G,R), 1, cv::LINE_AA);
            }

             if(u0<=0){
            //u0がマイナスになってしまう場合
             cv::line(img_line,cv::Point(0,v0), cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA);
             cv::line(img_dst, cv::Point(0,v0),cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA); }
             if(v0<=0){
            //v0がマイナスになってしまう場合
             cv::line(img_line,cv::Point(u0,0), cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA);
             cv::line(img_dst, cv::Point(u0,0),cv::Point(640,v640), cv::Scalar(B,G,R), 1, cv::LINE_AA); }
             }

          marker_pub.publish(line_list);
          // image_pub.publish(image_data)
        }
       
        int vn=0,vm=0,vf=0,Vp[300];
        double Vx[5][10][10],Vy[5][10][10],Vr[5][10][10],Vxn[5][150],Vyn[5][150],Vrn[5][150];
         //for(int i=0;i<i2;i++){
          //一次関数の交点
          //有効線Pの総数はi2
          //if(i!=0){
            Vp[j]=0;//交点番号
            for(vn=0;vn<=i2-2;vn++){
            for(vm=vn+1;vm<=i2-1;vm++){
             Vx[j][vn][vm]=-((vb[j][vn]-vb[j][vm])/(va[j][vn]-va[j][vm]));
             Vy[j][vn][vm]=va[j][vn]*Vx[j][vn][vm]+vb[j][vn];
             std::cout <<"一次関数の交点Vx["<<j<<"]["<<vn<<"]["<<vm<<"]= "<<Vx[j][vn][vm]<< std::endl;
             std::cout <<"一次関数の交点Vy["<<j<<"]["<<vn<<"]["<<vm<<"]= "<<Vy[j][vn][vm]<< std::endl;
             cv::circle(img_line,Point(Vx[j][vn][vm],Vy[j][vn][vm]),8,Scalar(0,0,255),-1);//赤点(一次関数交点画像座標)
             Vr[j][vn][vm]=sqrt(Vx[j][vn][vm]*Vx[j][vn][vm]+Vy[j][vn][vm]*Vy[j][vn][vm]);//交点から原点までの距離
             std::cout <<"原点から交点までの距離Vr["<<j<<"]["<<vn<<"]["<<vm<<"]= "<<Vr[j][vn][vm]<< std::endl;
             std::cout <<"交点番号基準に並び替える"<< std::endl;
             std::cout <<"交点番号n["<<j<<"]= "<<Vp[j]<< std::endl;

             //交点番号基準に配列を書き換える
             Vxn[j][Vp[j]]=Vx[j][vn][vm];
             Vyn[j][Vp[j]]=Vy[j][vn][vm];
             Vrn[j][Vp[j]]=Vr[j][vn][vm];//原点から交点までの距離（交点番号配列）
             std::cout <<"一次関数の交点Vxn["<<j<<"]["<<Vp[j]<<"]= "<<Vxn[j][Vp[j]]<< std::endl;
             std::cout <<"一次関数の交点Vyn["<<j<<"]["<<Vp[j]<<"]= "<<Vyn[j][Vp[j]]<< std::endl;
             std::cout <<"原点から交点までの距離Vrn["<<j<<"]["<<Vp[j]<<"]= "<<Vrn[j][Vp[j]]<< std::endl;
             Vp[j]=Vp[j]+1;
              }
            }
            std::cout <<"交点の総個数Vp["<<j<<"]="<<Vp[j]<< std::endl;

        //一次関数の交点を距離を利用してクラスタリングする
        //交点の(Vx,Vy)の数値を小さいにソート 
        //交点(Vx,Vy)を原点からの距離Vrで並び替える
        double tmpVx,tmpVy,tmpVr;
        for (int vs=0; vs<=Vp[j]; ++vs) {
        for (int vt=vs+1;vt<Vp[j]; ++vt) {
         if (Vrn[j][vs] > Vrn[j][vt]) {
                   tmpVx = Vxn[j][vs];
                   tmpVy = Vyn[j][vs];
                   tmpVr = Vrn[j][vs];

                   Vxn[j][vs]= Vxn[j][vt];
                   Vyn[j][vs]= Vyn[j][vt];
                   Vrn[j][vs]= Vrn[j][vt];

                   Vxn[j][vt] = tmpVx;
                   Vyn[j][vt] = tmpVy;
                   Vrn[j][vt] = tmpVr;
                  }
                  }
              }
        for (int vs=0; vs<=Vp[j]; ++vs) {
          std::cout <<"確認一次関数の交点Vxn["<<j<<"]["<<vs<<"]= "<<Vxn[j][vs]<< std::endl;
             std::cout <<"確認一次関数の交点Vyn["<<j<<"]["<<vs<<"]= "<<Vyn[j][vs]<< std::endl;
             std::cout <<"確認原点から交点までの距離Vrn["<<j<<"]["<<vs<<"]= "<<Vrn[j][vs]<< std::endl;
        }
          
/*
        //並び替えした数値を出力
        //printf("昇順ソートした数値\n");
        //for (int i=0; i<lines1.size(); ++i){ printf("lines[%d][1]=%f\n", i,lines1[i][1]); }
          int c[20],t,j,p;
          double A[20][20][2][4],B;
          c[0]=1,t=0,j=1,p=0;

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
                  A[t][j][0][0]=lines[i+1][0];//代入
                  A[t][j][0][1]=lines[i+1][1];//代入
                  A[t][j][0][2]=dep1[i+1];//代入
                  A[t][j][0][3]=theta[i+1];//代入
                  A[t][j][1][0]=lines[i+1][2];//代入
                  A[t][j][1][1]=lines[i+1][3];//代入
                  A[t][j][1][2]=dep2[i+1];//代入
                  A[t][j][1][3]=theta[i+1];//代入
                  j=j+1;//配列カウント
                  c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
              }
              //前の番号と異なる数値
              else{
              
                if(theta[i+1]-B>0.3){//前の角度との差が0.5より大きい
                   std::cout <<"theta["<<i+1<<"]= 異なる数値 "<< std::endl;
                   std::cout <<"theta["<<i+1<<"]="<<theta[i+1]<< std::endl;
                   std::cout <<"B="<<B<< std::endl;
                   std::cout <<"theta[i+1]-B="<<theta[i+1]-B<< std::endl;

                   t=t+1,j=0;//配列繰り上がり、ｊリセット
                   A[t][j][0][0]=lines[i+1][0];//代入
                   A[t][j][0][1]=lines[i+1][1];//代入
                   A[t][j][0][2]=dep1[i+1];//代入
                   A[t][j][0][3]=theta[i+1];//代入
                   A[t][j][1][0]=lines[i+1][2];//代入
                   A[t][j][1][1]=lines[i+1][3];//代入
                   A[t][j][1][2]=dep2[i+1];//代入
                   A[t][j][1][3]=theta[i+1];//代入
                   B=theta[i+1];//基準の更新
                   j=j+1;//配列カウント
                   c[t]=0;//配列要素数初期値
                  }

                else{//前の角度との差が0.5以下
                   std::cout <<"theta["<<i+1<<"]= ちょっと違う数値= "<<theta[i+1]-B<< std::endl;
                   A[t][j][0][0]=lines[i+1][0];//代入
                   A[t][j][0][1]=lines[i+1][1];//代入
                   A[t][j][0][2]=dep1[i+1];//代入
                   A[t][j][0][3]=theta[i+1];//代入
                   A[t][j][1][0]=lines[i+1][2];//代入
                   A[t][j][1][1]=lines[i+1][3];//代入
                   A[t][j][1][2]=dep2[i+1];//代入
                   A[t][j][1][3]=theta[i+1];//代
                   B=theta[i+1];//基準の更新
                   j=j+1;//配列カウント
                   c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
                  }
              }
            std::cout <<"C["<<t+1<<"]="<< c[t] << std::endl;

          } 
            


          //}
        // }*/
    }//for文 j最後



    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    //cv::imshow(win_depth, img_depth);
    cv::imshow(win_line, img_line);

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
  // image_pub = n.advertise<sensor_msgs::Image>("maskImageData", 10);//realsenseの画像データをパブリッシュ

  

	ros::spin();//トピック更新待機
			
	return 0;
}