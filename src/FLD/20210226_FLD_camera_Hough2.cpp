//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>



ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";
std::string win_fld = "fld";
std::string win_line = "line";
std::string win_line2 = "line2";
std::string win_line3 = "line3";

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
void callback_function(const sensor_msgs::Image::ConstPtr& msg)//画像トピックが更新されたらよばれるやつ//センサーデータがmsgに入る
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    }

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入


//ここに処理項目
	cv::Mat img_src = image;
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_FLD,img_line,img_line2,img_line3;
    double theta[1000],theta0,theta90;

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);
    img_line2 = img_src.clone();
    img_line2 = cv::Scalar(255,255,255);
    img_line3 = img_src.clone();
    img_line3 = cv::Scalar(255,255,255);

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    img_src.copyTo(img_FLD);
    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::line(img_FLD,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青水平
    cv::line(img_FLD,cv::Point(100,0),cv::Point(100,200),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑垂直

    //FLD変換
    std::vector<cv::Vec4f> lines;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines);//特徴線検索


    for(int i = 0; i < lines.size(); i++){
       cv::line(img_FLD,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
       cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }


    cv::cvtColor(img_line, img_gray2, cv::COLOR_RGB2GRAY);

    cv::Canny(img_gray2, img_edge, 200, 200);

    //確率的ハフ変換(元画像・lines)
    std::vector<cv::Vec4i> lines2;
    cv::HoughLinesP(img_edge, lines2, 1, CV_PI/180, 80,30,10);

    for(int i = 0; i < lines2.size(); i++){
       cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);  }
    

    //FLD＋確率ハフ変換抽出線とY軸との角度を求める
    for(int i = 0; i < lines2.size(); i++){

        cv::circle(img_line2,Point(lines2[i][0],lines2[i][1]),5,Scalar(255,0,0),-1);//青点
        cv::circle(img_line2,Point(lines2[i][2],lines2[i][3]),5,Scalar(0,255,0),-1);//緑点

         std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<")"<< std::endl;
         std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<")"<< std::endl;

         //FLD抽出線のy軸との角度を求める
         theta[i]=M_PI-atan2((lines2[i][2]-lines2[i][0]),(lines2[i][3]-lines2[i][1]));
         std::cout <<"FLD+確率ハフ変換の線の傾きl["<<i<<"]("<<theta[i]<<")"<< std::endl;
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
            
             theta[i] = theta[j];
             lines2[i][0] = lines2[j][0];
             lines2[i][1] = lines2[j][1];
             lines2[i][2] = lines2[j][2];
             lines2[i][3] = lines2[j][3];
             
             theta[j] = tmp;
             lines2[j][0] = tmp1x;
             lines2[j][1] = tmp1y;
             lines2[j][2] = tmp2x;
             lines2[j][3] = tmp2y;
            }
        }
    }
    std::cout <<"水平線の傾きl("<<theta0<<")"<< std::endl;
    std::cout <<"垂直線の傾きl("<<theta90<<")"<< std::endl;

    std::cout <<"並び替え後"<<std::endl;
    for (int i=0; i<=lines2.size(); ++i) {
        std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<")"<< std::endl;
        std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<")"<< std::endl;
        std::cout <<"FLD抽出線の傾きl2["<<i<<"]("<<theta[i]<<")"<< std::endl;
    }
    //角度データテキスト化
    for (int i=0; i<=lines2.size(); ++i) {std::cout <<theta[i]<< std::endl;}

    int c[20],t,j,p;
    double A[20][20][2][4],B;
    c[0]=1,t=0,j=1,p=0;

    if(lines2.size()>0){
    A[0][0][0][0]=lines2[0][0];
    A[0][0][0][1]=lines2[0][1];
    A[0][0][0][3]=theta[0];

    A[0][0][1][0]=lines2[0][2];
    A[0][0][1][1]=lines2[0][3];
    A[0][0][1][3]=theta[0];
    

    B=theta[0];}
    std::cout <<"初期値B[0]= "<<B<< std::endl;
    std::cout <<"初期値C[0]= "<<c[0]<< std::endl;

    
    //グルーピング
    for(int i = 0; i < lines2.size(); i++){

        std::cout <<"lines["<<i<<"][0]= "<<lines2[i][0]<< std::endl;
        std::cout <<"lines["<<i<<"][1]= "<<lines2[i][1]<< std::endl;
        std::cout <<"B["<<i<<"]= "<<B<< std::endl;

        //前の番号と同じ数値
        if( B==theta[i+1]){
            std::cout <<"theta["<<i+1<<"]= 同じ数値 "<< std::endl;
            A[t][j][0][0]=lines2[i+1][0];//代入
            A[t][j][0][1]=lines2[i+1][1];//代入
            A[t][j][0][3]=theta[i+1];//代入
            A[t][j][1][0]=lines2[i+1][2];//代入
            A[t][j][1][1]=lines2[i+1][3];//代入
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
             A[t][j][0][0]=lines2[i+1][0];//代入
             A[t][j][0][1]=lines2[i+1][1];//代入
             A[t][j][0][3]=theta[i+1];//代入
             A[t][j][1][0]=lines2[i+1][2];//代入
             A[t][j][1][1]=lines2[i+1][3];//代入
             A[t][j][1][3]=theta[i+1];//代入
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[t]=0;//配列要素数初期値
            }

          else{//前の角度との差が0.5以下
             std::cout <<"theta["<<i+1<<"]= ちょっと違う数値= "<<theta[i+1]-B<< std::endl;
             A[t][j][0][0]=lines2[i+1][0];//代入
             A[t][j][0][1]=lines2[i+1][1];//代入
             A[t][j][0][3]=theta[i+1];//代入
             A[t][j][1][0]=lines2[i+1][2];//代入
             A[t][j][1][1]=lines2[i+1][3];//代入
             A[t][j][1][3]=theta[i+1];//代
             B=theta[i+1];//基準の更新
             j=j+1;//配列カウント
             c[t]=c[t]+1;//要素数（同じ数値は何個あるか）
            }
        }
      std::cout <<"C["<<t+1<<"]="<< c[t] << std::endl;
    } 

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


    }//for文 j最後

    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_fld, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_fld, img_FLD);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_line, img_line);
    cv::imshow(win_line2, img_line2);
    cv::imshow(win_line3, img_line3);
	cv::waitKey(1);
   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/robot1/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録

	ros::spin();//トピック更新待機
			
	return 0;
}