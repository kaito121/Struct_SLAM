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
std::string win_depth = "depth";//深度画像
std::string win_edge = "edge";
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_line = "line";//FLDの線を表示
std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_line3 = "line3";//分類分けしたタテ、ヨコ、ナナメの線を表示
std::string win_line4 = "line4";//分類分けしたタテ、ヨコの線を表示
std::string win_line5 = "line5";//分類分けしたタテ、ヨコの線を表示

std::string win_dstP = "dstP";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
//std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像
std::string win_graph = "graph";//グラフ作成


using namespace std;
using namespace cv;
using Eigen::MatrixXd;

int linesu=400;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検bool reset = true;//初回プログラム用（特徴点検出&特徴点再検出用)出を行うように、初期値を TRUE にする
bool reset = true;
bool yoko_histgram = false;//ヨコ線のヒストグラム(true実行)
bool tate_histgram = false;//タテ線のヒストグラム(true実行)

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

int kaisu,optkaisu;//行列初期設定用変数

double Average_tate_theta,Average_tate_theta_Y;//最大クラスタ内の平均角度を求める

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

    cv::Mat img_src = bridgeImage->image.clone();//image変数に変換した画像データを代入
    cv::Mat img_depth = bridgedepthImage->image.clone();//image変数に変換した画像データを代入


    //ここに処理項目
	
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_line,img_line2,img_line3,img_line4,img_line5,img_graph;


    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_line = img_src.clone();
    img_line = cv::Scalar(255,255,255);
    img_line2 = img_src.clone();
    img_line2 = cv::Scalar(255,255,255);
    img_line3 = img_src.clone();
    img_line3 = cv::Scalar(255,255,255);
    img_line4 = img_src.clone();
    img_line4 = cv::Scalar(255,255,255);
    img_line5 = img_src.clone();
    img_line5 = cv::Scalar(255,255,255);
    img_graph= img_src.clone();
    img_graph= cv::Scalar(255,255,255);
    cv::line(img_graph,cv::Point(0,480),cv::Point(640,480),cv::Scalar(0,0,0), 3, cv::LINE_AA);//X軸 
    cv::line(img_graph,cv::Point(0,480),cv::Point(0,0),cv::Scalar(0,0,0), 3, cv::LINE_AA);//Y軸 
    cv::line(img_graph,cv::Point(180*3,0),cv::Point(180*3,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(170*3,0),cv::Point(170*3,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(100*3,0),cv::Point(100*3,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(90*3,0),cv::Point(90*3,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(80*3,0),cv::Point(80*3,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(10*3,0),cv::Point(10*3,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
    
    
    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    //cv::line(img_dst,cv::Point(0,100),cv::Point(200,100),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青水平
    //cv::line(img_dst,cv::Point(100,0),cv::Point(100,200),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑垂直

    //FLD変換
    std::vector<cv::Vec4f> lines2;
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines2);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines2.size(); i++){
       //cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_line,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }
    double theta[lines2.size()],theta0,theta90;
    float dep1[lines2.size()],dep2[lines2.size()];

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    

    cv::cvtColor(img_line, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 125, 255);
    //cv::Canny(img_gray2, img_edge, 200, 200);

    
    //FLD抽出線とY軸との角度を求める+三次元距離データの結合
    std::cout <<"並び替え前"<<std::endl;
    int ksk=0;
    for(int i =  0; i < lines2.size(); i++){

        dep1[i]= img_depth.at<float>(lines2[i][0],lines2[i][1]);//点の三次元距離データ取得
        dep2[i]= img_depth.at<float>(lines2[i][2],lines2[i][3]);
        //cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);

        if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
            //cv::circle(img_line2,Point(lines2[i][0],lines2[i][1]),5,Scalar(255,0,0),-1);//青点
            //cv::circle(img_line2,Point(lines2[i][2],lines2[i][3]),5,Scalar(0,255,0),-1);//緑点
            lines2[ksk][0]=lines2[i][0];//ここで距離０を除いてる
            lines2[ksk][1]=lines2[i][1];
            lines2[ksk][2]=lines2[i][2];
            lines2[ksk][3]=lines2[i][3];
            dep1[ksk]=dep1[i];
            dep2[ksk]=dep2[i];

            //std::cout <<"pt1[ksk="<<ksk<<"]("<<lines2[ksk][0]<<","<<lines2[ksk][1]<<","<<dep1[ksk]/1000<<"[m])"<< std::endl;
            //std::cout <<"pt2[ksk="<<ksk<<"]("<<lines2[ksk][2]<<","<<lines2[ksk][3]<<","<<dep2[ksk]/1000<<"[m])"<< std::endl;

             //FLD抽出線のy軸との角度を求める
             theta[ksk]=M_PI-atan2((lines2[ksk][2]-lines2[ksk][0]),(lines2[ksk][3]-lines2[ksk][1]));
             std::cout <<"FLDの線の傾きl["<<ksk<<"]("<<theta[ksk]<<")"<< std::endl;

             //cv::line(img_line2,cv::Point(lines2[i][0],lines2[i][1]),cv::Point(lines2[i][2],lines2[i][3]),cv::Scalar(0,255,255), 2, cv::LINE_AA);//黄色の線（距離データ取得可能）
             ksk=ksk+1;
        }
    }

    //thetaの数値を小さいにソート
    double tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0;
    for (int i=0; i<ksk; ++i) {
       for (int j=i+1;j<ksk; ++j) {
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
    //for (int i=0; i<ksk; ++i) {
    //    std::cout <<"pt1["<<i<<"]("<<lines2[i][0]<<","<<lines2[i][1]<<")"<< std::endl;
    //    std::cout <<"pt2["<<i<<"]("<<lines2[i][2]<<","<<lines2[i][3]<<")"<< std::endl;
    //    std::cout <<"FLD抽出線の傾きl2["<<i<<"]("<<theta[i]<<")"<< std::endl;
    //}
    //角度データテキスト化
    //for (int i=0; i<=lines2.size(); ++i) {std::cout <<theta[i]<< std::endl;}

    //線が縦線か横線か見極め２つに分類する（縦線0,横線１)

    int yokot,tatet,p,yokoyouso,tateyouso;
    double lines2X,lines2Y,yokolines2[lines2.size()][5],tatelines2[lines2.size()][5],yokotheta[lines2.size()],tatetheta[lines2.size()];
    double yokoZ1[lines2.size()],yokoZ2[lines2.size()],tateZ1[lines2.size()],tateZ2[lines2.size()];
    double yokothetal[lines2.size()],tatethetal[lines2.size()],yokolc[lines2.size()],tatelc[lines2.size()],yokol[lines2.size()][4],tatel[lines2.size()][4];
    double datat[ksk],datay[ksk];//ヒストグラムデータ用
    int clusCY,clusCT,clusy[ksk],clust[ksk],MAXY=0,MAXT=0;//クラスタリング用
    double clusR;
    double yokoclusy[100][200][5],tateclust[100][200][5];
    double Average_yoko_theta[ksk];
    double MINI_theta=100,mini_theta;
    int CLUSTER[ksk],clus_no=0,YOKO_CLUST;
    int nanamet=0,nanamey=0,NANAME_Line[100];
  
    //int yokocount[200],tatecount[200],yokoko=0,tatete=0;//ヒストグラム用
    //double yokotemp=0,tatetemp=0,yokohist[100],tatehist[100];
    
    yokot=0,tatet=0,p=0,yokoyouso=0,tateyouso=0;

    for (int j=0; j< ksk; ++j) {
        lines2X=abs(lines2[j][0]-lines2[j][2]);//傾きを調べる（x成分)
        lines2Y=abs(lines2[j][1]-lines2[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(lines2X>lines2Y){
            std::cout <<"yoko(lines2X>lines2Y)="<<lines2X<<">"<<lines2Y<< std::endl;
            std::cout <<"theta["<<j<<"](ヨコ)="<<theta[j]<< std::endl;

            yokolines2[yokoyouso][0]=lines2[j][0];//(x成分)
            yokolines2[yokoyouso][1]=lines2[j][1];//(y成分)
            yokolines2[yokoyouso][2]=lines2[j][2];//(x成分)
            yokolines2[yokoyouso][3]=lines2[j][3];//(y成分)
            yokolines2[yokoyouso][4]=theta[j];

            //yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=dep1[j];
            yokoZ2[yokoyouso]=dep2[j];
            cv::line(img_line3,cv::Point(yokolines2[yokoyouso][0],yokolines2[yokoyouso][1]),cv::Point(yokolines2[yokoyouso][2],yokolines2[yokoyouso][3]),cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(img_line2,cv::Point(yokolines2[yokoyouso][0],yokolines2[yokoyouso][1]),cv::Point(yokolines2[yokoyouso][2],yokolines2[yokoyouso][3]),cv::Scalar(0,255,0), 2, cv::LINE_AA);

            yokotheta[yokoyouso]=theta[j]*180/M_PI;//deg表示化
            if(yokotheta[yokoyouso]>=180){
                yokotheta[yokoyouso]=yokotheta[yokoyouso]-180;
                yokolines2[yokoyouso][4]=yokolines2[yokoyouso][4]-M_PI;
            }
            std::cout <<"yokotheta["<<yokoyouso<<"]="<<yokotheta[yokoyouso]<< std::endl;
            if(yokotheta[yokoyouso]>=90){
                yokotheta[yokoyouso]=180-yokotheta[yokoyouso];
            }
            std::cout <<"yokotheta["<<yokoyouso<<"](クラスタリング用)="<<yokotheta[yokoyouso]<< std::endl;

            //yokocount[yokoyouso]=0;//ヒストグラム用
            clusy[yokoyouso]=0;//クラスタリング用
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)

        }
        //縦線に分類
        else{
            if(lines2Y>lines2X){
                std::cout <<"tate(lines2Y>lines2X)="<<lines2Y<<">"<<lines2X<< std::endl;
                std::cout <<"theta["<<j<<"](タテ)="<<theta[j]<< std::endl;

                tatelines2[tateyouso][0]=lines2[j][0];
                tatelines2[tateyouso][1]=lines2[j][1];
                tatelines2[tateyouso][2]=lines2[j][2];
                tatelines2[tateyouso][3]=lines2[j][3];
                tatelines2[tateyouso][4]=theta[j];


                //tatetheta[tateyouso]=theta[j];
                tateZ1[tateyouso]=dep1[j];
                tateZ2[tateyouso]=dep2[j];
                cv::line(img_line3,cv::Point(tatelines2[tateyouso][0], tatelines2[tateyouso][1]),cv::Point(tatelines2[tateyouso][2],tatelines2[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                cv::line(img_line2,cv::Point(tatelines2[tateyouso][0], tatelines2[tateyouso][1]),cv::Point(tatelines2[tateyouso][2],tatelines2[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);

                tatetheta[tateyouso]=theta[j]*180/M_PI;//deg表示化
                //tatetheta[tateyouso]=(theta[j]*360)/(2*M_PI);//deg表示化
                if(tatetheta[tateyouso]>=180){
                    tatetheta[tateyouso]=tatetheta[tateyouso]-180;
                    tatelines2[tateyouso][4]=tatelines2[tateyouso][4]-M_PI;
                    }
                std::cout <<"tatetheta["<<tateyouso<<"]="<<tatetheta[tateyouso]<< std::endl;
                
                if(tatetheta[tateyouso]>=90){
                    tatetheta[tateyouso]=180-tatetheta[tateyouso];
                }
                std::cout <<"tatetheta["<<tateyouso<<"](クラスタリング用)="<<tatetheta[tateyouso]<< std::endl;
                //クラスタリング時に最大個数を持つ縦線のクラスタが２つ存在してしまうため、90度で反転させてクラスタリング処理を行う。
                //例(θ=0~10,170~180→180-170=10)

                //tatecount[tateyouso]=0;//ヒストグラム用
                clust[tateyouso]=0;//クラスタリング用
                tateyouso=tateyouso+1;//縦線に分類されたグループ数(tateyouso)
            }
        }
    }
    //クラスタリング-----------------------------------------------------------------------------------------------------------------------
    //タテ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCT:クラスタ数,Clust[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    //その他の線をナナメ線として確保
    clusR=1.3,clusCT=0,Average_tate_theta=0;
    //要素数が0の時は実行しない
    if(tateyouso==0){
        std::cout <<"実行しない--------------tateyouso="<<tateyouso<< std::endl;
        tate_histgram = false;//タテ線のヒストグラムを実行しない
    }
    else{
        tate_histgram = true;//タテ線のヒストグラムを実行
        for (int j=0; j< tateyouso; ++j) {
            if(tateyouso==1){//要素がひとつしかないとき比較ができない
                std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                std::cout <<"クラスタ番号clusCT="<<clusCT<< std::endl;//クラスタ数
                std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(tatetheta[j]*3, 240), 3, Scalar(0,30*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                tateclust[0][clust[0]][0]=tatelines2[j][0];//(x成分)
                tateclust[0][clust[0]][1]=tatelines2[j][1];//(y成分)
                tateclust[0][clust[0]][2]=tatelines2[j][2];//(x成分)
                tateclust[0][clust[0]][3]=tatelines2[j][3];//(y成分)
                tateclust[0][clust[0]][4]=tatelines2[j][4];//角度
                clust[MAXT]=1;
            }

            else{
                cv::circle(img_graph, cv::Point(tatetheta[j]*3, 240), 3, Scalar(0,30*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        std::cout <<"初回動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines2[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines2[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines2[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines2[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines2[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,30*clusCT,255), 3, cv::LINE_AA);

                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新

                    }
                    //クラスタリング範囲外
                    else{
                        std::cout <<"初回動作範囲外j="<<j<<",clusCT="<<clusCT<<"----------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines2[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines2[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines2[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines2[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines2[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,30*clusCT,255), 3, cv::LINE_AA);
                        MAXT=clusCT;

                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //中間動作
                if(j!=0&&j+1!=tateyouso){
                    std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //後方クラスタリング半径範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        std::cout <<"中間動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines2[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines2[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines2[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines2[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines2[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,30*clusCT,255), 3, cv::LINE_AA);

                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //後方クラスタリング半径範囲外
                    else{
                        std::cout <<"中間動作範囲外j="<<j<<",clusCT="<<clusCT<<"--------------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines2[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines2[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines2[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines2[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines2[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,30*clusCT,255), 3, cv::LINE_AA);

                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //最終動作
                if(j+1==tateyouso){
                    std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j-1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    std::cout <<"最終動作j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    tateclust[clusCT][clust[clusCT]][0]=tatelines2[j][0];//(x成分)
                    tateclust[clusCT][clust[clusCT]][1]=tatelines2[j][1];//(y成分)
                    tateclust[clusCT][clust[clusCT]][2]=tatelines2[j][2];//(x成分)
                    tateclust[clusCT][clust[clusCT]][3]=tatelines2[j][3];//(y成分)
                    tateclust[clusCT][clust[clusCT]][4]=tatelines2[j][4];//角度

                    cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                    cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,30*clusCT,255), 3, cv::LINE_AA);
                    std::cout <<"最終:最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<< std::endl;
                }

                if(clust[clusCT]>=clust[MAXT]){ MAXT=clusCT; }//最大クラスタをキープする
                //else{ 
                //    NANAME_Line[nanamet]=clusCT;//最大出ないクラスタをナナメ線グループとして保管する
                //    nanamet=nanamet+1;
                //}
                std::cout <<"最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<<"\n"<< std::endl;
            }
        }
        //一次関数を描写するプログラム
        for (int j=0; j<clust[MAXT]; ++j) {
            std::cout <<"j="<< j<< std::endl;
            std::cout <<"clust["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"tateclust["<<MAXT<<"]["<<j<<"][0]="<<tateclust[MAXT][j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数
            tatethetal[j]=(M_PI/2)-(M_PI-atan2((tateclust[MAXT][j][2]-tateclust[MAXT][j][0]),(tateclust[MAXT][j][3]-tateclust[MAXT][j][1])));
            tatelc[j]=(tateclust[MAXT][j][2]-tateclust[MAXT][j][0])*(tateclust[MAXT][j][2]-tateclust[MAXT][j][0])+(tateclust[MAXT][j][3]-tateclust[MAXT][j][1])*(tateclust[MAXT][j][3]-tateclust[MAXT][j][1]);
            tatel[j][0]=tateclust[MAXT][j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*100;//X1座標
            tatel[j][1]=tateclust[MAXT][j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*100;//Y1座標
            tatel[j][2]=tateclust[MAXT][j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*-100;//X2座標
            tatel[j][3]=tateclust[MAXT][j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*-100;//Y2座標
            //std::cout <<"直線の角度1θ="<< tatethetal[j] << std::endl;
            //std::cout <<"直線座標1=("<< tatel[j][0] <<","<<tatel[j][1]<<")"<< std::endl;
            //std::cout <<"直線座標2=("<< tatel[j][2] <<","<<tatel[j][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);

            datat[j]=tateclust[MAXT][clust[j]][4];

            //最大クラスタ内の平均角度を求める
            Average_tate_theta=Average_tate_theta+datat[j];
        }

        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta/(clust[MAXT]-1)),380+100*cos(Average_tate_theta/(clust[MAXT]-1))),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta/(clust[MAXT]-1)<<"\n"<< std::endl;

         MatrixXd R(2,2);
         R(0,0)=cos(M_PI/2);
         R(0,1)=-sin(M_PI/2);
         R(1,0)=sin(M_PI/2);
         R(1,1)=cos(M_PI/2);

        MatrixXd n(2,1);
        MatrixXd N(2,1);//Nは2行L列の行列
        N(0,0)=100-100*sin(Average_tate_theta/(clust[MAXT]-1));
        N(1,0)=380+100*cos(Average_tate_theta/(clust[MAXT]-1));
        MatrixXd P0(2,1);//推定交点
        P0(0,0)=(100+(100-100*sin(Average_tate_theta/(clust[MAXT]-1))))/2;
        P0(1,0)=(380+(380+100*cos(Average_tate_theta/(clust[MAXT]-1))))/2;

        n=R*(P0-N);//直線を90度回転させたベクトルn


        cv::line(img_graph,cv::Point(P0(0,0),P0(1,0)),cv::Point(P0(0,0)-n(0,0),P0(1,0)-n(1,0)),cv::Scalar(255,0,255), 3, cv::LINE_AA);//法線ベクトル

        //縦線の平均方向の法線ベクトルとy軸との角度を求める
        Average_tate_theta_Y=M_PI-atan2(-n(0,0),-n(1,0));
        std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<<"\n"<< std::endl;

        //次はこの法線ベクトルの角度に最も近いヨコ線のクラスタを求める
        //求める方法としては各クラスタの平均角度を求め、その平均角度と比較して最も比較の値が小さいやつをヨコ線のクラスタとする。
        //ただしその比較値がある値以上だった場合はヨコ線の方向は更新せず前ステップでのヨコ線の方向を使う。

        //縦線が検出されなかった場合もまた同様に縦線の方向は前ステップでの方向を利用する
    }

    std::cout <<"\n"<< std::endl;

    //ヨコ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCY:クラスタ数,Clusy[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    clusR=1.5,clusCY=0;
    //要素数が0の時は実行しない
    if(yokoyouso==0){
        std::cout <<"実行しない--------------yokoyouso="<<yokoyouso<< std::endl;
        yoko_histgram = false;//ヨコ線のヒストグラムを実行しない
    }
    else{
        yoko_histgram = true;//ヨコ線のヒストグラムを実行
        for (int j=0; j< yokoyouso; ++j) {
            if(yokoyouso==1){//要素がひとつしかないとき比較ができない
                std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                std::cout <<"クラスタ番号clusCY="<<clusCY<< std::endl;//クラスタ数
                std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(yokotheta[j]*3, 240), 3, Scalar(30*clusCY,255,0), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                yokoclusy[0][clusy[0]][0]=yokolines2[j][0];//(x成分)
                yokoclusy[0][clusy[0]][1]=yokolines2[j][1];//(y成分)
                yokoclusy[0][clusy[0]][2]=yokolines2[j][2];//(x成分)
                yokoclusy[0][clusy[0]][3]=yokolines2[j][3];//(y成分)
                yokoclusy[0][clusy[0]][4]=yokolines2[j][4];
                clusy[MAXY]=1;
            }

            else{
                cv::circle(img_graph, cv::Point(yokotheta[j]*3, 240), 3, Scalar(30*clusCY,255,0), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        std::cout <<"初回動作範囲内j="<<j<<",clusCY="<<clusCY<<",clusy[clusCY]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines2[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines2[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines2[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines2[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines2[j][4];// 角度
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(30*clusCY,255,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す(初回動作)

                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        std::cout <<"初回動作範囲外j="<<j<<",clusCY="<<clusCY<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines2[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines2[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines2[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines2[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines2[j][4];//角度
                        
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(30*clusCY,255,0), 3, cv::LINE_AA);
                        MAXY=clusCY;

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求める(初回動作クラスタリング範囲外なのでデータが１つしかない)
                        std::cout <<"初回動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での平均値初期化
                    }
                }
                //中間動作
                if(j!=0&&j+1!=yokoyouso){
                    std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //後方範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        std::cout <<"中間動作範囲内j="<<j<<",clusCY="<<clusCY<<",clusy[clusCY]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines2[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines2[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines2[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines2[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines2[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(30*clusCY,255,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //後方範囲外
                    else{
                        std::cout <<"中間動作範囲外j="<<j<<",clusCY="<<clusCY<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines2[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines2[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines2[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines2[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines2[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(30*clusCY,255,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        
                        if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                        }
                        else{//クラスタの要素が一つしかない時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均=要素(要素がひとつしかないから=平均)
                        }
                        std::cout <<"中間動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;
                        //要素数が５個以上のクラスタをキープする
                        if(clusy[clusCY]>=5){
                            std::cout <<"要素数が５個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                            CLUSTER[clus_no]=clusCY;
                            clus_no=clus_no+1;
                        }

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での平均値初期化
                    }
                }
                //最終動作
                if(j+1==yokoyouso){
                    std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j-1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    std::cout <<"最終動作j="<<j<<",clusCY="<<clusCY<<",clusy[clusCY]="<<clusy[clusCY]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    yokoclusy[clusCY][clusy[clusCY]][0]=yokolines2[j][0];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][1]=yokolines2[j][1];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][2]=yokolines2[j][2];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][3]=yokolines2[j][3];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][4]=yokolines2[j][4];//角度

                    cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                    cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(30*clusCY,255,0), 3, cv::LINE_AA);

                    Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                    if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                    }
                    else{//クラスタの要素が一つしかない時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均を求める
                    }
                    std::cout <<"最終動作:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<< std::endl;
                    //要素数が５個以上のクラスタをキープする
                    if(clusy[clusCY]>=5){
                        std::cout <<"要素数が５個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                        CLUSTER[clus_no]=clusCY;
                    }
                }

                //if(clusy[clusCY]>=clusy[MAXY]){ MAXY=clusCY; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXY=clusCY="<<MAXY<<",最大クラスタの内部個数clusy[MAXY]="<<clusy[MAXY]<< std::endl;
            }
        }
        for(int h=0;h<clus_no;h++){
            std::cout <<"要素数が５個以上のクラスタ  clusCY=CLUSTER[clus_no="<<h<<"]="<<CLUSTER[h]<<",Average_yoko_theta["<<CLUSTER[h]<<"]="<<Average_yoko_theta[CLUSTER[h]]<< std::endl;
            mini_theta=abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]]);
            if(MINI_theta>mini_theta){
                MINI_theta=mini_theta;
                YOKO_CLUST=CLUSTER[h];
            }
        }
        std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<< std::endl;
        std::cout <<"タテの法線ベクトルと最も並列なヨコ線クラスタ clusCY=YOKO_CLUST="<<YOKO_CLUST<<",Average_yoko_theta_Y="<<Average_yoko_theta[YOKO_CLUST]<< std::endl;
        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_yoko_theta[YOKO_CLUST]),380+100*cos(Average_yoko_theta[YOKO_CLUST])),cv::Scalar(255,0,0), 3, cv::LINE_AA);


        //一次関数を描写するプログラム
        for(int j = 0; j <clusy[YOKO_CLUST]; j++){
            std::cout <<"j="<< j<< std::endl;
            std::cout <<"clusy["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"yokoclusy["<<YOKO_CLUST<<"]["<<j<<"][0]="<<yokoclusy[YOKO_CLUST][j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数(線を延長してる)
            yokothetal[j]=(M_PI/2)-(M_PI-atan2((yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0]),(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])));
            yokolc[j]=(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])*(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])+(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])*(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1]);
            yokol[j][0]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*100;//X1座標
            yokol[j][1]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*100;//Y1座標
            yokol[j][2]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*-100;//X2座標
            yokol[j][3]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*-100;//Y2座標
            std::cout <<"直線の角度1θ="<< yokothetal[j] << std::endl;
            std::cout <<"直線座標1=("<< yokol[j][0] <<","<<yokol[j][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< yokol[j][2] <<","<<yokol[j][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);

            datay[j]=yokoclusy[YOKO_CLUST][j][4];
        }



        //std::cout <<"クラスタ数で並び替え前"<< std::endl;
        //for(int i = 0; i <  clusCY; i++){
        //    std::cout <<"clusCY="<<i<<",clusy["<<i<<"]="<<clusy[i]<< std::endl;
        //}


        /*std::cout <<"要素数が５個以上のクラスタ\n"<< std::endl;
        for(int h=0;h<clus_no;h++){
            for(int i=0;i<=CLUSTER[h];i++){
                for(int j=0;j<=clusy[i];j++){
                std::cout <<"yokoclusy[clus_no="<<i<<"][clusy[clus_no]="<<j<<"][0]="<< yokoclusy[i][j][0]<<std::endl;
                std::cout <<"yokoclusy[clus_no="<<i<<"][clusy[clus_no]="<<j<<"][1]="<< yokoclusy[i][j][1]<<std::endl;
                std::cout <<"yokoclusy[clus_no="<<i<<"][clusy[clus_no]="<<j<<"][2]="<< yokoclusy[i][j][2]<<std::endl;
                std::cout <<"yokoclusy[clus_no="<<i<<"][clusy[clus_no]="<<j<<"][3]="<< yokoclusy[i][j][3]<<std::endl;
                std::cout <<"yokoclusy[clus_no="<<i<<"][clusy[clus_no]="<<j<<"][4]="<< yokoclusy[i][j][4]<<std::endl;


                //std::cout <<"yokoclusy[clus_no="<<CLUSTER[h]<<"][clusy[clus_no]="<<clusy[CLUSTER[i]]<<"][0]="<< yokoclusy[CLUSTER[i]][clusy[CLUSTER[i]]][0]<<std::endl;
                //std::cout <<"yokoclusy[clus_no="<<CLUSTER[h]<<"][clusy[clus_no]="<<clusy[CLUSTER[i]]<<"][1]="<< yokoclusy[CLUSTER[i]][clusy[CLUSTER[i]]][1]<<std::endl;
                //std::cout <<"yokoclusy[clus_no="<<CLUSTER[h]<<"][clusy[clus_no]="<<clusy[CLUSTER[i]]<<"][2]="<< yokoclusy[CLUSTER[i]][clusy[CLUSTER[i]]][2]<<std::endl;
                //std::cout <<"yokoclusy[clus_no="<<CLUSTER[h]<<"][clusy[clus_no]="<<clusy[CLUSTER[i]]<<"][3]="<< yokoclusy[CLUSTER[i]][clusy[CLUSTER[i]]][3]<<std::endl;
                //std::cout <<"yokoclusy[clus_no="<<CLUSTER[h]<<"][clusy[clus_no]="<<clusy[CLUSTER[i]]<<"][4]="<< yokoclusy[CLUSTER[i]][clusy[CLUSTER[i]]][4]<<std::endl;
                }
            }
        }*/
        
        /*std::cout <<"クラスタ数で並び替え前"<< std::endl;
        for(int i = 0; i <  clusCY; i++){
            std::cout <<"clusCY="<<i<<",clusy["<<i<<"]="<<clusy[i]<< std::endl;
        }
        double clusy_tmp,clusCY_tmp,clusCY_all=clusCY;
        //クラスタ番号:clusCY,クラスタ内の要素数:clusy[clusCY],クラスタ内の要素:yokoclusy[clusCY][clusy[clusCY]][0]
        //要素数が多い順に並び替える
        //データを大きさの順に並べ替え
        for(int i = 1; i <  clusCY_all; i++){
            for(int j = 0; j < clusCY_all - i; j++){
                if(clusy[j] > clusy[j + 1]){
                    clusy_tmp = clusy[j];
                    clusCY_tmp = clusCY;
                    clusy[j] = clusy[j + 1];
                    clusy[j + 1] = clusy_tmp;
                }
            }
        }
        std::cout <<"クラスタ数で並び替え後"<< std::endl;
        for(int i = 0; i <  clusCY; i++){
            std::cout <<"clusCY="<<i<<",clusy["<<i<<"]="<<clusy[i]<< std::endl;
        }*/
        /*//一次関数を描写するプログラム
        for (int j=0; j<clusy[MAXY]; ++j) {
            std::cout <<"j="<< j<< std::endl;
            std::cout <<"clusy["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"yokoclusy["<<MAXY<<"]["<<j<<"][0]="<<yokoclusy[MAXY][j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数
            yokothetal[j]=(M_PI/2)-(M_PI-atan2((yokoclusy[MAXY][j][2]-yokoclusy[MAXY][j][0]),(yokoclusy[MAXY][j][3]-yokoclusy[MAXY][j][1])));
            yokolc[j]=(yokoclusy[MAXY][j][2]-yokoclusy[MAXY][j][0])*(yokoclusy[MAXY][j][2]-yokoclusy[MAXY][j][0])+(yokoclusy[MAXY][j][3]-yokoclusy[MAXY][j][1])*(yokoclusy[MAXY][j][3]-yokoclusy[MAXY][j][1]);
            yokol[j][0]=yokoclusy[MAXY][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*10;//X1座標
            yokol[j][1]=yokoclusy[MAXY][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*10;//Y1座標
            yokol[j][2]=yokoclusy[MAXY][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*-10;//X2座標
            yokol[j][3]=yokoclusy[MAXY][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*-10;//Y2座標
            std::cout <<"直線の角度1θ="<< yokothetal[j] << std::endl;
            std::cout <<"直線座標1=("<< yokol[j][0] <<","<<yokol[j][1]<<")"<< std::endl;
            std::cout <<"直線座標2=("<< yokol[j][2] <<","<<yokol[j][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);

            datay[j]=yokoclusy[MAXY][j][4];
        }*/
    }
    //ナナメ線ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
    //一次関数を描写するプログラム
    /*for(int i=0; i<nanamet;i++){
        std::cout <<"nanamet="<<nanamet<<",i="<<i<< std::endl;
        std::cout <<"NANAME_Line["<<i<<"]="<<NANAME_Line[i]<< std::endl;
        for (int j=0; j<clust[NANAME_Line[i]]; ++j) {
            std::cout <<"clust[NANAME_Line["<<i<<"]]="<<clust[NANAME_Line[i]]<<",j="<< j<< std::endl;//クラスタの内部個数
            //std::cout <<"tateclust["<<NANAME_Line[i]<<"]["<<j<<"][0]="<<tateclust[NANAME_Line[i]][j][0]<< std::endl;//確認用
            //座標から一次関数を引く関数
            tatethetal[j]=(M_PI/2)-(M_PI-atan2((tateclust[NANAME_Line[i]][j][2]-tateclust[NANAME_Line[i]][j][0]),(tateclust[NANAME_Line[i]][j][3]-tateclust[NANAME_Line[i]][j][1])));
            tatelc[j]=(tateclust[NANAME_Line[i]][j][2]-tateclust[NANAME_Line[i]][j][0])*(tateclust[NANAME_Line[i]][j][2]-tateclust[NANAME_Line[i]][j][0])+(tateclust[NANAME_Line[i]][j][3]-tateclust[NANAME_Line[i]][j][1])*(tateclust[NANAME_Line[i]][j][3]-tateclust[NANAME_Line[i]][j][1]);
            tatel[j][0]=tateclust[NANAME_Line[i]][j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*100;//X1座標
            tatel[j][1]=tateclust[NANAME_Line[i]][j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*100;//Y1座標
            tatel[j][2]=tateclust[NANAME_Line[i]][j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*-100;//X2座標
            tatel[j][3]=tateclust[NANAME_Line[i]][j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*-100;//Y2座標
            //std::cout <<"直線の角度1θ="<< tatethetal[j] << std::endl;
            //std::cout <<"直線座標1=("<< tatel[j][0] <<","<<tatel[j][1]<<")"<< std::endl;
            //std::cout <<"直線座標2=("<< tatel[j][2] <<","<<tatel[j][3]<<")\n"<< std::endl;
            cv::line(img_line4,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
            //datat[j]=tateclust[NANAME_Line[i]][clust[j]][4];
            //最大クラスタ内の平均角度を求める
            //Average_tate_theta=Average_tate_theta+datat[j];
        }
    }*/



    
    /*//ヨコ線の角度のヒストグラムから平行線を見つける---------------------------------------------------
    // 変数の初期化
    if(yoko_histgram == true){
        double histoy_i = 0;
        double histoy_j = 0;
        double histoy_tmp = 0;
        double histoy_med = 0;
        double histoy_renge = 0;
        double  histogram[2 + 1];  // ヒストグラム
    
        //データを大きさの順に並べ替え
        for(int i = 1; i <  clusy[MAXY]; i++){
            for(int j = 0; j < clusy[MAXY] - i; j++){
                if(datay[j] > datay[j + 1]){
                    histoy_tmp = datay[j];
                    datay[j] = datay[j + 1];
                    datay[j + 1] = histoy_tmp;
                }
            }
        }

        // メジアンを求める
        // データ数が奇数個の場合
        if(clusy[MAXY] % 2 == 1){ histoy_med = datay[(clusy[MAXY] - 1) / 2]; }  // メジアン

        // データ数が偶数の場合
        else{ histoy_med = (datay[(clusy[MAXY] / 2) - 1] + datay[clusy[MAXY] / 2]) / 2.0; } // メジアン

        // レンジを求める
        histoy_renge = datay[clusy[MAXY] - 1] - datay[0] + 0.0;  // 範囲

        double modey;              // これまでに調べた中での最頻値
        int count_maxy = 0;     // これまでに調べた登場回数の中で最大のもの
        for(int i = 1; i < clusy[MAXY]; i++){
            std::cout << datay[i] << std::endl;
            double valuey =  datay[i];
            int county = 1;

            // ソートされているのだから、
            // 同じ値があるとすれば、後続に連続して並んでいるはず。
            // その回数をカウントする。
            for (i = i + 1; i < clusy[MAXY]; ++i) {
                if (valuey ==  datay[i]) {
                    ++county;
                }
                else {
                    // 違う値が登場したら終わり
                    break;
                }
            }

            // これまでの最大の登場回数よりも多かったら、更新する
            if (count_maxy < county) {
                count_maxy = county;
                modey = valuey;
            }
        }
        for(int i = 1; i < clusy[MAXY]; i++){
            std::cout << datay[i] << std::endl;}
        std::cout <<"ヨコ線_  最瀕値は"<<modey<< std::endl;
        std::cout <<"ヨコ線_メジアンは"<<histoy_med<< std::endl;
        std::cout <<"ヨコ線_範囲    は"<<histoy_renge<< std::endl;

        cv::line(img_graph,cv::Point(100,380),cv::Point(100+100*sin(modey),380-100*cos(modey)),cv::Scalar(0,255,0), 3, cv::LINE_AA);
        cv::line(img_graph,cv::Point(100,380),cv::Point(100+100*sin(histoy_med),380-100*cos(histoy_med)),cv::Scalar(0,255,100), 3, cv::LINE_AA);


        //外積処理
        cv::Mat yoko_Mat = (cv::Mat_<double>(3, 1) << 
        100*sin(modey),
        -100*cos(modey), 
        1);
    }

    if(tate_histgram == true){
        //縦線の角度のヒストグラムから平行線を見つける-----------------------------------------------------------------------
        // 変数の初期化
        int histot_i = 0;
        int histot_j = 0;
        int histot_tmp = 0;
        double histot_med = 0;
        double histot_renge = 0;

        //データを大きさの順に並べ替え
        for(int i = 1; i < clust[MAXT]; i++){
            for(int j = 0; j < clust[MAXT] - i; j++){
                if(datat[j] > datat[j + 1]){
                    histot_tmp = datat[j];
                    datat[j] = datat[j + 1];
                    datat[j + 1] = histot_tmp;
                }
            }
        }

        // メジアンを求める
        // データ数が奇数個の場合
        if(clust[MAXT] % 2 == 1){ histot_med = datat[(clust[MAXT] - 1) / 2]; }  // メジアン

        // データ数が偶数の場合
        else{ histot_med = (datat[(clust[MAXT] / 2) - 1] + datat[clust[MAXT] / 2]) / 2.0; } // メジアン

        // レンジを求める
        histot_renge = datat[clust[MAXT] - 1] - datat[0] + 0.0;  // 範囲

        double modet;              // これまでに調べた中での最頻値
        int count_maxt = 0;     // これまでに調べた登場回数の中で最大のもの
        for(int i = 1; i < clust[MAXT]; i++){
            std::cout << datat[i] << std::endl;

            double valuet =  datat[i];
            int countt = 1;

            // ソートされているのだから、
            // 同じ値があるとすれば、後続に連続して並んでいるはず。
            // その回数をカウントする。
            for (i = i + 1; i < clust[MAXT]; ++i) {
                if (valuet ==  datat[i]) {
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
        for(int i = 1; i < clust[MAXT]; i++){
            std::cout << datat[i] << std::endl;}
        std::cout <<"タテ線_  最瀕値は"<<modet<< std::endl;
        printf("タテ線＿メジアンは %0.2f\n", histot_med);
        printf("タテ線＿範囲    は %0.2f\n", histot_renge);

        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(modet),380+100*cos(modet)),cv::Scalar(0,0,255), 3, cv::LINE_AA);
        
        //cv::line(img_line5,cv::Point(100,380),cv::Point(100-100*sin(histot_med),380+100*cos(histot_med)),cv::Scalar(0,100,255), 3, cv::LINE_AA);


        //外積処理
        cv::Mat tate_Mat = (cv::Mat_<double>(3, 1) << 
        -100*sin(modet),
        100*cos(modet), 
        1);
    }*/
    //cv::Mat naname_Mat = yoko_Mat.cross(tate_Mat);
	//cout << "yoko_Matとtate_Matの外積 = " << naname_Mat << endl;

    //cv::line(img_line3,cv::Point(100,380),cv::Point(naname_Mat.at<double>(0)/naname_Mat.at<double>(2)+100,
    //naname_Mat.at<double>(1)/naname_Mat.at<double>(2)+380),cv::Scalar(255,0,0), 3, cv::LINE_AA);*
   
    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line5, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_graph, cv::WINDOW_AUTOSIZE);

   
    cv::imshow(win_src, img_src);
    //cv::imshow(win_depth, img_depth);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_line, img_line);
    cv::imshow(win_line2, img_line2);
    cv::imshow(win_line3, img_line3);
    cv::imshow(win_line4, img_line4);
    cv::imshow(win_line5, img_line5);
    cv::imshow(win_graph, img_graph);

 
	cv::waitKey(1);
   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"FLD_clustering_3");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

    //Realsensesの時(roslaunch realsense2_camera rs_camera.launch align_depth:=true)(Depth修正版なのでこっちを使うこと)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));


	ros::spin();//トピック更新待機
			
	return 0;
}