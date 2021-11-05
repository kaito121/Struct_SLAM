//線検出(FLD)→Depth取得不可能な線を削除
//まずナナメの線のみを取り出し、取り出した後のナナメの線が含まれていないグループを作る
//その後作成したグループからタテ、ヨコ線の検出を行う
//20210908 青線がばーって表示されるバグを直したが、kskの個数が０個になるとSegmentationFolutになるのが確認できた（たまに起こる）
//そのためkskの対策が必要

//20211008 縦線の中点をテンプレートとしたテンプレートマッチングを導入する

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
std::string win_fld = "fld";//FLDの線を表示
std::string win_fld_ty = "FLD_TY";//抽出したFLDのタテ、ヨコの線を表示
std::string win_fld_t = "FLD_T";//抽出したFLDのタテ線を表示
std::string win_fld_y = "FLD_Y";//抽出したFLDのヨコの線を表示

std::string win_line2 = "line2";//FLD+確率Houghの線を表示
std::string win_line3 = "line3";//分類分けしたタテ、ヨコ、ナナメの線を表示
std::string win_line4 = "line4";//分類分けしたタテ、ヨコの線を表示

std::string win_dstP = "dstP";//特徴点表示+src画像
std::string win_prev = "prev";//一つ前の画像
std::string win_curr = "curr";//今の画像
std::string win_dst2 = "dst2";//オプティカルフローの動き画像
//std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像
std::string win_graph = "graph";//グラフ作成
std::string win_FTL = "FTL";//テンプレート画像

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
cv::Mat img_FLD_TY,img_FLD_T,img_FLD_Y,img_graph;
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
cv::Mat Feature_Tate_Line[100],FTL_template[100];//線のテンプレート画像
cv::Point min_pt1[100], max_pt1[100],min_pt2[100][100], max_pt2[100][100];//テンプレートマッチング用変数
double min_val1[100], max_val1[100],min_val2[100][100], max_val2[100][100];
int tateno_prev=0,TATE_XN_prev[100],TATE_YN_prev[100];
int template_size=15;//テンプレートの大きさ(1/2)
cv::Mat FTL_template_Keep[100][20];//マッチングしたテンプレート画像保存用(線IDにつき20個)
int FTL_template_Keep_number[100];//保存しているテンプレート数を記録(保存テンプレートをここの数で対応化させる)
int Matching_OK[100];//追跡可能かつ識別可能の場合1(識別可能判断要素)

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

    //テンプレート保存用配列初期化
    if(kaisu==0){
        for(int i=0;i<=100;i++){
            FTL_template_Keep_number[i]=0;
            Matching_OK[i]=0;//追跡可能かつ識別可能の場合1となる要素の初期化
        }
    }

    //ここに処理項目
    cv::Mat img_gray,img_gray2,img_edge,img_dst,img_fld,img_line2,img_line3,img_line4;

    //ラインだけの画像を作るために単色で塗りつぶした画像を用意する
    img_fld = img_src.clone();
    img_fld = cv::Scalar(255,255,255);
    img_line2 = img_src.clone();
    img_line2 = cv::Scalar(255,255,255);
    img_line3 = img_src.clone();
    img_line3 = cv::Scalar(255,255,255);
    img_line4 = img_src.clone();
    img_line4 = cv::Scalar(255,255,255);
    img_FLD_TY = img_src.clone();
    img_FLD_TY = cv::Scalar(255,255,255);
    img_FLD_T = img_src.clone();
    img_FLD_T = cv::Scalar(255,255,255);
    img_FLD_Y = img_src.clone();
    img_FLD_Y = cv::Scalar(255,255,255);

    img_graph= img_src.clone();
    img_graph= cv::Scalar(255,255,255);
    cv::line(img_graph,cv::Point(0,480),cv::Point(640,480),cv::Scalar(0,0,0), 3, cv::LINE_AA);//X軸 
    cv::line(img_graph,cv::Point(0,480),cv::Point(0,0),cv::Scalar(0,0,0), 3, cv::LINE_AA);//Y軸 
    //cv::line(img_graph,cv::Point(180*3,0),cv::Point(180*3,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    //cv::line(img_graph,cv::Point(170*3,0),cv::Point(170*3,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
    //cv::line(img_graph,cv::Point(100*3,0),cv::Point(100*3,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(90*6,0),cv::Point(90*6,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(80*6,0),cv::Point(80*6,480),cv::Scalar(0,255,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(45*6,0),cv::Point(45*6,480),cv::Scalar(0,0,0), 1, cv::LINE_AA);//180度(180*3)
    cv::line(img_graph,cv::Point(10*6,0),cv::Point(10*6,480),cv::Scalar(0,0,255), 1, cv::LINE_AA);//180度(180*3)
    
    img_src.copyTo(img_dst);
    img_src.copyTo(img_dst2);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    //FLD変換
    std::vector<cv::Vec4f> lines_fld;
    std::vector<cv::Vec4f> lines_std;
    
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();//特徴線クラスオブジェクトを作成
    fld->detect( img_gray, lines_fld);//特徴線検索
    fld->detect( img_gray, lines_std);//特徴線検索

    //FLDの線描写
    for(int i = 0; i < lines_fld.size(); i++){
       //cv::line(img_dst,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);  
       cv::line(img_fld,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA); 
       cv::line(img_line2,cv::Point(lines_fld[i][0],lines_fld[i][1]),cv::Point(lines_fld[i][2],lines_fld[i][3]),cv::Scalar(0,0,255), 1.5, cv::LINE_AA);
    }
    std::cout <<"lines_fld.size()="<<lines_fld.size()<< std::endl;

    cv::cvtColor(img_fld, img_gray2, cv::COLOR_RGB2GRAY);
    cv::Canny(img_gray2, img_edge, 125, 255);

    //確率的ハフ変換(元画像・lines)
    //std::vector<cv::Vec4i> lines2;
    //cv::HoughLinesP(img_edge, lines2, 1, CV_PI/180, 20,30,10);

    double lines2[lines_fld.size()][4],theta0,theta90;
    float dep1[lines_fld.size()],dep2[lines_fld.size()];

    //Y軸との角度(詳しくは2月の研究ノート)
    theta0=M_PI-atan2((200-0),(100-100));//水平(θ=π/2=1.5708)
    theta90=M_PI-atan2((100-100),(200-0));//垂直(θ=π=3.14159)    
    
    //FLD抽出線とY軸との角度を求める+三次元距離データの結合
    std::cout <<"並び替え前"<<std::endl;
    int ksk=0;
    for(int i =  0; i < lines_fld.size(); i++){
        dep1[i]= img_depth.at<float>(cv::Point(lines_fld[i][0],lines_fld[i][1]));//点の三次元距離データ取得//20210908ここでDepth情報を取得する必要はない気がする
        dep2[i]= img_depth.at<float>(cv::Point(lines_fld[i][2],lines_fld[i][3]));
        if(dep1[i]>0 && dep2[i]>0){//dep1とdep2が0より大きい時に実行する。(距離データの損失を考慮)
            lines2[ksk][0]=lines_fld[i][0];//ここで距離０を除いてる
            lines2[ksk][1]=lines_fld[i][1];
            lines2[ksk][2]=lines_fld[i][2];
            lines2[ksk][3]=lines_fld[i][3];
            dep1[ksk]=dep1[i];
            dep2[ksk]=dep2[i];
            ksk=ksk+1;//距離データを正しく持つ線のみを取得
        }
    }
    std::cout <<"テスト:Depthデータが取得可能な線の個数ksk="<<ksk<<std::endl;

    double lines_NNM[lines_fld.size()][4],lines_NNM2[lines_fld.size()][4],lines_NNM_lc[lines_fld.size()],lines_NNM_thetal[lines_fld.size()];
    double lines3[lines_fld.size()][4],lines3_dep[lines_fld.size()][2],lines3_theta[lines_fld.size()];//抽出されたナナメの線以外
    double NNMA[lines_fld.size()],NNMB[lines_fld.size()],NNM_TATE_X[lines_fld.size()][lines_fld.size()],NNM_TATE_Y[lines_fld.size()][lines_fld.size()];
    int lines_NNM_count=0,lines3_count=0;
	double minlength = image.cols * image.cols * 0.02 ;// (線の最小長さの要件を定義する)
    double NNM_A[lines_fld.size()],NNM_C[lines_fld.size()],NNM_A_MAX,NNM_C_MAX;//斜め線の一次関数
    double NNM_XN[lines_fld.size()],NNM_YN[lines_fld.size()],NNM_CHK[lines_fld.size()];//縦線の結合用
    double NNM_L[lines_fld.size()],tempx,tempy;//縦線の長さ
    double NNM_line[lines_fld.size()][4];//縦線結合後
    int nnmno=0;//縦線結合後個数
    //ナナメの線の抽出を行う-----------------------------------------------------------------------------------
    for(int i=0; i<ksk; i++){
	    //(ほぼ垂直の場合は無視)
	    if ( abs(lines2[i][0]-lines2[i][2]) < 10 || abs(lines2[i][1]-lines2[i][3]) < 10){ //check if almost vertical
            lines3[lines3_count][0]=lines2[i][0];//ナナメの線以外を抽出する
            lines3[lines3_count][1]=lines2[i][1];
            lines3[lines3_count][2]=lines2[i][2];
            lines3[lines3_count][3]=lines2[i][3];
            lines3_dep[lines3_count][0]=dep1[ksk];
            lines3_dep[lines3_count][1]=dep2[ksk];
            cv::line(img_FLD_TY,cv::Point(lines3[lines3_count][0],lines3[lines3_count][1]),cv::Point(lines3[lines3_count][2],lines3[lines3_count][3]),cv::Scalar(0,0,0), 2, cv::LINE_AA);

            //FLD抽出線のy軸との角度を求める
            lines3_theta[lines3_count]=M_PI-atan2((lines3[lines3_count][2]-lines3[lines3_count][0]),(lines3[lines3_count][3]-lines3[lines3_count][1]));
            //std::cout <<"FLDの線の傾きlines3_theta["<<lines3_count<<"]("<<lines3_theta[lines3_count]<<")"<< std::endl;

            lines3_count=lines3_count+1;//ナナメの線以外を線を数える
			continue;
        }
		//(短い線を無視する (x1-x2)^2 + (y2-y1)^2 < minlength)
		if( ((lines2[i][0]-lines2[i][2])*(lines2[i][0]-lines2[i][2]) +(lines2[i][1]-lines2[i][3])*(lines2[i][1]-lines2[i][3])) < minlength){
			continue;
        }   
        lines_NNM[lines_NNM_count][0]=lines2[i][0];//ナナメの線のみを抽出する
        lines_NNM[lines_NNM_count][1]=lines2[i][1];
        lines_NNM[lines_NNM_count][2]=lines2[i][2];
        lines_NNM[lines_NNM_count][3]=lines2[i][3];

        cv::line(img_line2,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
        cv::line(img_line4,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA); 
        cv::line(img_dst,cv::Point(lines_NNM[lines_NNM_count][0],lines_NNM[lines_NNM_count][1]),cv::Point(lines_NNM[lines_NNM_count][2],lines_NNM[lines_NNM_count][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA); 

        //座標から一次関数を引く関数
        lines_NNM_thetal[lines_NNM_count]=(M_PI/2)-(M_PI-atan2((lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0]),(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])));
        lines_NNM_lc[lines_NNM_count]=(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])*(lines_NNM[lines_NNM_count][2]-lines_NNM[lines_NNM_count][0])+(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1])*(lines_NNM[lines_NNM_count][3]-lines_NNM[lines_NNM_count][1]);
        lines_NNM2[lines_NNM_count][0]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//X1座標
        lines_NNM2[lines_NNM_count][1]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*1000;//Y1座標
        lines_NNM2[lines_NNM_count][2]=lines_NNM[lines_NNM_count][0]+(cos(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//X2座標
        lines_NNM2[lines_NNM_count][3]=lines_NNM[lines_NNM_count][1]+(sin(-lines_NNM_thetal[lines_NNM_count])*sqrt(lines_NNM_lc[lines_NNM_count]))*-1000;//Y2座標

        cv::line(img_line2,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_line4,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_FLD_TY,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        cv::line(img_dst,cv::Point(lines_NNM2[lines_NNM_count][0],lines_NNM2[lines_NNM_count][1]),cv::Point(lines_NNM2[lines_NNM_count][2],lines_NNM2[lines_NNM_count][3]),cv::Scalar(0,255,0), 1, cv::LINE_AA);
        //直線の作成(消失点と任意の点を端点とした線を作成する)
        //N(0,lines_std_count)=lines_std[i][0];//直線の端点の座標x
        //N(1,lines_std_count)=lines_std[i][1];//直線の端点の座標y
        //std::cout <<"直線の端点の座標x_N(0,"<<lines_std_count<<")="<< N(0,lines_std_count) << std::endl;
        //std::cout <<"直線の端点の座標y_N(1,"<<lines_std_count<<")="<< N(1,lines_std_count) << std::endl;
        //PC(0,lines_std_count)=lines_std[i][2];//消失点真値(x座標)_観測データ=真値±ノイズ
        //PC(1,lines_std_count)=lines_std[i][3];//消失点真値(y座標)
        //std::cout <<"消失点真値(x座標)_PC(0,"<<lines_std_count<<")="<< PC(0,lines_std_count) << std::endl;
        //std::cout <<"消失点真値(y座標)_PC(1,"<<lines_std_count<<")="<< PC(1,lines_std_count) << std::endl;
        NNM_CHK[lines_NNM_count]=0;//斜め線判別要素初期化（斜め線の結合に使用)
        lines_NNM_count=lines_NNM_count+1;//ナナメの線の数をカウント
	}//------------------------------------------------------------------------------------------------------------------------(ナナメ線抽出)

    //線のグループ化-----------------------------------------------------------------------------------------------------------
    for (int i=0; i<lines_NNM_count; ++i) {
        //lines_NNM[i][1]の位置を上にする
        if(lines_NNM[i][1]>lines_NNM[i][3]){
            tempx=lines_NNM[i][0];
            tempy=lines_NNM[i][1];
            lines_NNM[i][0]=lines_NNM[i][2];
            lines_NNM[i][1]=lines_NNM[i][3];
            lines_NNM[i][2]=tempx;
            lines_NNM[i][3]=tempy;
        }
        NNM_L[i]=sqrt((lines_NNM[i][0]-lines_NNM[i][2])*(lines_NNM[i][0]-lines_NNM[i][2])
         +(lines_NNM[i][1]-lines_NNM[i][3])*(lines_NNM[i][1]-lines_NNM[i][3]));//縦線の点間の長さ
        NNM_XN[i]=(lines_NNM[i][0]+lines_NNM[i][2])/2;//縦線の中点N
        NNM_YN[i]=(lines_NNM[i][1]+lines_NNM[i][3])/2;
        NNM_A[i]=(lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]);
        NNM_C[i]=lines_NNM[i][1]-(((lines_NNM[i][1]-lines_NNM[i][3])/(lines_NNM[i][0]-lines_NNM[i][2]))*lines_NNM[i][0]);
    }

    double clusNNM_R=0.5;//しきい値
     //角度が類似する線をまとめる
    for (int j=0; j<lines_NNM_count; ++j) {
        if(NNM_CHK[j]==0){
            std::cout <<"斜めの線["<<j<<"]"<< std::endl;
            int equal=1;//同じ線の要素数
            double NNM_MAX_LONG=NNM_L[j];
            double NNM_MIN=lines_NNM[j][1];
            double NNM_MAX=lines_NNM[j][3];
            NNM_line[nnmno][0]=lines_NNM[j][0],NNM_line[nnmno][1]=lines_NNM[j][1];//基準はjの線(更新が起きなければj単体)
            NNM_line[nnmno][2]=lines_NNM[j][2],NNM_line[nnmno][3]=lines_NNM[j][3];

            for (int k=j+1; k<lines_NNM_count; ++k) {
                //角度の差がしきい値以下の時同じ線とみなす
                if(abs(lines_NNM_thetal[j]-lines_NNM_thetal[k])<clusNNM_R){
                    std::cout <<"同じ線とみなすlines_NNM["<<k<<"]="<<lines_NNM_thetal[j]-lines_NNM_thetal[k]<< std::endl;
                    NNM_CHK[k]=1;//同じ線とみなす
                    //Y最小が直線の下の端点、Y最大が上の端点になる
                    if(NNM_MIN>lines_NNM[k][1]){
                        NNM_MIN=lines_NNM[k][1];
                        NNM_line[nnmno][1]=lines_NNM[k][1];//Y最小が直線の上の端点
                    }
                    if(NNM_MAX<lines_NNM[k][3]){
                        NNM_MAX=lines_NNM[k][3];
                        NNM_line[nnmno][3]=lines_NNM[k][3];//Y最大が下の端点
                    }
                    //長さが最も長い直線を見つける(長い方がデータに信頼性がある)
                    if(NNM_MAX_LONG<NNM_L[k]){
                        NNM_MAX_LONG=NNM_L[k];//最大長さ更新
                        NNM_A_MAX=NNM_A[k];//最大長さの一次関数を保存
                        NNM_C_MAX=NNM_C[k];//最大長さの一次関数を保存
                    }
                }
            }
            //最も長い線の一次関数からy最大、最小のxを求める
            NNM_line[nnmno][0]=(NNM_line[nnmno][1]-NNM_C_MAX)/NNM_A_MAX;
            NNM_line[nnmno][2]=(NNM_line[nnmno][3]-NNM_C_MAX)/NNM_A_MAX;
            //cv::line(img_dst,cv::Point(NNM_line[nnmno][0],NNM_line[nnmno][1]),cv::Point(NNM_line[nnmno][2],NNM_line[nnmno][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);
            nnmno=nnmno+1;//縦線の合計個数(まとめ後)
        }
    }
    std::cout <<"縦線のクラスタ数(まとめ前)lines_NNM_count="<<lines_NNM_count<< std::endl;
    std::cout <<"縦線の合計個数(まとめ後)tateno="<<nnmno<< std::endl;
    //------------------------------------------------------------------------------------------------------------------------------- 

    //thetaの数値を小さいにソート
    double tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    int yokot,tatet,p,yokoyouso,tateyouso;
    double lines3X,lines3Y,yokolines3[lines_fld.size()][5],tatelines3[lines_fld.size()][5],yokotheta[lines_fld.size()],tatetheta[lines_fld.size()];
    double yokoZ1[lines_fld.size()],yokoZ2[lines_fld.size()],tateZ1[lines_fld.size()],tateZ2[lines_fld.size()];
    double yokothetal[lines_fld.size()],tatethetal[lines_fld.size()],yokolc[lines_fld.size()],tatelc[lines_fld.size()],yokol[lines_fld.size()][4],tatel[lines_fld.size()][4];
    double datat[lines_fld.size()],datay[lines_fld.size()];//ヒストグラムデータ用
    int clusCY,clusCT,clusy[lines_fld.size()],clust[lines_fld.size()],MAXY=0,MAXT=0;//クラスタリング用
    double yokothetaCL[lines_fld.size()],tatethetaCL[lines_fld.size()],clusR;//クラスタリング用
    double yokoclusy[100][200][5],tateclust[100][200][5];
    double Average_yoko_theta[lines_fld.size()];
    double MINI_theta=100,mini_theta;
    int CLUSTER[lines_fld.size()],clus_no=0,YOKO_CLUST;
    int nanamet=0,nanamey=0,NANAME_Line[100];
    double lines_T_theta[lines_fld.size()];
    double tateA[lines_fld.size()],tateB[lines_fld.size()];
    double TATE_A[lines_fld.size()],TATE_C[lines_fld.size()],TATE_A_MAX,TATE_C_MAX;//縦線の一次関数y=TATE_Ax+TATE_C
    double TATE_D[lines_fld.size()][lines_fld.size()],TATE_XN[lines_fld.size()],TATE_YN[lines_fld.size()];//縦線の結合用
    double TATE_L[lines_fld.size()],TATE_K[lines_fld.size()];//縦線の長さ
    double TATE_line[lines_fld.size()][4];//縦線結合後
    int tateno=0;//縦線結合後個数
  
    //int yokocount[200],tatecount[200],yokoko=0,tatete=0;//ヒストグラム用
    //double yokotemp=0,tatetemp=0,yokohist[100],tatehist[100];
    
    yokot=0,tatet=0,p=0,yokoyouso=0,tateyouso=0;

    for (int j=0; j< lines3_count; ++j) {
        lines3X=abs(lines3[j][0]-lines3[j][2]);//傾きを調べる（x成分)
        lines3Y=abs(lines3[j][1]-lines3[j][3]);//傾きを調べる（y成分)
        
        //横線に分類
        if(lines3X>lines3Y){
            //std::cout <<"yoko(lines3X>lines3Y)="<<lines3X<<">"<<lines3Y<< std::endl;
            //std::cout <<"lines3_theta["<<j<<"](ヨコ)="<<lines3_theta[j]<< std::endl;

            yokolines3[yokoyouso][0]=lines3[j][0];//(x成分)
            yokolines3[yokoyouso][1]=lines3[j][1];//(y成分)
            yokolines3[yokoyouso][2]=lines3[j][2];//(x成分)
            yokolines3[yokoyouso][3]=lines3[j][3];//(y成分)
            yokolines3[yokoyouso][4]=lines3_theta[j];

            //yokotheta[yokoyouso]=theta[j];
            yokoZ1[yokoyouso]=lines3_dep[j][0];
            yokoZ2[yokoyouso]=lines3_dep[j][1];
            //cv::line(img_line3,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            //cv::line(img_FLD_Y,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
            cv::line(img_line2,cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]),cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]),cv::Scalar(255,0,0), 2, cv::LINE_AA);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][0],yokolines3[yokoyouso][1]), 4, cv::Scalar(255, 0, 255), 1.5);
          	//cv::circle(img_dst, cv::Point(yokolines3[yokoyouso][2],yokolines3[yokoyouso][3]), 4, cv::Scalar(255, 0, 255), 1.5);


            yokotheta[yokoyouso]=lines3_theta[j]*180/M_PI;//deg表示化
            //θの範囲を0〜180にする
            if(yokotheta[yokoyouso]>=180){
                yokotheta[yokoyouso]=yokotheta[yokoyouso]-180;
                yokolines3[yokoyouso][4]=yokolines3[yokoyouso][4]-M_PI;
            }
            //std::cout <<"yokotheta["<<yokoyouso<<"]="<<yokotheta[yokoyouso]<< std::endl;

            if(yokotheta[yokoyouso]>=90){
                yokotheta[yokoyouso]=180-yokotheta[yokoyouso];
            }
            else{yokotheta[yokoyouso]=yokotheta[yokoyouso];}
            //std::cout <<"yokotheta["<<yokoyouso<<"](クラスタリング用)="<<yokotheta[yokoyouso]<< std::endl;

            clusy[yokoyouso]=0;//クラスタリング用
            yokoyouso=yokoyouso+1;//横線に分類されたグループ数(yokoyouso)
        }
        //縦線に分類
        else{
            if(lines3Y>lines3X){
                //std::cout <<"tate(lines3Y>lines3X)="<<lines3Y<<">"<<lines3X<< std::endl;
                //std::cout <<"lines3_theta["<<j<<"](タテ)="<<lines3_theta[j]<< std::endl;

                tatelines3[tateyouso][0]=lines3[j][0];
                tatelines3[tateyouso][1]=lines3[j][1];
                tatelines3[tateyouso][2]=lines3[j][2];
                tatelines3[tateyouso][3]=lines3[j][3];
                tatelines3[tateyouso][4]=lines3_theta[j];

                //tatetheta[tateyouso]=lines3_theta[j];
                tateZ1[tateyouso]=lines3_dep[j][0];
                tateZ2[tateyouso]=lines3_dep[j][1];
                //cv::line(img_line3,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                //cv::line(img_FLD_T,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);
                cv::line(img_line2,cv::Point(tatelines3[tateyouso][0], tatelines3[tateyouso][1]),cv::Point(tatelines3[tateyouso][2],tatelines3[tateyouso][3]),cv::Scalar(0,0,255), 2, cv::LINE_AA);

                //確率ハフ変換を使用しない時
                tatetheta[tateyouso]=lines3_theta[j]*180/M_PI;//deg表示化

                //θの範囲を0〜180にする
                if(tatetheta[tateyouso]>=180){
                    tatetheta[tateyouso]=tatetheta[tateyouso]-180;
                    tatelines3[tateyouso][4]=tatelines3[tateyouso][4]-M_PI;
                    }
                //std::cout <<"tatetheta["<<tateyouso<<"]="<<tatetheta[tateyouso]<< std::endl;

                //クラスタリング時に最大個数を持つ縦線のクラスタが２つ存在してしまうため、90度で反転させてクラスタリング処理を行う。
                //例(θ=0~10,170~180→180-170=10)
                if(tatetheta[tateyouso]>=90){
                    tatetheta[tateyouso]=180-tatetheta[tateyouso];
                }
                else{tatetheta[tateyouso]=tatetheta[tateyouso];}
                //std::cout <<"tatetheta["<<tateyouso<<"](クラスタリング用)="<<tatetheta[tateyouso]<< std::endl;

                clust[tateyouso]=0;//クラスタリング用
                tateyouso=tateyouso+1;//縦線に分類されたグループ数(tateyouso)
            }
        }
    }

    //ここの並び替え上の並び替えとまとめられそう（先に範囲を狭めてから並び替えして分類する感じにしたらできそう）
    //今は並び替えして分類して範囲狭めて再び並び替えしてる
    //クラスタリング用θで並び替えを行う
    //tatethetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<tateyouso; ++i) {
       for (int j=i+1;j<tateyouso; ++j) {
            if (tatetheta[i] > tatetheta[j]) {
                tmp =  tatetheta[i];
                tmp1x =  tatelines3[i][0];
                tmp1y =  tatelines3[i][1];
                tmp2x =  tatelines3[i][2];
                tmp2y =  tatelines3[i][3];
                tmp3 =  tatelines3[i][4];

                tatetheta[i] = tatetheta[j];
                tatelines3[i][0] = tatelines3[j][0];
                tatelines3[i][1] = tatelines3[j][1];
                tatelines3[i][2] = tatelines3[j][2];
                tatelines3[i][3] = tatelines3[j][3];
                tatelines3[i][4] = tatelines3[j][4];

                tatetheta[j] = tmp;
                tatelines3[j][0] = tmp1x;
                tatelines3[j][1] = tmp1y;
                tatelines3[j][2] = tmp2x;
                tatelines3[j][3] = tmp2y;
                tatelines3[j][4] = tmp3;
            }
        }
    }
    
    //クラスタリング用θで並び替えを行う
    //yokothetaの数値を小さいにソート
    tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
    for (int i=0; i<yokoyouso; ++i) {
       for (int j=i+1;j<yokoyouso; ++j) {
            if (yokotheta[i] > yokotheta[j]) {
                tmp =  yokotheta[i];
                tmp1x =  yokolines3[i][0];
                tmp1y =  yokolines3[i][1];
                tmp2x =  yokolines3[i][2];
                tmp2y =  yokolines3[i][3];
                tmp3 =  yokolines3[i][4];

                yokotheta[i] = yokotheta[j];
                yokolines3[i][0] = yokolines3[j][0];
                yokolines3[i][1] = yokolines3[j][1];
                yokolines3[i][2] = yokolines3[j][2];
                yokolines3[i][3] = yokolines3[j][3];
                yokolines3[i][4] = yokolines3[j][4];

                yokotheta[j] = tmp;
                yokolines3[j][0] = tmp1x;
                yokolines3[j][1] = tmp1y;
                yokolines3[j][2] = tmp2x;
                yokolines3[j][3] = tmp2y;
                yokolines3[j][4] = tmp3;
            }
        }
    }


    //クラスタリング-----------------------------------------------------------------------------------------------------------------------
    //タテ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCT:クラスタ数,Clust[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    clusR=0.15,clusCT=0,Average_tate_theta=0;
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
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                tateclust[0][clust[0]][0]=tatelines3[j][0];//(x成分)
                tateclust[0][clust[0]][1]=tatelines3[j][1];//(y成分)
                tateclust[0][clust[0]][2]=tatelines3[j][2];//(x成分)
                tateclust[0][clust[0]][3]=tatelines3[j][3];//(y成分)
                tateclust[0][clust[0]][4]=tatelines3[j][4];//角度
                clust[MAXT]=1;
            }

            else{
                cv::circle(img_graph, cv::Point(tatetheta[j]*6, 240), 3, Scalar(0,5*clusCT,255), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);

                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新

                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",clusCT="<<clusCT<<"----------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                        MAXT=clusCT;

                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //中間動作
                if(j!=0&&j+1!=tateyouso){
                    //std::cout <<"abs(tatetheta["<<j<<"]*3-tatetheta["<<j+1<<"]*3)="<<abs(tatetheta[j]*3-tatetheta[j+1]*3)<< std::endl;
                    //後方クラスタリング半径範囲内
                    if(clusR>abs(tatetheta[j]*3-tatetheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);

                        clust[clusCT]=clust[clusCT]+1;//クラスタの内部個数更新
                    }
                    //後方クラスタリング半径範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",clusCT="<<clusCT<<"--------------------------------"<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                        tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                        tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                        tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                        tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                        cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                        cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);

                        clusCT=clusCT+1;//クラスタ更新
                    }
                }
                //最終動作
                if(j+1==tateyouso){
                    //std::cout <<"最終動作j="<<j<<",clusCT="<<clusCT<<",clust[clusCT]="<<clust[clusCT]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    tateclust[clusCT][clust[clusCT]][0]=tatelines3[j][0];//(x成分)
                    tateclust[clusCT][clust[clusCT]][1]=tatelines3[j][1];//(y成分)
                    tateclust[clusCT][clust[clusCT]][2]=tatelines3[j][2];//(x成分)
                    tateclust[clusCT][clust[clusCT]][3]=tatelines3[j][3];//(y成分)
                    tateclust[clusCT][clust[clusCT]][4]=tatelines3[j][4];//角度

                    cv::line(img_line4,cv::Point(tateclust[clusCT][clust[clusCT]][0],tateclust[clusCT][clust[clusCT]][1]),
                    cv::Point(tateclust[clusCT][clust[clusCT]][2],tateclust[clusCT][clust[clusCT]][3]),cv::Scalar(0,5*clusCT,255), 3, cv::LINE_AA);
                    //std::cout <<"最終:最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<< std::endl;
                }
                if(clust[clusCT]>=clust[MAXT]){ MAXT=clusCT; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXT=clusCT="<<MAXT<<",最大クラスタの内部個数clust[MAXT]="<<clust[MAXT]<<"\n"<< std::endl;
            }
        }
        //線のグループ化-----------------------------------------------------------------------------------------------------------
        //Xの値が小さい順に並び変える
        tmp=0,tmp1x=0,tmp1y=0,tmp2x=0,tmp2y=0,tmpdep1=0,tmpdep2=0,tmp3=0;
        for (int i=0; i<clust[MAXT]; ++i) {
           for (int j=i+1;j<clust[MAXT]; ++j) {
                if (tateclust[MAXT][i][0]+tateclust[MAXT][i][2] > tateclust[MAXT][j][0]+tateclust[MAXT][j][2]) {
                    tmp1x =  tateclust[MAXT][i][0];
                    tmp1y =  tateclust[MAXT][i][1];
                    tmp2x =  tateclust[MAXT][i][2];
                    tmp2y =  tateclust[MAXT][i][3];
                    tmp3 =  tateclust[MAXT][i][4];

                    tateclust[MAXT][i][0] = tateclust[MAXT][j][0];
                    tateclust[MAXT][i][1] = tateclust[MAXT][j][1];
                    tateclust[MAXT][i][2] = tateclust[MAXT][j][2];
                    tateclust[MAXT][i][3] = tateclust[MAXT][j][3];
                    tateclust[MAXT][i][4] = tateclust[MAXT][j][4];

                    tateclust[MAXT][j][0] = tmp1x;
                    tateclust[MAXT][j][1] = tmp1y;
                    tateclust[MAXT][j][2] = tmp2x;
                    tateclust[MAXT][j][3] = tmp2y;
                    tateclust[MAXT][j][4] = tmp3;
                }
            }
            //tateclust[MAXT][j][1]の位置を上にする
            if(tateclust[MAXT][i][1]>tateclust[MAXT][i][3]){
                tempy=tateclust[MAXT][i][1];
                tempx=tateclust[MAXT][i][0];
                tateclust[MAXT][i][1]=tateclust[MAXT][i][3];
                tateclust[MAXT][i][0]=tateclust[MAXT][i][2];
                tateclust[MAXT][i][3]=tempy;
                tateclust[MAXT][i][2]=tempx;
            }
            TATE_L[i]=sqrt((tateclust[MAXT][i][0]-tateclust[MAXT][i][2])*(tateclust[MAXT][i][0]-tateclust[MAXT][i][2])
             +(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])*(tateclust[MAXT][i][1]-tateclust[MAXT][i][3]));//縦線の点間の長さ
            TATE_XN[i]=(tateclust[MAXT][i][0]+tateclust[MAXT][i][2])/2;//縦線の中点N
            TATE_YN[i]=(tateclust[MAXT][i][1]+tateclust[MAXT][i][3])/2;
        }

        double clusTATE_R=10;//しきい値
        //縦線の中点と直線の距離を使用し、直線をまとめる(距離がしきい値以内なら同じ線とみなす)
       for (int j=0; j<clust[MAXT]; ++j) {
            int equal=0;//同じ線の要素数
            TATE_line[tateno][0]=tateclust[MAXT][j][0],TATE_line[tateno][1]=tateclust[MAXT][j][1];//基準はjの線(更新が起きなければj単体)
            TATE_line[tateno][2]=tateclust[MAXT][j][2],TATE_line[tateno][3]=tateclust[MAXT][j][3];
            double TATE_MAX_LONG=TATE_L[j];
            double TATE_MIN=tateclust[MAXT][j][1];
            double TATE_MAX=tateclust[MAXT][j][3];

            //縦線がy軸と平行でない時のみ実行
            if(tateclust[MAXT][j][0]!=tateclust[MAXT][j][2]){
                std::cout <<"縦線が斜めのとき:tatethetal["<<j<<"]"<< std::endl;

                TATE_A[j]=(tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]);
                TATE_C[j]=tateclust[MAXT][j][1]-(((tateclust[MAXT][j][1]-tateclust[MAXT][j][3])/(tateclust[MAXT][j][0]-tateclust[MAXT][j][2]))*tateclust[MAXT][j][0]);
                for (int k=j+1; k<clust[MAXT]; ++k) {
                    TATE_D[j][k]=abs((TATE_A[j]*TATE_XN[k])-TATE_YN[k]+TATE_C[j])/sqrt((TATE_A[j]*TATE_A[j])+1);//点と直線の距離
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        std::cout <<"(斜め)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_A_MAX=(tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]);
                            TATE_C_MAX=tateclust[MAXT][k][1]-(((tateclust[MAXT][k][1]-tateclust[MAXT][k][3])/(tateclust[MAXT][k][0]-tateclust[MAXT][k][2]))*tateclust[MAXT][k][0]);
                            //最も長い線の一次関数からy最大、最小のxを求める
                            TATE_line[tateno][0]=(TATE_line[tateno][1]-TATE_C_MAX)/TATE_A_MAX;
                            TATE_line[tateno][2]=(TATE_line[tateno][3]-TATE_C_MAX)/TATE_A_MAX;
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い
                }
            }
            //縦線がy軸と平行なとき
            else{
                std::cout <<"縦線がY軸と平行:tatethetal["<<j<<"]"<< std::endl;
                for (int k=j+1; k<clust[MAXT]; ++k) {
                    TATE_D[j][k]=abs(tateclust[MAXT][j][0]-TATE_XN[k]);//点と直線の距離(y軸並行)
                    //距離がしきい値以下の時同じ線とみなす
                    if(TATE_D[j][k]<clusTATE_R){
                        std::cout <<"(平行)同じ線とみなすTATE_D["<<j<<"]["<<k<<"]="<<TATE_D[j][k]<< std::endl;
                        //長さが最も長い直線にまとめる(長い方がデータに信頼性がある)
                        if(TATE_MAX_LONG<TATE_L[k]){
                            TATE_MAX_LONG=TATE_L[k];//最大長さ更新
                            TATE_line[tateno][0]=tateclust[MAXT][k][0];//最大長さのxを保存
                            TATE_line[tateno][2]=tateclust[MAXT][k][2];
                        }
                        //Y最小が直線の下の端点、Y最大が上の端点になる(y軸並行なのでxは長い方の値を使用)
                        if(TATE_MIN>tateclust[MAXT][k][1]){
                            TATE_MIN=tateclust[MAXT][k][1];
                            TATE_line[tateno][1]=tateclust[MAXT][k][1];//Y最小が直線の上の端点
                        }
                        if(TATE_MAX<tateclust[MAXT][k][3]){
                            TATE_MAX=tateclust[MAXT][k][3];
                            TATE_line[tateno][3]=tateclust[MAXT][k][3];//Y最大が下の端点
                        }
                        equal=equal+1;//同じ線の要素数
                    }
                    else{continue;}//連番で見ているのでしきい値を超えた時点で同じ線は無い(まだ連番ではない20211007)
                }
            }
            //短い線はノイズなので除去する
            if(sqrt((TATE_line[tateno][0]-TATE_line[tateno][2])*(TATE_line[tateno][0]-TATE_line[tateno][2])
             +(TATE_line[tateno][1]-TATE_line[tateno][3])*(TATE_line[tateno][1]-TATE_line[tateno][3]))>0){
                tateno=tateno+1;//縦線の合計個数(まとめ後)
            }
            j=j+equal;//同じ線とみなされた個数分進む
        }
        std::cout <<"縦線のクラスタ数(まとめ前)clust[MAXT]="<<clust[MAXT]<< std::endl;
        std::cout <<"縦線の合計個数(まとめ後)tateno="<<tateno<< std::endl;
        //-------------------------------------------------------------------------------------------------------------------------------

        //一次関数を描写するプログラム
        for (int j=0; j<tateno; ++j) {
            //std::cout <<"j="<< j<< std::endl;
            //std::cout <<"clust["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
            //std::cout <<"tateclust["<<MAXT<<"]["<<j<<"][0]="<<TATE_line[j][0]<< std::endl;//確認用

            //座標から一次関数を引く関数
            tatethetal[j]=(M_PI/2)-(M_PI-atan2((TATE_line[j][2]-TATE_line[j][0]),(TATE_line[j][3]-TATE_line[j][1])));
            tatelc[j]=(TATE_line[j][2]-TATE_line[j][0])*(TATE_line[j][2]-TATE_line[j][0])+(TATE_line[j][3]-TATE_line[j][1])*(TATE_line[j][3]-TATE_line[j][1]);
            tatel[j][0]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*100;//X1座標
            tatel[j][1]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*100;//Y1座標
            tatel[j][2]=TATE_line[j][0]+(cos(-tatethetal[j])*sqrt(tatelc[j]))*-100;//X2座標
            tatel[j][3]=TATE_line[j][1]+(sin(-tatethetal[j])*sqrt(tatelc[j]))*-100;//Y2座標
            //std::cout <<"直線の角度1θ="<< tatethetal[j] << std::endl;
            //std::cout <<"直線座標1=("<< tatel[j][0] <<","<<tatel[j][1]<<")"<< std::endl;
            //std::cout <<"直線座標2=("<< tatel[j][2] <<","<<tatel[j][3]<<")\n"<< std::endl;

            cv::line(img_line4,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(tatel[j][0],tatel[j][1]),cv::Point(tatel[j][2],tatel[j][3]),cv::Scalar(0,0,255), 1, cv::LINE_AA);
            cv::line(img_dst,cv::Point(TATE_line[j][0],TATE_line[j][1]),cv::Point(TATE_line[j][2],TATE_line[j][3]),cv::Scalar(0,0,255), 4, cv::LINE_AA);
            datat[j]=TATE_line[clust[j]][4];
            //最大クラスタ内の平均角度を求める
            Average_tate_theta=Average_tate_theta+datat[j];

            //テンプレートマッチングを導入する
            //合成した縦線の中点Nを求める(この中点が線の追跡点であり、テンプレート中心座標)
            TATE_XN[j]=(TATE_line[j][0]+TATE_line[j][2])/2;//中点N
            TATE_YN[j]=(TATE_line[j][1]+TATE_line[j][3])/2;
            cv::circle(img_dst, cv::Point(TATE_XN[j], TATE_YN[j]), 3, Scalar(0,255,255), -1, cv::LINE_AA);//中点N

            //特徴点を検出したら特徴点の周りの画像をクロップして保存→その後保存した画像でBoWを行う予定
            //テンプレートマッチングの不具合を考慮(外枠8Pixelの特徴点を検出しない)
            template_size=15;
            if(template_size<TATE_YN[j]&&TATE_YN[j]<480-template_size){
                if(template_size<TATE_XN[j]&&TATE_XN[j]<640-template_size){
                    //テンプレート作成----------------------------------------------------------------------------------------
                    cv::Rect roi(cv::Point(TATE_XN[j]-template_size,TATE_YN[j]-template_size), cv::Size(template_size*2, template_size*2));//特徴点を中心とした16☓16pixelの画像を切り取る
                    Feature_Tate_Line[j] = img_src(roi); // 切り出し画像
                    cv::rectangle(img_dst, cv::Point(TATE_XN[j]-template_size,TATE_YN[j]+template_size), cv::Point(TATE_XN[j]+template_size,TATE_YN[j]-template_size), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);//四角形を描写
                    cv::namedWindow(win_FTL, cv::WINDOW_AUTOSIZE);
                    cv::imshow(win_FTL, Feature_Tate_Line[j]);
                }
            }
        }
        cv::line(img_dst,cv::Point(TATE_line[0][0],TATE_line[0][1]),cv::Point(TATE_line[0][2],TATE_line[0][3]),cv::Scalar(255,255,255), 4, cv::LINE_AA);//白
        cv::line(img_dst,cv::Point(TATE_line[1][0],TATE_line[1][1]),cv::Point(TATE_line[1][2],TATE_line[1][3]),cv::Scalar(255,0,0), 4, cv::LINE_AA);//青
        cv::line(img_dst,cv::Point(TATE_line[2][0],TATE_line[2][1]),cv::Point(TATE_line[2][2],TATE_line[2][3]),cv::Scalar(0,255,0), 4, cv::LINE_AA);//緑
        cv::line(img_dst,cv::Point(TATE_line[3][0],TATE_line[3][1]),cv::Point(TATE_line[3][2],TATE_line[3][3]),cv::Scalar(0,255,255), 4, cv::LINE_AA);//黄色

        //テンプレートマッチング
        for(int j=0;j<tateno_prev;j++){
            cv::rectangle(img_dst2, cv::Point(TATE_XN_prev[j]-template_size,TATE_YN_prev[j]+template_size), 
             cv::Point(TATE_XN_prev[j]+template_size,TATE_YN_prev[j]-template_size), cv::Scalar(0, 0, 255), 1, cv::LINE_AA);//四角形を描写

            cv::Mat img_minmax1,img_minmax2;
            cv::matchTemplate(img_src, FTL_template[j], img_minmax1, cv::TM_SQDIFF);//差分二乗和
            cv::minMaxLoc(img_minmax1, &min_val1[j], &max_val1[j], &min_pt1[j], &max_pt1[j]);
            std::cout << "min_val1(白)["<<j<<"]=" << min_val1[j] << std::endl;//一致度が上がると値が小さくなる
            std::cout << "max_val1(白)["<<j<<"]=" << max_val1[j] << std::endl;
            if(min_val1[j]<max_val1[j]*0.01){//最小値がしきい値以下なら追跡可能
                cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(255, 255, 255), 3);//白枠
                cv::Rect roi2(cv::Point(min_pt1[j].x,min_pt1[j].y), cv::Size(FTL_template[j].cols, FTL_template[j].rows));//特徴点を中心とした16☓16pixelの画像を切り取る
                cv::Mat TempMatch = img_src(roi2); // 切り出し画像

              //初回線検出時
              if(FTL_template_Keep_number[j]==0){
                FTL_template[j].copyTo(FTL_template_Keep[j][FTL_template_Keep_number[j]]);//追跡可能テンプレートを保存(初回)
                FTL_template_Keep_number[j]=FTL_template_Keep_number[j]+1;//追跡可能テンプレートの個数(保存番号対応化)
                TempMatch.copyTo(FTL_template_Keep[j][FTL_template_Keep_number[j]]);//マッチングした画像もテンプレートとして保存
                FTL_template_Keep_number[j]=FTL_template_Keep_number[j]+1;//追跡可能テンプレートの個数(保存番号対応化)
                std::cout << "FTL_template_Keep_number["<<j<<"]=" << FTL_template_Keep_number[j] << std::endl;
              }
              //2回目線検出時
              /*if(FTL_template_Keep_number[j]!=0){
                for(int k=0;k<FTL_template_Keep_number[j];k++){
                  //今まで保存した全てのテンプレートとマッチング
                  cv::matchTemplate(FTL_template[j], FTL_template_Keep[j][k], img_minmax2, cv::TM_SQDIFF);//差分二乗和
                  cv::minMaxLoc(img_minmax2, &min_val2[j][k], &max_val2[j][k], &min_pt2[j][k], &max_pt2[j][k]);
                    std::cout << "識別判別マッチング:min_val1(白)["<<j<<"]["<<k<<"]=" << min_val2[j][k] << std::endl;//一致度が上がると値が小さくなる
                    std::cout << "識別判別マッチング:max_val1(白)["<<j<<"]["<<k<<"]=" << max_val2[j][k] << std::endl;

                  if(min_val2[j][k]<5e+6){//最小値がしきい値以下なら識別可能
                        cv::Mat img_match;
                        Matching_OK[j]=1;//どれか１つでも識別可能なら1
                        if(j==0){cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(255, 255, 255), 3);}
                        else if(j==1){cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(255, 0, 0), 3);}
                        else if(j==2){cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(0, 255, 0), 3);}
                        else if(j==3){cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(0, 255, 255), 3);}
                        else {cv::rectangle(img_dst2, cv::Rect(min_pt1[j].x, min_pt1[j].y, FTL_template[j].cols, FTL_template[j].rows), cv::Scalar(255, 255, 255), 3);}
                        std::vector<cv::KeyPoint> kpts0;// 特徴点情報を格納するための変数
    	                std::vector<cv::DMatch> matches0;// 特徴点のマッチング情報を格納する変数
		                cv::drawMatches(FTL_template[j], kpts0, FTL_template_Keep[j][k], kpts0, matches0, img_match);
		                cv::imshow("matchs", img_match);
                  }
                }
                //追跡可能かつ識別可能線
                if(Matching_OK[j]==1){
                    FTL_template[j].copyTo(FTL_template_Keep[j][FTL_template_Keep_number[j]]);//追跡可能テンプレートを追加保存
                    FTL_template_Keep_number[j]=FTL_template_Keep_number[j]+1;//追跡可能テンプレートの個数(保存番号対応化)
                }
              }*/
            }

            cv::line(img_dst2,cv::Point(min_pt1[j].x, min_pt1[j].y),cv::Point(TATE_XN_prev[j]-template_size,TATE_YN_prev[j]-template_size),cv::Scalar(0,255,255), 2, cv::LINE_AA);
            Matching_OK[j]=0;
        }
        cv::rectangle(img_dst, cv::Rect(min_pt1[0].x, min_pt1[0].y, FTL_template[0].cols, FTL_template[0].rows), cv::Scalar(255, 255, 255), 3);
        cv::rectangle(img_dst, cv::Rect(min_pt1[1].x, min_pt1[1].y, FTL_template[1].cols, FTL_template[1].rows), cv::Scalar(255, 0, 0), 3);
        cv::rectangle(img_dst, cv::Rect(min_pt1[2].x, min_pt1[2].y, FTL_template[2].cols, FTL_template[2].rows), cv::Scalar(0, 255, 0), 3);
        cv::rectangle(img_dst, cv::Rect(min_pt1[3].x, min_pt1[3].y, FTL_template[3].cols, FTL_template[3].rows), cv::Scalar(0, 255, 255), 3);
        cv::line(img_dst,cv::Point(min_pt1[0].x, min_pt1[0].y),cv::Point(TATE_XN_prev[0]-template_size,TATE_YN_prev[0]-template_size),cv::Scalar(255,255,255), 2, cv::LINE_AA);
        cv::line(img_dst,cv::Point(min_pt1[1].x, min_pt1[1].y),cv::Point(TATE_XN_prev[1]-template_size,TATE_YN_prev[1]-template_size),cv::Scalar(255,0,0), 2, cv::LINE_AA);
        cv::line(img_dst,cv::Point(min_pt1[2].x, min_pt1[2].y),cv::Point(TATE_XN_prev[2]-template_size,TATE_YN_prev[2]-template_size),cv::Scalar(0,255,0), 2, cv::LINE_AA);
        cv::line(img_dst,cv::Point(min_pt1[3].x, min_pt1[3].y),cv::Point(TATE_XN_prev[3]-template_size,TATE_YN_prev[3]-template_size),cv::Scalar(0,255,255), 2, cv::LINE_AA);



        //最大クラスタの要素数が１つだけの時を考慮
        if(clust[MAXT]>1){Average_tate_theta=Average_tate_theta/(clust[MAXT]-1);}//最大クラスタの要素が２つ以上なら通常の平均計算
        else{Average_tate_theta=Average_tate_theta;}//最大クラスタの要素数が２つ未満ならその値を平均値とする

        cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta),380+100*cos(Average_tate_theta)),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta<<"\n"<< std::endl;
        //cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_tate_theta/(clust[MAXT]-1)),380+100*cos(Average_tate_theta/(clust[MAXT]-1))),cv::Scalar(0,100,255), 3, cv::LINE_AA);
        //std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の平均角度="<<Average_tate_theta/(clust[MAXT]-1)<<"\n"<< std::endl;

        //縦線の法線ベクトルを求める-----------------------------------------
        MatrixXd R(2,2);
        R(0,0)=cos(M_PI/2),    R(1,0)=sin(M_PI/2);
        R(0,1)=-sin(M_PI/2),   R(1,1)=cos(M_PI/2);

        MatrixXd n(2,1);
        MatrixXd N(2,1);//Nは2行L列の行列
        N(0,0)=100-100*sin(Average_tate_theta);
        N(1,0)=380+100*cos(Average_tate_theta);
        //N(0,0)=100-100*sin(Average_tate_theta/(clust[MAXT]-1));
        //N(1,0)=380+100*cos(Average_tate_theta/(clust[MAXT]-1));

        MatrixXd P0(2,1);//推定交点
        P0(0,0)=(100+(100-100*sin(Average_tate_theta)))/2;
        P0(1,0)=(380+(380+100*cos(Average_tate_theta)))/2;
        //P0(0,0)=(100+(100-100*sin(Average_tate_theta/(clust[MAXT]-1))))/2;
        //P0(1,0)=(380+(380+100*cos(Average_tate_theta/(clust[MAXT]-1))))/2;

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
//ヨコ線----------------------------------------------------------------------------------------------------------------------------------------------------
    //ヨコ線の平行線のクラスタリング(clusRがクラスタリング半径,clusCY:クラスタ数,Clusy[]:クラスタ内の個数)
    //最も個数の多い並行線クラスタを各線の並行線とする
    clusR=0.15,clusCY=0;
    //要素数が0の時は実行しない
    if(yokoyouso==0){
        std::cout <<"実行しない--------------yokoyouso="<<yokoyouso<< std::endl;
        yoko_histgram = false;//ヨコ線のヒストグラムを実行しない
    }
    else{
        yoko_histgram = true;//ヨコ線のヒストグラムを実行
        for (int j=0; j< yokoyouso; ++j) {
            if(yokoyouso==1){//要素がひとつしかない時は比較ができない(クラスタリングできない)
                std::cout <<"要素がひとつしかない222222222222222222222222222222222222222222222222222222222"<< std::endl;//クラスタ数
                std::cout <<"クラスタ番号clusCY="<<clusCY<< std::endl;//クラスタ数
                std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ

                // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                yokoclusy[0][clusy[0]][0]=yokolines3[j][0];//(x成分)
                yokoclusy[0][clusy[0]][1]=yokolines3[j][1];//(y成分)
                yokoclusy[0][clusy[0]][2]=yokolines3[j][2];//(x成分)
                yokoclusy[0][clusy[0]][3]=yokolines3[j][3];//(y成分)
                yokoclusy[0][clusy[0]][4]=yokolines3[j][4];
                clusy[MAXY]=1;
            }

            else{//クラスタリング
                cv::circle(img_graph, cv::Point(yokotheta[j]*6, 240), 3, Scalar(255,5*clusCY,0), -1, cv::LINE_AA);//線の角度グラフ
                //初回動作時
                if(j==0){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //クラスタリング範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"初回動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];// 角度
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す(初回動作)

                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //クラスタリング範囲外
                    else{
                        //std::cout <<"初回動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<< ",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度
                        
                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);
                        MAXY=clusCY;

                        Average_yoko_theta[clusCY]=yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求める(初回動作クラスタリング範囲外なのでデータが１つしかない)
                        //std::cout <<"初回動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での平均値初期化
                    }
                }
                //中間動作
                if(j!=0&&j+1!=yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"abs(yokotheta["<<j<<"]*3-yokotheta["<<j+1<<"]*3)="<<abs(yokotheta[j]*3-yokotheta[j+1]*3)<< std::endl;
                    //後方範囲内
                    if(clusR>abs(yokotheta[j]*3-yokotheta[j+1]*3)){
                        //std::cout <<"中間動作範囲内j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                        clusy[clusCY]=clusy[clusCY]+1;//クラスタの内部個数更新
                    }
                    //後方範囲外
                    else{
                        //std::cout <<"中間動作範囲外j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                        // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                        yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                        yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                        yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                        cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                        cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す
                        
                        if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                        }
                        else{//クラスタの要素が一つしかない時
                            Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均=要素(要素がひとつしかないから=平均)
                        }
                        //std::cout <<"中間動作範囲外:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<<"--------------------------------"<< std::endl;
                        //要素数が3個以上のクラスタをキープする(開始要素が0を考慮)
                        if(clusy[clusCY]>=3-1){
                            clus_no=clus_no+1;
                            //std::cout <<"要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                            CLUSTER[clus_no]=clusCY;//クラスタ番号を CLUSTER[clus_no]に保存
                            
                        }

                        clusCY=clusCY+1;//クラスタ更新
                        Average_yoko_theta[clusCY]=0;//中間動作での次の平均値初期化
                    }
                }
                //最終動作(j<yokoyouso個)
                if(j+1==yokoyouso){
                    //std::cout <<"yokotheta["<<j<<"]*3="<<yokotheta[j]*3<<",yokotheta[j+1]*3="<<yokotheta[j+1]*3<< std::endl;
                    //std::cout <<"最終動作j="<<j<<",クラスタ番号clusCY="<<clusCY<<",クラスタ内部個数clusy["<<clusCY<<"]="<<clusy[clusCY]<< std::endl;
                    // クラスタリングされたヨコ線(クラスタ番号,クラスタ内部個数,成分番号)
                    yokoclusy[clusCY][clusy[clusCY]][0]=yokolines3[j][0];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][1]=yokolines3[j][1];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][2]=yokolines3[j][2];//(x成分)
                    yokoclusy[clusCY][clusy[clusCY]][3]=yokolines3[j][3];//(y成分)
                    yokoclusy[clusCY][clusy[clusCY]][4]=yokolines3[j][4];//角度

                    cv::line(img_line4,cv::Point(yokoclusy[clusCY][clusy[clusCY]][0],yokoclusy[clusCY][clusy[clusCY]][1]),
                    cv::Point(yokoclusy[clusCY][clusy[clusCY]][2],yokoclusy[clusCY][clusy[clusCY]][3]),cv::Scalar(255,5*clusCY,0), 3, cv::LINE_AA);

                    Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]+yokoclusy[clusCY][clusy[clusCY]][4];//角度の平均を求めるために角度を足す

                    if(clusy[clusCY]>0){//クラスタの要素が複数ある時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY]/clusy[clusCY];//角度の平均を求める
                    }
                    else{//クラスタの要素が一つしかない時
                        Average_yoko_theta[clusCY]=Average_yoko_theta[clusCY];//角度の平均を求める
                    }
                    //std::cout <<"最終動作:クラスタ平均角度Average_yoko_theta["<<clusCY<<"]="<<Average_yoko_theta[clusCY]<< std::endl;
                    //要素数が3個以上のクラスタをキープする
                    if(clusy[clusCY]>=3-1){
                        clus_no=clus_no+1;
                        //std::cout <<"最終動作:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
                        CLUSTER[clus_no]=clusCY;
                    }
                }

                //if(clusy[clusCY]>=clusy[MAXY]){ MAXY=clusCY; }//最大クラスタをキープする
                //std::cout <<"最大クラスタMAXY=clusCY="<<MAXY<<",最大クラスタの内部個数clusy[MAXY]="<<clusy[MAXY]<< std::endl;
            }
        }
        //クラスタリングの要素数が３つ以上のクラスタがあるときだけ実行
        if(clus_no>0){
            std::cout <<"テスト:要素数が3個以上のクラスタ  clus_no="<<clus_no<< std::endl;
            for(int h=1;h<=clus_no;h++){
                std::cout <<"要素数が3個以上のクラスタ  クラスタ番号clusCY=CLUSTER[clus_no="<<h<<"]="<<CLUSTER[h]<<std::endl;
                std::cout <<"Average_yoko_theta["<<CLUSTER[h]<<"]="<<Average_yoko_theta[CLUSTER[h]]<< std::endl;
                //縦線の法線ベクトルとヨコ線の平均角度の差を見てる
                if(MINI_theta>abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]])){
                    MINI_theta=abs(Average_tate_theta_Y-Average_yoko_theta[CLUSTER[h]]);
                    YOKO_CLUST=CLUSTER[h];
                }
            }
            std::cout <<"最大クラスタMAXT=clusCT["<<MAXT<<"]内の法線ベクトルの角度="<<Average_tate_theta_Y<< std::endl;
            std::cout <<"タテの法線ベクトルと最も並列なヨコ線クラスタ clusCY=YOKO_CLUST="<<YOKO_CLUST<<",Average_yoko_theta_Y="<<Average_yoko_theta[YOKO_CLUST]<< std::endl;
            cv::line(img_graph,cv::Point(100,380),cv::Point(100-100*sin(Average_yoko_theta[YOKO_CLUST]),380+100*cos(Average_yoko_theta[YOKO_CLUST])),cv::Scalar(255,0,0), 3, cv::LINE_AA);

            std::cout <<"clusy[YOKO_CLUST="<<YOKO_CLUST<<"]="<<clusy[YOKO_CLUST]<< std::endl;

            //一次関数を描写するプログラム
            for(int j = 0; j <clusy[YOKO_CLUST]; j++){
                std::cout <<"j="<< j<< std::endl;
                std::cout <<"clusy["<<j<<"]="<<j<< std::endl;//クラスタの内部個数
                std::cout <<"yokoclusy["<<YOKO_CLUST<<"]["<<j<<"][0]="<<yokoclusy[YOKO_CLUST][j][0]<< std::endl;//確認用

                //座標から一次関数を引く関数(線を延長してる)
                yokothetal[j]=(M_PI/2)-(M_PI-atan2((yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0]),(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])));
                yokolc[j]=(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])*(yokoclusy[YOKO_CLUST][j][2]-yokoclusy[YOKO_CLUST][j][0])+(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1])*(yokoclusy[YOKO_CLUST][j][3]-yokoclusy[YOKO_CLUST][j][1]);
                yokol[j][0]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*100;//X1座標
                yokol[j][1]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*100;//Y1座標
                yokol[j][2]=yokoclusy[YOKO_CLUST][j][0]+(cos(-yokothetal[j])*sqrt(yokolc[j]))*-100;//X2座標
                yokol[j][3]=yokoclusy[YOKO_CLUST][j][1]+(sin(-yokothetal[j])*sqrt(yokolc[j]))*-100;//Y2座標
                //std::cout <<"直線の角度1θ="<< yokothetal[j] << std::endl;
                //std::cout <<"直線座標1=("<< yokol[j][0] <<","<<yokol[j][1]<<")"<< std::endl;
                //std::cout <<"直線座標2=("<< yokol[j][2] <<","<<yokol[j][3]<<")\n"<< std::endl;

                cv::line(img_line4,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);
                cv::line(img_dst,cv::Point(yokol[j][0],yokol[j][1]),cv::Point(yokol[j][2],yokol[j][3]),cv::Scalar(255,0,0), 1, cv::LINE_AA);

                cv::line(img_dst,cv::Point(yokoclusy[YOKO_CLUST][j][0],yokoclusy[YOKO_CLUST][j][1]),
                 cv::Point(yokoclusy[YOKO_CLUST][j][2],yokoclusy[YOKO_CLUST][j][3]),cv::Scalar(255,0,0), 4, cv::LINE_AA);

                datay[j]=yokoclusy[YOKO_CLUST][j][4];
            }
        }
    }
    //タテ線とナナメ線の交点を検出する--------------------------------------------------------------------------------------------------
    //縦線の一次関数を求める
    for (int j=0; j<lines_NNM_count; ++j) {
        NNMA[j]=(lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]);
        NNMB[j]=lines_NNM[j][1]-(((lines_NNM[j][1]-lines_NNM[j][3])/(lines_NNM[j][0]-lines_NNM[j][2]))*lines_NNM[j][0]);
        for (int i=0; i<clust[MAXT]; ++i) {
            if(tateclust[MAXT][i][0]!=tateclust[MAXT][i][2]){
                tateA[i]=(tateclust[MAXT][i][1]-tateclust[MAXT][i][3])/(tateclust[MAXT][i][0]-tateclust[MAXT][i][2]);
                tateB[i]=tateclust[MAXT][i][1]-tateA[i]*tateclust[MAXT][i][0];
                NNM_TATE_X[j][i]=(tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]);
                NNM_TATE_Y[j][i]=NNMA[j]*((tateB[i]-NNMB[j])/(NNMA[j]-tateA[i]))+NNMB[j];
            }
            else{
                NNM_TATE_X[j][i]=tateclust[MAXT][i][0];
                NNM_TATE_Y[j][i]=(NNMA[j]*NNM_TATE_X[j][i])+NNMB[j];
            }
            //std::cout <<"縦線とヨコ線の交点["<<j<<"]["<<i<<"]=("<<NNM_TATE_X[j][i]<<","<<NNM_TATE_Y[j][i]<<")"<< std::endl;
            //cv::circle(img_dst,cv::Point(NNM_TATE_X[j][i],NNM_TATE_Y[j][i]),3,Scalar(0,255,255),-1);
        }
    }


   
    std::cout <<"for文終了"<< std::endl;

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_depth, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_ty, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_t, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_fld_y, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line2, cv::WINDOW_AUTOSIZE);
    //cv::namedWindow(win_line3, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_line4, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_graph, cv::WINDOW_AUTOSIZE);

   
    cv::imshow(win_src, img_src);
    //cv::imshow(win_depth, img_depth);
    cv::imshow(win_dst, img_dst);
    cv::imshow(win_dst2, img_dst2);
    cv::imshow(win_fld, img_fld);
    //cv::imshow(win_fld_ty, img_FLD_TY);
    //cv::imshow(win_fld_t, img_FLD_T);
    //cv::imshow(win_fld_y, img_FLD_Y);
    cv::imshow(win_line2, img_line2);
    //cv::imshow(win_line3, img_line3);
    cv::imshow(win_line4, img_line4);
    cv::imshow(win_graph, img_graph);

    for (int j=0; j<tateno; ++j) {
        cv::swap(Feature_Tate_Line[j], FTL_template[j]);// Feature_Tate_Line を FTL_template に移す（交換する）
        TATE_XN_prev[j]=TATE_XN[j];
        TATE_YN_prev[j]=TATE_YN[j];
    }
    tateno_prev=tateno;
    kaisu=kaisu+1;
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