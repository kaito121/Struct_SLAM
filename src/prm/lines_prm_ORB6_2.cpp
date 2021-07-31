//学部時代の卒業研究に改良を加えたバージョン
//具体的にはエッジ検出のをFLDを使ってやる
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
#include <struct_slam/Depth_pclConfig.h>
#include<mutex>
#include<struct_slam/wakuhairetu.h>//自作メッセージ用ヘッダ
#include<algorithm>//並び替え用
#include <opencv2/ximgproc/fast_line_detector.hpp>//FLD
#include <math.h>
#include <highgui.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher  pub;
ros::Publisher  waku_pub;
std::string win_src = "src";//カメラ画像
std::string win_depth = "depth";//深度画像（修正前）
std::string win_depth4 = "depth4";//深度画像（修正前）
std::string win_dst = "dst";//カメラ画像+FLDの線表示
std::string win_prev = "prev";
std::string win_curr = "curr";
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";
std::string win_FeaturePoint = "FeaturePoint";//テンプレートマッチング用テンプレート画像

using namespace std;
using namespace cv;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;

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


cv::Mat img_dst,img_dst6,img_dst8,img_dst9;//depth_pclで使用してる奴
cv::Mat image_curr, image_prev,img_dst2,img_dst3;
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
double Points_curr_average_X,Points_curr_average_Y,Points_prev_average_X,Points_prev_average_Y;//全体オプティカルフローの大きさ(平均)


void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
    cv::Mat img_depth2,img_depth3,img_depth4;//リサイズするdepthはここで定義しないとダメ

	//ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;
    }

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;
    }
    

    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入
    //Depth修正----------------------------------------------------------------------------------------------------
    img_depth2 = depthimage.clone();//depthの画像をコピーする
    //画像クロップ(中距離でほぼ一致)
    cv::Rect roi(cv::Point(110, 95), cv::Size(640/1.6, 480/1.6));//このサイズでDepth画像を切り取るとほぼカメラ画像と一致する
    cv::Mat img_dstdepth = depthimage(roi); // 切り出し画像
    resize(img_dstdepth, img_depth3,cv::Size(), 1.6, 1.6);//クロップした画像を拡大
    img_depth4 = img_depth3.clone();//depth3の画像をコピーする
    cv::imshow(win_depth4, img_depth4);//クロップされたdepth画像
    cv::imshow(win_depth, depthimage);//クロップされたdepth画像

    // pointcloud を作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointCloud->points.reserve(RGBimage.size().width*RGBimage.size().height);//点の数

    	RGBimage.copyTo(img_dst);//ここ間違えていたので注意
  if(kaisu==0){
    img_dst2 = RGBimage.clone();
    img_dst2 = cv::Scalar(0,0,0);
    img_dst3 = RGBimage.clone();
    img_dst3 = cv::Scalar(0,0,0);
  }
 


  int req;


//グレースケール化
cvtColor(RGBimage,image_curr,COLOR_BGR2GRAY);

if (reset == true) {
		std::cout <<"初回検出プログラム"<< std::endl;//最初のフレーム
		// 特徴点検出(グレースケール画像から特徴点検出)
		cv::goodFeaturesToTrack(image_curr, points_curr, 150, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピー
		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      cv::circle(img_dst2, points_curr[i], 3, Scalar(0,255,255), -1, cv::LINE_AA);//追跡記録に描写
      cv::rectangle(img_dst, cv::Point(points_curr[i].x-15,points_curr[i].y+15), 
      cv::Point(points_curr[i].x+15,points_curr[i].y-15), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);//四角形を描写
    
      ////特徴点を検出したら特徴点の周りの画像をクロップして保存→その後保存した画像でBoWを行う予定
      //if(points_curr[i].y<480-30&&points_curr[i].x<640-30){
      //cv::Rect roi2(cv::Point(points_curr[i].x,points_curr[i].y), cv::Size(30, 30));//特徴点を中心とした15☓15pixelの画像を切り取る
      //FeaturePoint[i] = RGBimage(roi2); // 切り出し画像
      //}
		}
		reset = false;//if文切り替え
	} 
	else {
		std::cout <<"二回目オプティカルフロー"<< std::endl;// 特徴点追跡(二回目のフレーム)
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
  //float Zs[points_curr.size()];//距離データ定義
  //std::vector<float> Zs(points_curr.size());
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
  int NotZeroL=-1;
  Points_curr_average_X=0,Points_curr_average_Y=0,Points_prev_average_X=0,Points_prev_average_Y=0;//オプティカルフローの大きさ初期化
  


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
		cv::circle(img_dst, points_curr[i], 4, c, -1, cv::LINE_AA);//今の座標情報
		cv::circle(img_dst, points_prev[i], 3, Scalar(255,255,255), -1, cv::LINE_AA);//一つ前の画像の座標
		cv::line(img_dst,cv::Point(points_curr[i]),cv::Point(points_prev[i]),cv::Scalar(0,255,255), 1, cv::LINE_AA);//線を描写する(オプティカルフロー)
    cv::line(img_dst,cv::Point(points_curr[i].x,points_prev[i].y),cv::Point(points_prev[i]),cv::Scalar(255,0,0), 1, cv::LINE_AA);//線を描写する(オプティカルフローx軸方向)
    cv::line(img_dst,cv::Point(points_prev[i].x,points_curr[i].y),cv::Point(points_prev[i]),cv::Scalar(0,255,0), 1, cv::LINE_AA);//線を描写する(オプティカルフローy軸方向)
    //オプティカルフローの全体の大きさを求める（平均(仮)）
    Points_curr_average_X=Points_curr_average_X+points_curr[i].x;
    Points_curr_average_Y=Points_curr_average_Y+points_curr[i].y;
    Points_prev_average_X=Points_prev_average_X+points_prev[i].x;
    Points_prev_average_Y=Points_prev_average_Y+points_prev[i].y;


    if((points_curr[i].x-points_prev[i].x)>(points_curr[i].y-points_prev[i].y)){
      cv::arrowedLine(img_dst2,points_curr[i],points_prev[i],cv::Scalar(255,255,0),1,8,0,1.0);//矢印の描写
    }
    else{
      cv::arrowedLine(img_dst2,points_curr[i],points_prev[i],cv::Scalar(0,0,255),1,8,0,1.0);//矢印の描写
    }
		
    std::cout <<"特徴点の画像座標(curr)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標
		std::cout <<"特徴点の画像座標(prev)["<<i<<"]="<<points_prev[i]<< std::endl;//特徴点の座標

    pcl::PointXYZRGB jk;

    //XYはimageのデータなのでpclにそのままもって行くとでかい そこである定数で割ることで食らうタリング座標に変換する-------------------------(1)
     jk.x=points_curr[i].x/X_wariai;//ピクセル距離をクラスタリング距離に変換
     jk.y=points_curr[i].y/Y_wariai;
     jk.z=img_depth3.at<float>(points_curr[i])/1000;//ZはDepthデータなのでそのままで行ける

    pointCloud->points.emplace_back(jk);//ポイントクラウドに座標データを移動
    std::cout <<"pointCloud->points.back().z="<<pointCloud->points.back().z<<"_m"<< std::endl;

	}
  //オプティカルフローの全体平均を求める
  Points_curr_average_X=Points_curr_average_X/points_curr.size();
  Points_curr_average_Y=Points_curr_average_Y/points_curr.size();
  Points_prev_average_X=Points_prev_average_X/points_prev.size();
  Points_prev_average_Y=Points_prev_average_Y/points_prev.size();

  cv::line(img_dst,cv::Point(100,380),cv::Point(100+Points_curr_average_X-Points_prev_average_X,380),cv::Scalar(255,0,0), 2, cv::LINE_AA);//線を描写する(オプティカルフローx軸方向)
  cv::line(img_dst,cv::Point(100,380),cv::Point(100,380+Points_curr_average_Y-Points_prev_average_Y),cv::Scalar(0,255,0), 2, cv::LINE_AA);//線を描写する(オプティカルフローy軸方向)



//------------------------------------------------------------------------------

    //pointcloudサイズ設定
    pointCloud -> width = pointCloud -> points.size();
    pointCloud -> height = 1;
    pointCloud -> is_dense = true;

     // クラスタリングの設定
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pointCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
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


    struct_slam::wakuhairetu pclP;

     // クラスタリング の結果を色々できるところ(配列にアクセス)
    for (std::vector<pcl::PointIndices>::iterator it = indices.begin (); it != indices.end (); ++it){ // グループにアクセスするループ(it=クラスタ番号番号)
        std::sort(it->indices.begin (),it->indices.end (),[&] (int const& a,int const& b){//並び替え
        return pointCloud -> points[a].x > pointCloud -> points[b].x;//並び替えの条件
        });
    }
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){ 
        j=j+1; MAXPX=-10000,MINPX=10000,MAXPY=-10000,MINPY=10000;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // グループ中の点にアクセスするループ(pit=グループ内の点番号)
            // 点へのアクセス
      
            pointCloud -> points[*pit].r = R[j%12];//ポイントクラウドのマスの色付け
            pointCloud -> points[*pit].g = G[j%12];
            pointCloud -> points[*pit].b = B[j%12];

		        cv::circle(img_dst, cv::Point((int)(pointCloud -> points[*pit].x*X_wariai), (int)(pointCloud -> points[*pit].y*Y_wariai)), 4, Scalar(B[j%12],G[j%12],R[j%12]), -1, cv::LINE_AA);//今の座標情報

            std::cout <<"image_points["<<pointCloud -> points[*pit].x*X_wariai<<"]["<<pointCloud -> points[*pit].y*Y_wariai<<"]="<<pointCloud -> points[*pit].z<<"_m"<< std::endl;//画像座標
            
            //ここで並び替えを行っている
            if(MAXPY<=(pointCloud -> points[*pit].y * Y_wariai)){MAXPY = pointCloud -> points[*pit].y * Y_wariai;}//yの最大値(ピクセル座標)
            if(MINPY>=(pointCloud -> points[*pit].y * Y_wariai)){MINPY = pointCloud -> points[*pit].y * Y_wariai;}//pointsをwariaiでかけるとピクセル座標になる(ピクセル座標)
            CENTPY=(MAXPY+MINPY)/2;

            
            //image座標からpcl座標に移行（image座標の原点は左上,pcl座標の原点は画像の中心
            //ピクセルX-Xの高さ半分の値
            //pointsはクラスタリング座標なのである定数をかけることでピクセル座標に変換している、ピクセル座標をpcl定数で割ることでpcl座標に変換している
            pointCloud -> points[*pit].x = ((pointCloud -> points[*pit].x*X_wariai)-(RGBimage.size().width/2))/X_pcl; //ピクセル原点からpcl上のX原点変換(pcl座標)
            pointCloud -> points[*pit].y = ((pointCloud -> points[*pit].y*Y_wariai)-(RGBimage.size().height/2))/Y_pcl;//ピクセル原点からpcl上のY原点変換(pcl座標)
            pointCloud -> points[*pit].z *= Z_pcl;
            std::cout <<"pointCloud_point["<<pointCloud -> points[*pit].x<<"]["<<pointCloud -> points[*pit].y<<"]="<<pointCloud -> points[*pit].z<<"_m"<< std::endl;//pointcloud上の座標

            //ここの段階でpointsがPCL座標になる
        }

        //ROS_INFO("おわり");//printと秒数表示
    /*//itの回数が大枠の個数
    //
        double CENPX,CENPXMAX,CENPXMIN,cenpx=1000,cenpxmin=1000,cenpxmax=1000;
        //pcl座標にpclをかけるとピクセル座標に変換される
         MINPX = pointCloud -> points[*(it->indices.end ()-1)].x * X_pcl +(RGBimage.size().width/2);//並び替えたのでitの最後の値がXの最小値となる(leftX)(ピクセル座標)
         MAXPX = pointCloud -> points[*(it->indices.begin ())].x * X_pcl +(RGBimage.size().width/2);//(RightX)(ピクセル座標)
         CENPX = (pointCloud -> points[*(it->indices.end ()-1)].x+pointCloud -> points[*(it->indices.begin ())].x)/2;//(CenterX)（pcl座標）
         CENPXMIN = (pointCloud -> points[*(it->indices.end ()-1)].x+CENPX)/2;//LeftXとCenterXの間のX（pcl座標）
         CENPXMAX = (pointCloud -> points[*(it->indices.begin ())].x+CENPX)/2;//RightXとCenterXの間のX（pcl座標）

        std::cout <<"MINPX["<<MINPX<<"]  MINPY["<<MINPY<<"]"<< std::endl;
        std::cout <<"MAXPX["<<MAXPX<<"]  MAXPY["<<MAXPY<<"]"<< std::endl;
        std::cout <<"CENPX["<<CENPX<<"]"<< std::endl;

     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // 中心点にアクセスするループ(pit=グループ内の点番号
       
       //注目点のX座標を求めそのときのZ座標も求める
        if(cenpx>=std::abs(CENPX-pointCloud -> points[*pit].x)){cenpx = std::abs(CENPX-pointCloud -> points[*pit].x);//大枠の中心点のX座標を求める
            centerx = (pointCloud -> points[*pit].x * X_pcl +(RGBimage.size().width/2));//pcl座標にpcl定数でかけるとピクセル座標になりXsizeの半分の値を足して原点をピクセル原点に戻す(ピクセル座標)
            centerz = pointCloud -> points[*pit].z;
            }//if文終了
        
        if(cenpxmin>=std::abs(CENPXMIN-pointCloud -> points[*pit].x)){cenpxmin = std::abs(CENPXMIN-pointCloud -> points[*pit].x);//大枠の中心点とLeftXの間のX座標を求める
            centerleftx = (pointCloud -> points[*pit].x * X_pcl +(RGBimage.size().width/2));//pcl座標にpcl定数でかけるとピクセル座標になりXsizeの半分の値を足して原点をピクセル原点に戻す(ピクセル座標)
            centerleftz = pointCloud -> points[*pit].z;
            }//if文終了

        if(cenpxmax>=std::abs(CENPXMAX-pointCloud -> points[*pit].x)){cenpxmax = std::abs(CENPXMAX-pointCloud -> points[*pit].x);//大枠の中心点とRightXの間のX座標を求める
            centerrightx = (pointCloud -> points[*pit].x * X_pcl +(RGBimage.size().width/2));//pcl座標にpcl定数でかけるとピクセル座標になりXsizeの半分の値を足して原点をピクセル原点に戻す(ピクセル座標)
            centerrightz = pointCloud -> points[*pit].z;
            }//if文終了*/    
      }//fot文終了

    //rightz = pointCloud -> points[*(it->indices.end ()-1)].z;
    //leftz = pointCloud -> points[*it->indices.begin ()].z;


    //rectangle(img_dst9,Rect(MINPX,MINPY,MAXPX-MINPX+W,MAXPY-MINPY+H),Scalar(24,248,159),1.5);//四角作成
    //putText(img_dst9,"No."+std::to_string(j-1),Point(MINPX,MINPY+30),0,0.5,Scalar(0,255,255),1);
    //putText(img_dst9,"Left_Z="+std::to_string(leftz),Point(MINPX,MINPY+45),0,0.5,Scalar(0,255,255),1);
    //putText(img_dst9,"centerleft_z="+std::to_string(centerleftz),Point(MINPX,MINPY+60),0,0.5,Scalar(0,255,255),1);
    //putText(img_dst9,"center_z="+std::to_string(centerz),Point(MINPX,MINPY+75),0,0.5,Scalar(0,255,255),1);
    //putText(img_dst9,"centerright_z="+std::to_string(centerrightz),Point(MINPX,MINPY+90),0,0.5,Scalar(0,255,255),1);
    //putText(img_dst9,"right_z="+std::to_string(rightz),Point(MINPX,MINPY+105),0,0.5,Scalar(0,255,255),1);
    //line(img_dst9,Point(RGBimage.size().width/2,0),Point(RGBimage.size().width/2,RGBimage.size().height),Scalar(255,0,255),1.7);//画像の中心線X
    //line(img_dst9,Point(0,RGBimage.size().height/2),Point(RGBimage.size().width,RGBimage.size().height/2),Scalar(255,0,255),1.7);//画像の中心線Y
    //line(img_dst9,Point(0,RGBimage.size().height/2-30),Point(RGBimage.size().width,RGBimage.size().height/2-30),Scalar(255,0,255),1.7);//画像の中心線Y-30
    //line(img_dst9,Point(0,RGBimage.size().height/2+90),Point(RGBimage.size().width,RGBimage.size().height/2+90),Scalar(255,0,255),1.7);//画像の中心線Y+90


    //std::cout <<"Left_Z["<<j<<"]="<<leftz<< std::endl;
    //std::cout <<"center_Z["<<j<<"]="<<centerz<< std::endl;
    //std::cout <<"right_Z["<<j<<"]="<<rightz<< std::endl;
    //std::cout <<"Left_X["<<j<<"]="<<MINPX<< std::endl;
    //std::cout <<"CENPX["<<j<<"]="<<CENPX<< std::endl;
    //std::cout <<"center_X["<<j<<"]="<<centerx<< std::endl;
    //std::cout <<"Right_X["<<j<<"]="<<MAXPX<< std::endl;

    //struct_slam::waku waku;
    //waku.point_left.x = (MINPX-(RGBimage.size().width/2))/X_pcl;//送るデータ作成(大枠の右下と左下の座標)(pcl座標)
    //waku.point_left.y = (MINPY-(RGBimage.size().height/2))/Y_pcl;//(pcl座標)
    //waku.point_left.z = leftz;
    //waku.point_right.x = (MAXPX-(RGBimage.size().width/2))/X_pcl;
    //waku.point_right.y = (MAXPY-(RGBimage.size().height/2))/Y_pcl;
    //waku.point_right.z = rightz;
    //waku.point_center.x = (centerx-(RGBimage.size().width/2))/X_pcl;//大枠の中心点のX座標
    //waku.point_center.z = centerz;//大枠の中心点の距離Z
    //waku.point_centerleft.x = (centerleftx-(RGBimage.size().width/2))/X_pcl;
    //waku.point_centerleft.z = centerleftz;
    //waku.point_centerright.x = (centerrightx-(RGBimage.size().width/2))/X_pcl;
    //waku.point_centerright.z = centerrightz;

    //pclP.pclP.emplace_back(waku);//配列に送るデータを追加
    //}
    //pclP.header.stamp = ros::Time::now();//トピックにパブリッシュした（データを送った
    //waku_pub.publish(pclP);

    sensor_msgs::PointCloud2 depth_pcl;
        pcl::toROSMsg (*pointCloud, depth_pcl);
        depth_pcl.header.stamp = ros::Time::now();
        depth_pcl.header.frame_id = rgb_msg->header.frame_id;
        pub.publish(depth_pcl);

	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow("win_src", RGBimage);
    cv::imshow("win_dst", img_dst);
    cv::imshow("win_dst2", img_dst2);
    cv::imshow("win_dst3", img_dst3);
	  cv::imshow("win_curr", image_curr);//今の画像
    //cv::imshow("win_FeaturePoint", FeaturePoint[0]);



		if (req == 1) {//初回は直前の画像がないため考慮
		cv::imshow("win_prev", image_prev);//一つ前の画像像
		std::cout <<"でてるよ"<< std::endl;//特徴点の座標
		}
    int key = cv::waitKey(30);
    if (key == 'r') {reset = true;}// Rキーが押されたら特徴点を再検出
		
		cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
		cv::swap(points_curr, points_prev);// points_curr を points_prev に移す（交換する）
		kaisu=kaisu+1;//行列初期設定用変数
    if(kaisu%50==0){img_dst2 = cv::Scalar(0,0,0),img_dst3 = cv::Scalar(0,0,0);}//100回で描写記録画像をリセット


	cv::waitKey(1);



   //ros::spinにジャンプする
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
	ros::init(argc,argv,"opencv_main");
    	
	ros::NodeHandle nhSub;
    //sub設定(データ受け取り)
   //Realsensesの時
	 //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);
	 //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/depth/image_rect_raw", 1);

   //Realsensesの時(roslaunch realsense2_camera demo_pointcloud.launch)
	 message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	 message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);



    //kinectのとき
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/rgb/image_rect_color", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth_registered/image_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    //reconfigure設定
    dynamic_reconfigure::Server<struct_slam::Depth_pclConfig> drs_;
    drs_.setCallback(boost::bind(&dynamicParamsCB, _1, _2));

    ros::NodeHandle nhPub;
    pub=nhPub.advertise<sensor_msgs::PointCloud2>("depth_pcl", 1000);
    waku_pub=nhPub.advertise<struct_slam::wakuhairetu>("wakuhairetu", 1000);//パブリッシュ設定
    	
	ros::spin();
	
	return 0;
}