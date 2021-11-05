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
std::string win_FeaturePoint = "FeaturePoint";//特徴点を検出したら特徴点の周りの画像をクロップした画像
std::string win_depth4 = "depth";//クロップされたdepth画像
std::string win_img_1 = "img_1";//カメラ画像

using namespace std;
using namespace cv;

// reset == TRUE のとき特徴点検出を行う
// 最初のフレームで必ず特徴点検出を行うように、初期値を TRUE にする
bool reset = true;
bool temp2=false;
int templP=0;
int matchP=0;
int templPkosuu=0;
// image_curr:  現在の入力画像、    image_prev:  直前の入力画像
// points_curr: 現在の特徴点リスト、points_prev: 直前の特徴点リスト
cv::Mat frame,image_curr, image_prev,img_dst,img_dst2,img_1;
cv::Mat img_template1;
vector<cv::Point2f> points_prev, points_curr,points_prev_not, points_curr_not;
vector<cv::Point2f> mappoint;
vector<cv::Point2f> points_prev_limited, points_curr_limited;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
vector<cv::Point2f> points_temp;//テンプレートマッチングの不具合を考慮(外枠15Pixelの特徴点を検出しない)
vector<cv::Point2f> Template_Points_Prev,Template_Points_Curr;//テンプレートの中心座標

cv::Mat_<double>d0;//カメラ座標系と正規化画像座標系の変換に使用する行列
cv::Mat FeaturePoint[100];//特徴点周囲の切り取った画像

int kaisu,optkaisu;//行列初期設定用変数


cv::Point min_pt1[10], max_pt1[10] ,min_pt2, max_pt2, min_pt3, max_pt3;//テンプレートマッチング用変数
double min_val1[10], max_val1[10], min_val2[10], max_val2[10], min_val3[10], max_val3[10];


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


  //画像処理------------------------------------------------------------------------------------------------------------------------
  image.copyTo(frame);//ここ間違えていたので注意
	image.copyTo(img_dst);//ここ間違えていたので注意

	cv::cvtColor(image, image_curr, cv::COLOR_BGR2GRAY);//グレースケール化

	//初回検出プログラム-----------------------------------------------------------------------------------------------------
    optkaisu=0;//オプティカルフロー回数リセット
		std::cout <<"初回検出プログラム["<<optkaisu<<"]"<< std::endl;//最初のフレーム
    image.copyTo(img_1);
		// 特徴点検出(グレースケール画像から特徴点検出)
    //cv::goodFeaturesToTrack(今画像, 前画像, 特徴点の個数, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::goodFeaturesToTrack(image_curr, points_curr, 4, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
		cv::cornerSubPix(image_curr, points_curr, cv::Size(10, 10), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03));
		points_prev = points_curr;//データのコピ
    points_curr_limited = points_curr;//points_curr_limitedの定義
    points_prev_limited = points_curr;//points_prev_limitedの定義
    Template_Points_Curr = points_curr;//Template_Points_Currの定義
    Template_Points_Prev = points_curr;//Template_Points_Prevの定義
    int limited=0;//テンプレートマッチングの不具合を考慮(外枠8Pixelの特徴点を検出しない)
    int template_size=15;

		for (int i = 0; i < points_curr.size(); i++) {
			cv::circle(img_dst, points_curr[i], 6, Scalar(255,0,0), -1, cv::LINE_AA);
      //cv::rectangle(img_dst, cv::Point(points_curr[i].x-template_size,points_curr[i].y+template_size), 
      //cv::Point(points_curr[i].x+template_size,points_curr[i].y-template_size), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);//四角形を描写(青)
    
    //特徴点を検出したら特徴点の周りの画像をクロップして保存→その後保存した画像でBoWを行う予定
    //テンプレートマッチングの不具合を考慮(外枠8Pixelの特徴点を検出しない)
    if(template_size<points_curr[i].y&&points_curr[i].y<480-template_size){
      if(template_size<points_curr[i].x&&points_curr[i].x<640-template_size){

      points_curr_limited[limited] = points_curr[i];//新しいpoint_vecterに書き換える(外枠8Pixelの特徴点を検出しない

      //テンプレート作成----------------------------------------------------------------------------------------
      std::cout <<"特徴点画像クロッププログラム["<<i<<"]"<< std::endl;//最初のフレーム
      cv::Rect roi2(cv::Point(points_curr[i].x-template_size,points_curr[i].y-template_size), cv::Size(template_size*2, template_size*2));//特徴点を中心とした16☓16pixelの画像を切り取る
      FeaturePoint[i] = image(roi2); // 切り出し画像
      

      std::cout <<"特徴点画像クロッププログラムlimited["<<limited<<"]"<< std::endl;
      cv::rectangle(img_1, cv::Point(points_curr_limited[i].x-template_size,points_curr_limited[i].y+template_size), 
      cv::Point(points_curr_limited[i].x+template_size,points_curr_limited[i].y-template_size), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)
      std::cout <<"特徴点の画像座標(curr_if)["<<i<<"]="<<points_curr[i]<< std::endl;//特徴点の座標(範囲制限前)
      std::cout <<"特徴点の画像座標(limited)["<<limited<<"]="<<points_curr_limited[limited]<< std::endl;//特徴点の座標(範囲制限後)
      limited=limited+1;//範囲制限後の特徴点の個数
      }}
		}
    cv::rectangle(img_1, cv::Point(points_curr_limited[0].x-template_size,points_curr_limited[0].y+template_size), 
    cv::Point(points_curr_limited[0].x+template_size,points_curr_limited[0].y-template_size), cv::Scalar(255, 0, 0), 2, cv::LINE_AA);//四角形を描写(白)

    for(int i=0;i<Template_Points_Prev.size();i++){
      cv::rectangle(img_1, cv::Point(points_temp[i].x-template_size,points_temp[i].y+template_size), 
      cv::Point(points_temp[i].x+template_size,points_temp[i].y-template_size), cv::Scalar(0, 255, 255), 2, cv::LINE_AA);//四角形を描写(白)
    }

    points_prev_limited = points_curr_limited;
    points_curr_limited.resize(limited);
    points_prev_limited.resize(limited);
    std::cout <<"制限した特徴点の個数(limited.size)="<<points_curr_limited.size()<< std::endl;
    Template_Points_Prev = points_curr_limited;//テンプレートの中心座標
    Template_Points_Prev.resize(limited);
		reset = false;//if文切り替え

     //テンプレートマッチング----------------------------------------------------------------------------------------
    cv::Point min_ptA[100];//テンプレートマッチング用変数

    //std::cout << "二回目動作プログラム"<< std::endl;
    if(temp2==false){
    templP=0;
    for (int i = 0; i < Template_Points_Prev.size(); i++) {
     //if(req == 1){
        std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
        FeaturePoint[i].copyTo(img_template1); // 切り出し画像
        //cv::imshow("win_FeaturePoint", FeaturePoint[1]);//黄色の特徴点を中心としたクロップ画像

        cv::Mat img_minmax1;
        // テンプレートマッチング
        cv::matchTemplate(image, img_template1, img_minmax1, cv::TM_SQDIFF);//差分二乗和
        cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
        std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
        std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
        if(min_val1[i]<max_val1[i]*0.05){//最小値がしきい値以下なら表示
          min_ptA[templP]=min_pt1[i];
          cv::rectangle(img_dst, cv::Rect(min_pt1[i].x, min_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
          cv::circle(img_dst, cv::Point(min_pt1[i].x+img_template1.cols/2, min_pt1[i].y+img_template1.rows/2), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
          std::cout << "マッチング座標min_pt1["<<i<<"]=" << min_pt1[i] << std::endl;
          //std::cout << "マッチング座標min_pt1["<<i<<"].y=" << min_pt1[i].y << std::endl;
          Template_Points_Prev[i].x=min_pt1[i].x+img_template1.cols/2;
          Template_Points_Prev[i].y=min_pt1[i].y+img_template1.rows/2;
          FeaturePoint[matchP]=FeaturePoint[i];//マッチングしたテンプレートを保存
          points_temp[matchP]=points_curr_limited[i];//テンプレートの取得座標を保存

          //MatchTemp[matchP]=img_template1;//マッチングしたテンプレートを保存
          templP=templP+1;//マッチングの中心座標個数
          matchP=matchP+1;
          //if(Template_Points_Prev.size()+templP>10){
          //  templP=templP-1;
          //}
          std::cout <<"マッチングの中心座標["<<i<<"]="<<Template_Points_Prev[i]<< std::endl;
        }
      }
      temp2=true;
    }

    else{
     std::cout << "三回目動作プログラム"<< std::endl;
     templP=0;
      for (int i = 0; i < Template_Points_Prev.size(); i++) {
     //テンプレートマッチング----------------------------------------------------------------------------------------
     //if(req == 1){
        std::cout << "テンプレートマッチングプログラム["<<i<<"]"<< std::endl;
        FeaturePoint[i].copyTo(img_template1); // 切り出し画像
        //cv::imshow("win_FeaturePoint", FeaturePoint[1]);//黄色の特徴点を中心としたクロップ画像

        cv::Mat img_minmax1;
        // テンプレートマッチング
        cv::matchTemplate(image, img_template1, img_minmax1, cv::TM_SQDIFF);//差分二乗和
        cv::minMaxLoc(img_minmax1, &min_val1[i], &max_val1[i], &min_pt1[i], &max_pt1[i]);
        std::cout << "min_val1(白)["<<i<<"]=" << min_val1[i] << std::endl;//一致度が上がると値が小さくなる
        std::cout << "max_val1(白)["<<i<<"]=" << max_val1[i] << std::endl;
        if(min_val1[i]<max_val1[i]*0.05){//最小値がしきい値以下なら表示
          min_ptA[templP]=min_pt1[i];
          cv::rectangle(img_dst, cv::Rect(min_pt1[i].x, min_pt1[i].y, img_template1.cols, img_template1.rows), cv::Scalar(0, 255, 0), 3);//白枠
          cv::circle(img_dst, cv::Point(min_pt1[i].x+img_template1.cols/2, min_pt1[i].y+img_template1.rows/2), 5, cv::Scalar(0, 255, 0), -1);//テンプレートの中心座標
          std::cout << "マッチング座標min_pt1["<<i<<"]=" << min_pt1[i] << std::endl;
          //std::cout << "マッチング座標min_pt1["<<i<<"].y=" << min_pt1[i].y << std::endl;
          Template_Points_Prev[i].x=min_pt1[i].x+img_template1.cols/2;
          Template_Points_Prev[i].y=min_pt1[i].y+img_template1.rows/2;
          //新規に取得しマッチングしたテンプレートのみ保存
          if(i<4&&matchP<=11){
            FeaturePoint[matchP]=img_template1;//マッチングしたテンプレートを保存
            //MatchTemp[matchP]=img_template1;//マッチングしたテンプレートを保存
            points_temp[matchP]=points_curr_limited[i];//テンプレートの取得座標を保存

            templP=templP+1;//マッチングの中心座標個数
            matchP=matchP+1;
          }

          //if(Template_Points_Prev.size()+templP>10){
          //  templP=templP-1;
          //}
          std::cout <<"マッチングの中心座標["<<i<<"]="<<Template_Points_Prev[i]<< std::endl;
        }
      }
      templPkosuu=matchP;
    }
      std::cout <<"matchP="<<matchP<< std::endl;

      Template_Points_Prev.resize(Template_Points_Prev.size()+templPkosuu);
      std::cout <<"マッチングの中心座標の個数="<<Template_Points_Prev.size()<< std::endl;
      //基本的にこれで同じものを追跡することができている(ひとつだけ青で固定できている)
      cv::rectangle(img_dst, cv::Rect(min_ptA[0].x, min_ptA[0].y, img_template1.cols, img_template1.rows), cv::Scalar(255, 0, 0), 3);//白枠
      cv::circle(img_dst, cv::Point(min_ptA[0].x+img_template1.cols/2, min_ptA[0].y+img_template1.rows/2), 5, cv::Scalar(255, 0, 0), -1);//テンプレートの中心座標

    //マッチングの個数が1以下になったらオプティカルフロースキップ(オプティカルフローの追跡エラー対策)
		if(Template_Points_Prev.size()<=1){reset = true;}


     

  cv::imshow("win_curr", image_curr);//今の画像
  cv::imshow("win_img_1", img_1);//今の画像
  cv::imshow("win_dst", img_dst);

  //---------------------------------------------------------------------------------------------
  
  cv::swap(image_curr, image_prev);// image_curr を image_prev に移す（交換する）
  cv::swap(Template_Points_Curr, Template_Points_Prev);// points_curr を points_prev に移す（交換する）
  //cv::swap(points_curr_limited, points_prev_limited);// points_curr を points_prev に移す（交換する）
  kaisu=kaisu+1;//行列初期設定用変数
  
  cv::waitKey(1);//ros::spinにジャンプする
}

//メイン関数-----------------------------------------------------------------------------------
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nhSub;//ノードハンドル
  points_temp.resize(1000);

	
	//subscriber関連
	//message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	//message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);//センサーメッセージを使うときは対応したヘッダーが必要
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/aligned_depth_to_color/image_raw", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

	ros::spin();//トピック更新待機
			
	return 0;
}