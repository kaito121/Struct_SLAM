#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
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

ros::Publisher  laser_pub;

void callback(const struct_slam::wakuhairetu::ConstPtr& waku_msg,const sensor_msgs::LaserScan::ConstPtr& laser_msg){
ROS_INFO("callBack1");
sensor_msgs::LaserScan laserScan_src = *laser_msg; //データ取り出し、変数定義
//waku_msg -> pclP[0].point_left.x;//pclPという名の配列の0番目のpoint_left.xの値
//waku_msg -> pclP[0].point_center.x;
//waku_msg -> pclP.size();//pclPの配列の個数(大枠の個数)
ROS_INFO("callBack");
std::cout <<"waku_size="<<waku_msg -> pclP.size()<<"_m"<< std::endl;
int i,p,renge,waku_size=waku_msg -> pclP.size();

// double left_l[waku_size],right_l[waku_size],left_s[waku_size],right_s[waku_size];
// double kosu[waku_size],henka_l[waku_size];

std::vector<double> left_l(waku_size);//リサイズ可能な配列を定義
std::vector<double> right_l(waku_size);
std::vector<double> center_l(waku_size);
std::vector<double> centerleft_l(waku_size);
std::vector<double> centerright_l(waku_size);
std::vector<double> left_s(waku_size);
std::vector<double> right_s(waku_size);
std::vector<double> center_s(waku_size);
std::vector<double> centerleft_s(waku_size);
std::vector<double> centerright_s(waku_size);
std::vector<int> kosuLCL(waku_size);
std::vector<int> kosuCLC(waku_size);
std::vector<int> kosuCCR(waku_size);
std::vector<int> kosuCRR(waku_size);
std::vector<double> henka_LCL(waku_size);
std::vector<double> henka_CLC(waku_size);
std::vector<double> henka_CCR(waku_size);
std::vector<double> henka_CRR(waku_size);
std::vector<std::vector<float>> LCL_l(waku_size);//leftL
std::vector<std::vector<float>> CLC_l(waku_size);//LeftCenterL
std::vector<std::vector<float>> CCR_l(waku_size);//RightL
std::vector<std::vector<float>> CRR_l(waku_size);//RightCenterL
std::vector<std::vector<double>> LCL_s(waku_size);//
std::vector<std::vector<double>> CLC_s(waku_size);
std::vector<std::vector<double>> CCR_s(waku_size);
std::vector<std::vector<double>> CRR_s(waku_size);

for(i=0;i<laserScan_src.ranges.size();i++){
  if(laserScan_src.ranges[i]==0){
   laserScan_src.ranges[i]=std::numeric_limits<float>::quiet_NaN();
   std::cout <<"i="<<i<< std::endl;
   }
}


for(p=0;p<waku_msg -> pclP.size();p++){//枠の個数
//waku_msg -> pclP[p].point_left.x(pcl座標)
  left_l[p]=std::sqrt(waku_msg -> pclP[p].point_left.x * waku_msg -> pclP[p].point_left.x + waku_msg -> pclP[p].point_left.z * waku_msg -> pclP[p].point_left.z);//カメラからのLeftXの距離Lを求める
  right_l[p]=std::sqrt(waku_msg -> pclP[p].point_right.x * waku_msg -> pclP[p].point_right.x + waku_msg -> pclP[p].point_right.z * waku_msg -> pclP[p].point_right.z);//カメラからのRightXの距離を求める
  center_l[p]=std::sqrt(waku_msg -> pclP[p].point_center.x * waku_msg -> pclP[p].point_center.x + waku_msg -> pclP[p].point_center.z * waku_msg -> pclP[p].point_center.z);
  centerleft_l[p]=std::sqrt(waku_msg -> pclP[p].point_centerleft.x * waku_msg -> pclP[p].point_centerleft.x + waku_msg -> pclP[p].point_centerleft.z * waku_msg -> pclP[p].point_centerleft.z);
  centerright_l[p]=std::sqrt(waku_msg -> pclP[p].point_centerright.x * waku_msg -> pclP[p].point_centerright.x + waku_msg -> pclP[p].point_centerright.z * waku_msg -> pclP[p].point_centerright.z);
  left_s[p]=std::atan(-waku_msg -> pclP[p].point_left.x/waku_msg -> pclP[p].point_left.z);//カメラからLeftXまでの角度θを求める
  right_s[p]=std::atan(-waku_msg -> pclP[p].point_right.x/waku_msg -> pclP[p].point_right.z);
  center_s[p]=std::atan(-waku_msg -> pclP[p].point_center.x/waku_msg -> pclP[p].point_center.z);
  centerleft_s[p]=std::atan(-waku_msg -> pclP[p].point_centerleft.x/waku_msg -> pclP[p].point_centerleft.z);
  centerright_s[p]=std::atan(-waku_msg -> pclP[p].point_centerright.x/waku_msg -> pclP[p].point_centerright.z);

  // left_s[p]=std::atan(-waku_msg -> pclP[p].point_left.z/waku_msg -> pclP[p].point_left.x);//カメラからLeftXまでの角度θを求める
  // right_s[p]=std::atan(-waku_msg -> pclP[p].point_right.z/waku_msg -> pclP[p].point_right.x);
  // center_s[p]=std::atan(-waku_msg -> pclP[p].point_center.z/waku_msg -> pclP[p].point_center.x);
  // centerleft_s[p]=std::atan(-waku_msg -> pclP[p].point_centerleft.z/waku_msg -> pclP[p].point_centerleft.x);
  // centerright_s[p]=std::atan(-waku_msg -> pclP[p].point_centerright.z/waku_msg -> pclP[p].point_centerright.x);

  std::cout <<"waku_msg -> pclP[p].point_left.x["<<p<<"]       ="<<waku_msg -> pclP[p].point_left.x<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_centerleft.x["<<p<<"] ="<<waku_msg -> pclP[p].point_centerleft.x<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_center.x["<<p<<"]     ="<<waku_msg -> pclP[p].point_center.x<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_centerright.x["<<p<<"]="<<waku_msg -> pclP[p].point_centerright.x<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_right.x["<<p<<"]      ="<<waku_msg -> pclP[p].point_right.x<<"_m"<< std::endl;

  std::cout <<"waku_msg -> pclP[p].point_left.z["<<p<<"]       ="<<waku_msg -> pclP[p].point_left.z<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_centerleft.z["<<p<<"] ="<<waku_msg -> pclP[p].point_centerleft.z<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_center.z["<<p<<"]     ="<<waku_msg -> pclP[p].point_center.z<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_centerright.z["<<p<<"]="<<waku_msg -> pclP[p].point_centerright.z<<"_m"<< std::endl;
  std::cout <<"waku_msg -> pclP[p].point_right.z["<<p<<"]      ="<<waku_msg -> pclP[p].point_right.z<<"_m"<< std::endl;
  
  std::cout <<"Left_L["<<p<<"]       ="<<left_l[p]<<"_m"<< std::endl;
  std::cout <<"centerleft_l["<<p<<"] ="<<centerleft_l[p]<<"_m"<< std::endl;
  std::cout <<"center_l["<<p<<"]     ="<<center_l[p]<<"_m"<< std::endl;
  std::cout <<"centerright_l["<<p<<"]="<<centerright_l[p]<<"_m"<< std::endl;
  std::cout <<"Right_L["<<p<<"]      ="<<right_l[p]<<"_m"<< std::endl;

  std::cout <<"Left_S["<<p<<"]       ="<<left_s[p]<<"_m"<< std::endl;
  std::cout <<"centerleft_S["<<p<<"] ="<<centerleft_s[p]<<"_m"<< std::endl;
  std::cout <<"center_S["<<p<<"]     ="<<center_s[p]<<"_m"<< std::endl;
  std::cout <<"centerright_S["<<p<<"]="<<centerright_s[p]<<"_m"<< std::endl;
  std::cout <<"Right_S["<<p<<"]      ="<<right_s[p]<<"_m"<< std::endl;


  kosuLCL[p]=std::abs(left_s[p]-centerleft_s[p])/laser_msg -> angle_increment;//LeftXからCenterXの角度θを1サンプリング角度で割ることでLeftからCenterまでの個数を求める
  kosuCLC[p]=std::abs(centerleft_s[p]-center_s[p])/laser_msg -> angle_increment;
  kosuCCR[p]=std::abs(center_s[p]-centerright_s[p])/laser_msg -> angle_increment;
  kosuCRR[p]=std::abs(centerright_s[p]-right_s[p])/laser_msg -> angle_increment;
  
  //henka_l[p]=(left_l[p]-right_l[p])/kosu[p];
  henka_LCL[p]=(left_l[p]-centerleft_l[p])/kosuLCL[p];//距離の差をLeftからCenterまでの個数で割ることで1サンプリングあたりの変化量を求める
  henka_CLC[p]=(centerleft_l[p]-center_l[p])/kosuCLC[p];
  henka_CCR[p]=(center_l[p]-centerright_l[p])/kosuCCR[p];
  henka_CRR[p]=(centerright_l[p]-right_l[p])/kosuCRR[p];

   std::cout <<"kosuLCL["<<p<<"]="<<kosuLCL[p]<< std::endl;
   std::cout <<"kosuCLC["<<p<<"]="<<kosuCLC[p]<< std::endl;
   std::cout <<"kosuCCR["<<p<<"]="<<kosuCCR[p]<< std::endl;
   std::cout <<"kosuCRR["<<p<<"]="<<kosuCRR[p]<< std::endl;
   std::cout <<"henka_LCL["<<p<<"]="<<henka_LCL[p]<<std::endl;
   std::cout <<"henka_CLC["<<p<<"]="<<henka_CLC[p]<<std::endl;
   std::cout <<"henka_CCR["<<p<<"]="<<henka_CCR[p]<<std::endl;
   std::cout <<"henka_CRR["<<p<<"]="<<henka_CRR[p]<<std::endl;

  LCL_l[p].resize(kosuLCL[p]);
  CLC_l[p].resize(kosuCLC[p]);
  CCR_l[p].resize(kosuCCR[p]);
  CRR_l[p].resize(kosuCRR[p]);
  LCL_s[p].resize(kosuLCL[p]);
  CLC_s[p].resize(kosuCLC[p]);
  CCR_s[p].resize(kosuCCR[p]);
  CRR_s[p].resize(kosuCRR[p]);

  // LL[p].resize(laserScan_src.ranges.size());
  // LCL[p].resize(laserScan_src.ranges.size());
  // RL[p].resize(laserScan_src.ranges.size());
  // RCL[p].resize(laserScan_src.ranges.size());
  // LS[p].resize(laserScan_src.ranges.size());
  // LCS[p].resize(laserScan_src.ranges.size());
  // RS[p].resize(laserScan_src.ranges.size());
  // RCS[p].resize(laserScan_src.ranges.size());

  //ROS_INFO("koko1");

for(i=0;i<(kosuLCL[p]);i++){
    //ROS_INFO("koko2");

    LCL_l[p][i] = left_l[p] - henka_LCL[p] * i;//LB番号の距離の値 
    LCL_s[p][i] = left_s[p]-laser_msg -> angle_increment*i;//LB番号の角度の値

    std::cout <<"LCL_l="<<LCL_l[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 
    std::cout <<"LCL_s="<<LCL_s[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 

    int LCLB,RB;
    LCLB=(LCL_s[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//rengesの要素番号を求める(LBは要素番号)

    if(LCLB>=laserScan_src.ranges.size()||LCLB<0){}
     else{ 
       laserScan_src.ranges[LCLB] = LCL_l[p][i];
       std::cout <<"LCLB["<<i<<"]="<<LCLB<< std::endl; }//rengesの要素(距離データ)書き換え 
     //ROS_INFO("koko3");
}//for文i

for(i=0;i<(kosuCLC[p]);i++){

    CLC_l[p][i] = centerleft_l[p] - henka_CLC[p] * i;//LB番号の距離の値 
    CLC_s[p][i] = centerleft_s[p]-laser_msg -> angle_increment*i;//LB番号の角度の値
    std::cout <<"CLC_l="<<CLC_l[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 
    std::cout <<"CLC_s="<<CLC_s[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 

    int CLCB,RB;
    CLCB=(CLC_s[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//rengesの要素番号を求める(LBは要素番号)

    if(CLCB>=laserScan_src.ranges.size()||CLCB<0){}
     else{ laserScan_src.ranges[CLCB] = CLC_l[p][i];std::cout <<"CLCB["<<i<<"]="<<CLCB<< std::endl; }//rengesの要素(距離データ)書き換え 
     //ROS_INFO("koko4");
}//for文i

for(i=0;i<kosuCCR[p];i++){
    CCR_l[p][i] = center_l[p] - henka_CCR[p] * i;//RB番号の距離
    CCR_s[p][i]=center_s[p]-laser_msg -> angle_increment*i;//RB番号の角度

    std::cout <<"CCR_l="<<CCR_l[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 
    std::cout <<"CCR_s="<<CCR_s[p][i]<< std::endl; //rengesの要素(距離データ)書き換え

    int CCRB;
    CCRB=(CCR_s[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//要素番号

    if(CCRB>=laserScan_src.ranges.size()||CCRB<0){}
    else{ laserScan_src.ranges[CCRB] = CCR_l[p][i]; std::cout <<"CCRB="<<CCRB<< std::endl;}//rengesの要素(距離データ)書き換え

    //ROS_INFO("koko5");

}//for文i

for(i=0;i < kosuCRR[p];i++){
    CRR_l[p][i] = centerright_l[p] - henka_CRR[p] * i;//RB番号の距離
    CRR_s[p][i]= centerright_s[p]-laser_msg -> angle_increment*i;//RB番号の角度

    std::cout <<"CRR_l="<<CRR_l[p][i]<< std::endl; //rengesの要素(距離データ)書き換え 
    std::cout <<"CRR_s="<<CRR_s[p][i]<< std::endl; //rengesの要素(距離データ)書き換え

    int CRRB;
    CRRB=(CRR_s[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//要素番号

    if(CRRB>=laserScan_src.ranges.size()||CRRB<0){}
    else{ laserScan_src.ranges[CRRB] = CRR_l[p][i]; std::cout <<"CRRB="<<CRRB<< std::endl;}//rengesの要素(距離データ)書き換え

    //ROS_INFO("koko6");
}//for文i



}//for文p

//プログラミング実装用
laserScan_src.header.stamp = ros::Time::now();//pubされた現在時刻を記録
laser_pub.publish(laserScan_src);//データ送信

//プログラミング非実装用
// laser_pub.publish(*laser_msg);//データ送信

cv::waitKey(1);

}

int main(int argc,char **argv){//int argc=引数の個数 char **argv=引数の配列
	ros::init(argc,argv,"deptu_pcl4_sub"/*ノードの名前*/);

    //データ受け取り設定	
	ros::NodeHandle nhSub;//topic設定で使う
	message_filters::Subscriber<struct_slam::wakuhairetu> waku_sub(nhSub, "wakuhairetu", 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nhSub, "/robot1/scan", 1);

    
	typedef message_filters::sync_policies::ApproximateTime<struct_slam::wakuhairetu,sensor_msgs::LaserScan> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),waku_sub, laser_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

  ros::NodeHandle nhPub;//topic設定で使う
  laser_pub/*pub変数名*/=nhPub.advertise<sensor_msgs::LaserScan/*変数型*/>("Laser_dst"/*トピック名*/, 1000);

    	
	ros::spin();
	
	return 0;
}
