#include "ros/ros.h"  // rosで必要はヘッダーファイル
#include <geometry_msgs/Twist.h> // ロボットを動かすために必要
#include <unistd.h>//waitを使用
using namespace std;
int kaisu=0;

struct timeval startTime, endTime;  // 構造体宣言
float realsec;//サンプリング時間（C++)
float ALLrealsec;//サンプリング時間（C++)

double roll, pitch, yaw;//クオータニオン→オイラー角変換用
double Act_RobotX=0,Act_RobotY=0,Act_RobotTH=0;//ロボットの状態方程式(実際の状態)
double Des_RobotX=0,Des_RobotY=0,Des_RobotTH=0;//ロボットの状態方程式(理想状態)
double Est_RobotX=0,Est_RobotY=0,Est_RobotTH=0;//ロボットの状態方程式(推定状態)
double Act_RobotV,Des_RobotV;//ロボットの速度ベクトル(実測値,指令値)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_teleop_node");
    // initでROSを初期化し、my_teleop_nodeという名前をノードにつける
    // 同じ名前のノードが複数あるとだめなので、ユニークな名前をつける

    ros::NodeHandle nh;
    // ノードハンドラの作成。ハンドラは必要時に起動される。

    ros::Publisher  pub;
    // パブリッシャの作成。トピックに対してデータを送信。

    ros::Rate rate(10);
    // ループの頻度を設定するためのオブジェクトを作成。この場合は10Hz、1秒間に10回数、1ループ100ms。

    geometry_msgs::Twist robot_velocity;
    // geometry_msgs::Twist　この型は並進速度と回転速度(vector3:3次元ベクトル) を合わせたもので、速度指令によく使われる

    pub= nh.advertise<geometry_msgs::Twist>("/robot1/mobile_base/commands/velocity", 10);
    // マスターにgeometry_msgs::Twist型のデータを送ることを伝える
    // マスターは/cmd_velトピック(1番目の引数）を購読する
    // 全てのノードにトピックができたことを知らせる(advertise)。
    // 2番目の引数はデータのバッファサイズ

    cout << "f: forward, b: backward, r: right, l:left" << endl;
    //直進するプログラム(xが1になったら実行終了)

    for(int i=0;i<1000000000;i++){
      //サンプリング時間取得(C言語の方法)(こっちのほうが正確らしい)
        gettimeofday(&startTime, NULL);// 開始時刻取得
        if(kaisu!=0){
          time_t diffsec = difftime(startTime.tv_sec,endTime.tv_sec);    // 秒数の差分を計算
          suseconds_t diffsub = startTime.tv_usec - endTime.tv_usec;      // マイクロ秒部分の差分を計算
          realsec = diffsec+diffsub*1e-6;                          // 実時間を計算
          ALLrealsec=ALLrealsec+realsec;
          //printf("処理の時間=%f\n", realsec);
          //printf("処理時間合計=%f\n", ALLrealsec);
        }

      robot_velocity.linear.x  =  0.1;
      robot_velocity.angular.z  =  0;
      pub.publish(robot_velocity);    // 速度指令メッセージをパブリッシュ（送信）

      Des_RobotV=robot_velocity.linear.x+robot_velocity.linear.y;//速度ベクトルの合成
      std::cout << "Des_RobotV=" <<Des_RobotV<< std::endl;
      //des_v<<Des_RobotV <<"\n";
      //des_u<<robot_velocity.angular.z <<"\n";

        ////目標指令状態
      if(robot_velocity.angular.z==0){
        Des_RobotX=Des_RobotX+(Des_RobotV*cos(Des_RobotTH)*realsec);
        Des_RobotY=Des_RobotY+(Des_RobotV*sin(Des_RobotTH)*realsec);
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }
      else{
        Des_RobotX=Des_RobotX+((Des_RobotV/robot_velocity.angular.z)*(sin(Des_RobotTH+robot_velocity.angular.z*realsec)-sin(Des_RobotTH)));
        Des_RobotY=Des_RobotY+((Des_RobotV/robot_velocity.angular.z)*(-cos(Des_RobotTH+robot_velocity.angular.z*realsec)+cos(Des_RobotTH)));
        Des_RobotTH=Des_RobotTH+(robot_velocity.angular.z*realsec);
      }

      std::cout << "Des_RobotX=" <<Des_RobotX<< std::endl;
      std::cout << "Des_RobotY=" <<Des_RobotY<< std::endl;
      std::cout << "Des_RobotTH=" <<Des_RobotTH<< std::endl;


      if(Des_RobotX>=1.00){break;}//xが1以上になったら終了
      usleep(5 * 10000);//0.05秒ストップ(マイクロ秒)
      endTime=startTime;//動作終了時刻取得
      kaisu++;
    }

      robot_velocity.linear.x  = 0.0; // 並進速度の初期化
      robot_velocity.angular.z = 0.0; // 回転速度の初期化





    /*while (ros::ok()) { // このノードが使える間は無限ループする
        char key;  // 入力キーの値

        cin >> key;// 標準入力からキーを読み込む
        cout << key << endl; // 読み込んだキーの値を標準出力へ出力

        switch (key) {
        case 'f': // fキーが押されていたら
            vel.linear.x  =  0.5;
            break;
        case 'b':
            vel.linear.x  = -0.5;
            break;
        case 'l':
            vel.angular.z =  1.0;
            break;
        case 'r':
            vel.angular.z = -1.0;
            break;
            // linear.xは前後方向の並進速度(m/s)。前方向が正。
            // angular.zは回転速度(rad/s)。反時計回りが正。
        }

        pub.publish(vel);    // 速度指令メッセージをパブリッシュ（送信）
        ros::spinOnce();     // １回だけコールバック関数を呼び出す
        vel.linear.x  = 0.0; // 並進速度の初期化
        vel.angular.z = 0.0; // 回転速度の初期化
        rate.sleep();        // 指定した周期でループするよう寝て待つ

    }*/


    return 0;
}
