#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#define DATA_NUM 4  // データ数
//#define DATA_NUM 56  // データ数
#define MAX_DATA 6.0  // データの最大値
//#define MAX_DATA 2  // データの最大値


int main(int argc, char **argv){
    ros::init(argc, argv, "hello_node");
    ROS_INFO("hello world");

   // 変数定義
    //int data[DATA_NUM] = {6,3,5,9,8,10,2,10,4,9,
    //                      7,9,9,10,6,5,10,9,5,8,
    //                      11,4,9,10,4,5,9,7,6,100};

    //double data[DATA_NUM] = {1.0,1.1,1.2,1.3,1.5,1.5,1.5,2.0,2.0,2.0,2.0};
    //double data[DATA_NUM] = {1.0,2.0,4.0,6.0};
    double data[DATA_NUM] = {3.01,3.02,3.04,3.06};




    double tmp;
    double med;  // メジアン
    double renge;  // レンジ
    //double histogram[MAX_DATA + 1];  // ヒストグラム

    // 変数の初期化
    int i = 0;
    int j = 0;
    tmp = 0;
    med = 0;
    renge = 0;
    //memset(histogram, '\0', sizeof(histogram));  // 配列の中身を初期化する

    //データを大きさの順に並べ替え
    for(i = 1; i < DATA_NUM; i++){
        for(j = 0; j < DATA_NUM - i; j++){
            if(data[j] > data[j + 1]){
                tmp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = tmp;
            }
        }
    }

    // メジアンを求める
    if(DATA_NUM % 2 == 1){  // データ数が奇数個の場合
        med = data[(DATA_NUM - 1) / 2];  // メジアン
    }
    else{  // データ数が偶数の場合
        med = (data[(DATA_NUM / 2) - 1] + data[DATA_NUM / 2]) / 2.0;  // メジアン
    }

    // レンジを求める
    renge = data[DATA_NUM - 1] - data[0] + 0.0;  // 範囲


    std::cout <<"メジアンは"<<med<< std::endl;
    std::cout <<"範囲    は"<<renge<< std::endl;

    // histogram[]はヒストグラムで使用する配列
    /*for(i = 0; i <= MAX_DATA; i++)
    {
        histogram[i] = 0;
    }

    //0≦x≦10であるdata[i]のデータxの個数を
    //histogram[x]の数値と対応させる
    for(i = 0; i < DATA_NUM; i++)
    {
        histogram[data[i]]++;
    }

    printf("\nヒストグラム\n");
    for(i = 0; i <= MAX_DATA; i++)
    {
        printf("%2.0d|", i);  // 座標軸表示
        for(j = 0; j < histogram[i]; j++)
        {
            printf("*");  // データの個数分"*"を表示
        }
        printf("\n");
    }*/

    return 0;
}