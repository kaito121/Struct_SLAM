//hsvで色でフィルタを作りあるしきい値以上の色のみをマスクで取り出す
//その後取り出した画像に対し膨張圧縮処理を施しノイズ除去
//マスク処理してある１面積のみにした画像に対し重心を求め、その重心の移動値からカルマンフィルタを使い位置を推定する
//つまりこのプログラムは１つのマークに対してのみしか処理が行われていない
//このプログラムはリアルタイムでコロコロの緑色部分を追跡するプログラム(20210701)
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

int main(int argc, const char * argv[]) {
    //マーカ辞書作成 6x6マスのマーカを250種類生成
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    //charucoボード生成 10x7マスのチェスボード、グリッドのサイズ0.04f、グリッド内マーカのサイズ0.02f
    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 7, 0.04f, 0.02f, dictionary);
    cv::Mat boardImage;
    board->draw(cv::Size(1920, 1080), boardImage, 10, 1);
    cv::imwrite("/home/fuji/catkin_ws/src/Struct_SLAM/src/marker/BoardImage.jpg", boardImage);
}
