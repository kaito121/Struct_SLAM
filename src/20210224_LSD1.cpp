#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>

int main( int argc, char** argv )
{
    //白黒画像(判定用)
    cv::Mat imageMat = cv::imread("stb1.jpg", cv::IMREAD_GRAYSCALE );
    //カラー画像(表示用)
    cv::Mat imageMat_color = cv::imread("stb1.jpg", cv::IMREAD_COLOR  );
    std::vector<cv::Vec4f> lines;

        //特徴線クラスオブジェクトを作成
    cv::Ptr<cv::ximgproc::FastLineDetector> fld =  cv::ximgproc::createFastLineDetector();

        //特徴線検索
    fld->detect( imageMat, lines);

    for (int i = 0; i < lines.size(); ++i) {
        //線の始点
        cv::Point pt1 = cv::Point2f( lines[i][0], lines[i][1] );
        //線の終点
        cv::Point pt2 = cv::Point2f( lines[i][2], lines[i][3] );
        cv::line( imageMat_color, pt1, pt2, cv::Scalar( 0, 0, 255 ), 2);
    }

    cv::namedWindow("line features", cv::WINDOW_AUTOSIZE);
    cv::imshow("line features", imageMat_color);

    cv::waitKey(0);

    cv::destroyAllWindows();
}



/*#include <iostream>
#include <opencv2/opencv.hpp>
#include "lsd.h"
 
cv::Mat img;
int n_lines;
double* lines;
 
void change_th_lsd(int nfa, void* dummy)
{
    cv::Mat result = img.clone();
    for(int i = 0; i < n_lines; i++)
    {
        const double *line = &lines[i * 7];
        if(nfa < line[6])
        {
            const cv::Point p1(line[0], line[1]);
            const cv::Point p2(line[2], line[3]);
            cv::line(result, p1, p2, cv::Scalar(0, 0, 255));
        }
    }
    cv::imshow("result_image", result);
}
 
int main(int argc, char *argv[])
{
    //画像をグレースケールとして読み込み
    img = cv::imread(argv[1], 0);
 
    //LSD用画像に変換＞＜
    double *dat = new double[img.rows * img.cols];
    for(int y = 0; y < img.rows; y++)
        for(int x = 0; x < img.cols; x++)
            dat[y * img.cols + x] = img.at<unsigned char>(y, x);
 
    //LSD処理
    //lines = lsd(&n_lines, dat, img.cols, img.rows);
    cv::

 
    //しきい値の最大値と最小値をもってくる
    int max_NFA = 0;
    for(int i = 0; i < n_lines; i++)
        max_NFA = std::max(max_NFA, static_cast<int>(lines[i * 7 + 6]));
 
    //結果描画用画像
    cv::cvtColor(img, img, CV_GRAY2RGB);
 
    //結果表示用ウィンドウ
    cv::namedWindow("result_image");
    cv::createTrackbar("NFA", "result_image", NULL, max_NFA, change_th_lsd);
    cv::setTrackbarPos("NFA", "result_image", max_NFA);
 
    //結果表示
    cv::imshow("result_image", img);
    cv::waitKey(0);
}*/