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


