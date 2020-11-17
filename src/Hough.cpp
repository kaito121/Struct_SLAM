#define _CRT_SECURE_NO_WARNIGS
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
std::string win_src = "src";
std::string win_edge = "edge";
std::string win_dst = "dst";

int main(){
    cv::Mat img_src = cv::imread("image/DSC_0633.JPG",1);
    cv::Mat img_gray,img_edge,img_dst;

    if(!img_src.data){
        std::cout << "error" << std::endl;
        return -1;
    }
    img_src.copyTo(img_dst);
    cv::cvtColor(img_src, img_gray, cv::COLOR_RGB2GRAY);

    cv::Canny(img_gray, img_edge, 200, 200);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(img_edge, lines, 1, CV_PI/180, 120);
    for(int i = 0; i < lines.size(); i++){
        double rho = lines[i][0], theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;

        cv::line(img_dst,
            cv::Point(x0 - img_dst.cols*b, y0 + img_dst.cols*a),
            cv::Point(x0 + img_dst.cols*b, y0 - img_dst.cols*a),
            cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }

    cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_edge, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

    cv::imshow(win_src, img_src);
    cv::imshow(win_edge, img_edge);
    cv::imshow(win_dst, img_dst);

    cv::waitKey(0);

    return 0;

}