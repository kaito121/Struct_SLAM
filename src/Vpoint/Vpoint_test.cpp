/*
May 2015
		  Authors:
Ankit Dhall     Yash Chandak
TO DO:
-set capture frame size in VideoCapture object
-take lines only closer to the region of the vanishing points
	- plug the prevRes in the eqns for new frame and consider lines only with error < |epsilon|
-cluster more if more than 2 vanishing points are present (advanced)
-refactor the CODE
-error calculation, remove so many divisions!
*/
#include <iostream>
#include <armadillo>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <fstream>
#include <math.h>
#include <ctime>


//add OpenCV and Armadillo namespaces
using namespace cv;
using namespace std;
using namespace arma;

class vanishingPt
{
	public:
	cv::Mat image, img, gray;
	cv::Mat frame;
	vector< vector<int> > points;
	mat A,b, prevRes;
	mat Atemp, btemp, res, aug, error, soln;
	//ofstream out1, out2;
	float epsilon;

	//store slope (m) and y-intercept (c) of each lines(各直線の傾き(m)とY字型切片(c)を格納する。)
	float m,c;

	//store minimum length for lines to be considered while estimating vanishing point
	//消失点を推定する際に考慮すべき線の最小長さを保存する。
	int minlength;

	//temporary vector for intermediate storage(中間貯蔵のための一時的なベクトル)
	vector<int> temp;

	//store (x1, y1) and (x2, y2) endpoints for each line segment(各線分の端点(x1, y1)と(x2, y2)を格納する)
	vector<cv::Vec4i> lines_std;

	//video capture object from OpenCV
	cv::VideoCapture cap;

	//to store intermediate errors(中間的なエラーを格納する)
	double temperr;

	//constructor to set video/webcam and find vanishing point(ビデオ/ウェブカメラをセットし、消失点を見つけるためのコンストラクタ)
	vanishingPt()
	{
		cv::namedWindow("win", 2);
		cv::namedWindow("Lines", 2);
		
		// to calculate fps(fpsを計算する)
		clock_t begin, end;

		//read from video file on disk
		//to read from webcam initialize as: cap = VideoCapture(int device_id);
		//ディスク上のビデオファイルから読み取る場合
		//ウェブカメラから読み取る場合 以下のように初期化します： cap = VideoCapture(int device_id);
		//cap = VideoCapture(1);
		cap = VideoCapture("road.m4v");

		if( cap.isOpened() )//check if camera/ video stream is available(//カメラ/ビデオストリームが利用可能かどうかをチェック)
		{
			//get first frame to intialize the values(値を初期化するための最初のフレームを取得する)
			cap.read(frame);
        	image= cv::Mat(cv::Size(frame.rows,frame.cols), CV_8UC1, 0.0);
		}
    	
    	// define minimum length requirement for any line(あらゆるラインの最小長さの要件を定義する)
		minlength = image.cols * image.cols * 0.001 ;

	    int flag=0;
		while( cap.isOpened() )//check if camera/ video stream is available(カメラ/ビデオストリームが利用可能かどうかをチェック)
		{
		    if ( ! cap.grab() )
		        continue;

			if(!cap.retrieve(img))
				continue;

			//it's advised not to modify image stored in the buffer structure of the opencv.
			////opencv.orgのバッファ構造に格納されている画像を変更しないことをお勧めします。
			frame  = img.clone();

			//to calculate fps(fpsを計算する)
			begin = clock();
			
			cv::cvtColor(frame,image , cv::COLOR_BGR2GRAY);

			//resize frame to 480x320
			cv::resize(image, image, cv::Size(480,320));

			//equalize histogram(ヒストグラムの均等化)
			cv::equalizeHist(image, image);

			//initialize the line segment matrix in format y = m*x + c	(y = m*x + c という形式で線分行列を初期化する。)
			init(image, prevRes);

			//draw lines on image and display(画像に線を引いて表示する)
			makeLines(flag);
			
			//approximate vanishing point(消失点の近似値)
			eval();

			//to calculate fps(fpsを計算する)
			end = clock();
			cout<<"fps: "<<1/(double(end-begin)/CLOCKS_PER_SEC)<<endl;

			//hit 'esc' to exit program('esc'を押してプログラムを終了)
		    int k = cv::waitKey(1);
		    if ( k==27 )
		        break;
		}

	}
	void init(cv::Mat image, mat prevRes)
	{
		//create OpenCV object for line segment detection(線分検出用のOpenCVオブジェクトの作成)
		cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);

		//initialize
    	lines_std.clear();

    	//detect lines in image and store in linse_std(画像中の線を検出し，linse_std に格納する)
    	//store (x1, y1) and (x2, y2) endpoints for each line segment(各線分の端点(x1, y1)と(x2, y2)を格納する)
    	ls->detect(image, lines_std);

    	// Show found lines
    	cv::Mat drawnLines (image);

		for(int i=0; i<lines_std.size(); i++)
		{

			//ignore if almost vertical(ほぼ垂直の場合は無視)
			if ( abs(lines_std[i][0]-lines_std[i][2]) < 10 || abs(lines_std[i][1]-lines_std[i][3]) < 10) //check if almost vertical
				continue;
			//ignore shorter lines (x1-x2)^2 + (y2-y1)^2 < minlength(短い行を無視する (x1-x2)^2 + (y2-y1)^2 < minlength)
			if( ((lines_std[i][0]-lines_std[i][2])*(lines_std[i][0]-lines_std[i][2]) +(lines_std[i][1]-lines_std[i][3])*(lines_std[i][1]-lines_std[i][3])) < minlength)
				continue;

            //store valid lines' endpoints for calculations(有効なラインの端点を計算のために保存)
			for(int j=0; j<4; j++)
			{
				temp.push_back(lines_std[i][j]);
			}

			points.push_back(temp);
			temp.clear();
		}
		ls->drawSegments(drawnLines, lines_std);
		cv::imshow("Lines", drawnLines);
		//cout<<"Detected:"<<lines_std.size()<<endl;
		//cout<<"Filtered:"<<points.size()<<endl;
	}
	void makeLines(int flag)
	{
		// to solve Ax = b for x(を使って、Ax = bをxについて解く)
	    A = zeros<mat>(points.size(), 2);
	    b = zeros<mat>(points.size(), 1);

	    //convert given end-points of line segment into a*x + b*y = c format for calculations(与えられた線分の端点を、計算のためにa*x + b*y = c 形式に変換する) 
	    //do for each line segment detected(検出された各線分に対して行う)
	    for(int i=0; i<points.size(); i++)
	    {
            
			A(i,0)=-(points[i][3]-points[i][1]);			//-(y2-y1)
			A(i,1)=(points[i][2]-points[i][0]);				//x2-x1
			b(i,0)=A(i,0)*points[i][0]+A(i,1)*points[i][1];	//-(y2-y1)*x1 + (x2-x1)*y1
	    }
	}

	//estimate the vanishing point(消失点の推定)
	void eval()
	{
		//stores the estimated co-ordinates of the vanishing point with respect to the image(は、画像に対する消失点の座標の推定値を格納します。)
		soln= zeros<mat>(2,1);

		//initialize error(初期化エラー)
		double err = 9999999999;

		//calculate point of intersection of every pair of lines and
		//find the sum of distance from all other lines
		//select the point which has the minimum sum of distance
		//すべての直線のペアの交点を計算し、他のすべての直線からの距離の合計を求める。
		//距離の合計が最小となる点を選択する。
		for(int i=0; i<points.size(); i++)
		{
			for(int j=0; j<points.size(); j++)
			{
				if(i >= j)
				continue;

				//armadillo vector
				uvec indices;

				//store indices of lines to be used for calculation(計算に使用するラインのインデックスを格納する)
				indices << i << j;

				//extract the rows with indices specified in uvec indices(uvec indices で指定されたインデックスを持つ行を抽出します。)
				//stores the ith and jth row of matrices A and b into Atemp and btemp respectively
				//行列 A と b の i 行目と j 行目をそれぞれ Atemp と btemp に格納します．
				//hence creating a 2x2 matrix for calculating point of intersection
				//従って，交点を計算するための2x2の行列を作成する
				Atemp = A.rows(indices);
				btemp = b.rows(indices);

				//if lines are parallel then skip(ラインが平行であればスキップ)
				//if(rank(Atemp) != 2)
				//if(arma::rank(Atemp) != 2)
			 	 //continue; 

				//solves for 'x' in A*x = b(A*x = bの'x'を解きます。)
				res = calc(Atemp, btemp);


				if(res.n_rows == 0 || res.n_cols == 0)
					continue;

				// calculate error assuming perfect intersection is (完全に交差していると仮定して誤差を計算すると )
				error = A*res - b;

				//reduce size of error(誤差の縮小)
				error = error/1000;

				// to store intermediate error values(中間的なエラー値を格納する)
				temperr = 0;
				//summation of errors(誤差の総和)
				for(int i=0; i<error.n_rows ; i++)
                    temperr+=(error(i,0)*error(i,0))/1000;

                //scale errors to prevent any overflows(オーバーフローを防ぐためのスケールエラー)
				temperr/=1000000;

				//if current error is smaller than previous min error then update the solution (point)
				//現在の誤差が前回の最小誤差よりも小さければ、解を更新する(ポイント)
				if(err > temperr)
				{
					soln = res;
					err = temperr;
				}
			}
		}

		//cout<<"\n\nResult:\n"<<soln(0,0)<<","<<soln(1,0)<<"\nError:"<<err<<"\n\n";

		// draw a circle to visualize the approximate vanishing point(円を描いておおよその消失点をイメージする)
		if(soln(0,0) > 0 && soln(0,0) < image.cols && soln(1,0) > 0 && soln(1,0) < image.rows)
			cv::circle(image, Point(soln(0,0), soln(1,0)), 25, cv::Scalar(0,0,255), 10);
		cv::imshow("win", image);

		//flush the vector
		points.clear();

		//toDo: use previous frame's result to reduce calculations and stabilize the region of vanishing point
		//toDo: 前のフレームの結果を使って計算を減らし、消失点の領域を安定させる
		prevRes = soln;
	}

	//function to calculate and return the intersection point(交差点を計算して返す関数)
	mat calc(mat A, mat b)
	{
	    mat x = zeros<mat>(2,1);
		solve(x,A,b);
	    return x;
	}
};


int main()
{
	// make object
    vanishingPt obj;
    cv::destroyAllWindows();
    return 0;
}
