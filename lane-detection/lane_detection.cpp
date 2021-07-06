#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


#include <sstream>
#include <fstream>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

bool lineIntersection(Mat a, Mat& b, Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4);



int main(void)
{
	VideoCapture cap("driving2.mp4");

	if (!cap.isOpened())
	{
		printf("Can't open the camera");
		return -1;
	}
	String filename;
	int cnt = 1;
	Mat img;

	while (1)
	{

		cap >> img;

		if (img.empty())
		{
			printf("empty image");
			return 0;
		}

		Mat img_gray;

		cvtColor(img, img_gray, COLOR_BGR2GRAY);

		Mat img_canny;
		Canny(img_gray, img_canny, 350, 350);

		Mat img_point = Mat::zeros(img_canny.rows, img_canny.cols, CV_8U);
		int finalj1 = 0, finalj2 = 0;

		for (int i = img_canny.rows - 1; i > int(img_canny.rows * 2 / 3); i--)
		{
			for (int j = 0; j < int(img_canny.cols / 3); j++)
			{
				if (img_canny.at<uchar>(i, j) > 60)
				{
					finalj1 = j;
				}

			}


			for (int j = int(img_canny.cols * 3 / 4); j > int(img_canny.cols * 2 / 5); j--)
			{

				if (img_canny.at<uchar>(i, j) > 60)
				{
					finalj2 = j;
				}

			}

			img_point.at<uchar>(i, finalj1) = 255;

			img_point.at<uchar>(i, finalj2) = 255;

		}

		vector<Vec2f> lines;
		HoughLines(img_point, lines, 1, CV_PI / 180, 10);

		Mat img_hough;
		img.copyTo(img_hough);

		float finalrho1 = 0, finaltheta1 = 0, finalrho2 = 0, finaltheta2 = 0;
		Point final1, final2;

		for (size_t i = 0; i < lines.size() - 1; i++)
		{
			float rhoC = lines[i][0], thetaC = lines[i][1];
			float rhoN = lines[i + 1][0], thetaN = lines[i + 1][1];
			if (tan(thetaC) > 0.5 && tan(thetaC) < 1.5)
			{
				if (tan(thetaN) > 0.5 && tan(thetaN) < 1.5)
				{
					if (tan(thetaC) >= tan(thetaN))
					{
						finalrho1 = rhoC;
						finaltheta1 = thetaC;
					}
					else
					{
						finalrho1 = rhoN;
						finaltheta1 = thetaN;
					}
				}
				else
				{
					finalrho1 = rhoC;
					finaltheta1 = thetaC;
				}
			}
			else if (tan(thetaC) < (-0.5) && tan(thetaC) > (-1.5))
			{
				if (tan(thetaN) < (-0.5) && tan(thetaN) > (-1.5))
				{
					if (tan(thetaC) <= tan(thetaN))
					{
						finalrho2 = rhoC;
						finaltheta2 = thetaC;
					}
					else
					{
						finalrho2 = rhoN;
						finaltheta2 = thetaN;
					}
				}
				else
				{
					finalrho2 = rhoC;
					finaltheta2 = thetaC;
				}
			}
		}

		Point2f pt1, pt2, pt3, pt4;
		Point2f pt5;
		double a, b, c, d;
		double x01, y01, x02, y02;

		a = cos(finaltheta1); b = sin(finaltheta1);
		x01 = a * finalrho1; y01 = b * finalrho1;
		pt1.x = cvRound(x01 + 1000 * (-b));
		pt1.y = cvRound(y01 + 1000 * (a));
		pt2.x = cvRound(x01 - 1000 * (-b));
		pt2.y = cvRound(y01 - 1000 * (a));

		// Y = (pt1.y- pt2.y)/(pt1.x - pt2.x)X+B; X = 222, B = 


		c = cos(finaltheta2); d = sin(finaltheta2);
		x02 = c * finalrho2; y02 = d * finalrho2;
		pt3.x = cvRound(x02 + 1000 * (-d));
		pt3.y = cvRound(y02 + 1000 * (c));
		pt4.x = cvRound(x02 - 1000 * (-d));
		pt4.y = cvRound(y02 - 1000 * (c));

		Mat lineimage = Mat::zeros(img.rows, img.cols, CV_8UC3);
		if (finaltheta1 != 0)
		{
			if (finaltheta2 != 0)
			{
				line(lineimage, pt1, pt2, Scalar(0, 0, 255), 2, 8);
				line(lineimage, pt3, pt4, Scalar(0, 255, 0), 2, 8);
			}

		}

		Mat finalimage;

		lineIntersection(lineimage, finalimage, pt1, pt2, pt3, pt4);

		finalimage = finalimage + img;

		cv::imshow("img", finalimage);

		filename = "result" + to_string(cnt) + ".jpg";

		//imwrite(filename, finalimage);

		cnt++;

		if (cv::waitKey(25) == 27)
			break;

	}



	return 0;
}


bool lineIntersection(Mat a, Mat& b, Point2f pt1, Point2f pt2, Point2f pt3, Point2f pt4)
{
	b = Mat::zeros(a.rows, a.cols, CV_8UC3);
	Point2f pt(0, 0);

	int i, j, k, m;
	for (i = (int)(a.rows * 3 / 4); i > (int)(a.rows / 2); i--)
	{
		for (j = (int)(a.cols / 3); j < (int)(a.cols * 2 / 3); j++)
		{
			if (a.at<Vec3b>(i, j)[2] == 255)
			{
				if (a.at<Vec3b>(i, j + 1)[1] == 255)
				{
					line(b, pt1, pt2, Scalar(0, 0, 255), 2, 8, 0);
					line(b, pt3, pt4, Scalar(0, 255, 0), 2, 8, 0);

					Mat mask = Mat::zeros(i, b.cols, CV_8UC3);
					mask.copyTo(b.rowRange(0, i).colRange(0, b.cols));

					pt.x = j;
					pt.y = i;

					circle(b, pt, 10, Scalar(255, 0, 0), 2, 8);

					return true;
				}
			}

		}

	}
	return false;
}

