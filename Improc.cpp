#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/objdetect.hpp>
#include<conio.h> 
#include <Windows.h>
#include <cstdio>
#include <mmsystem.h>
#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include<chrono>
#include<string>
#include<stdlib.h>
#include <cmath>

#include "Improc.h"
#include "Robot.h"
#include "Line.h"
#pragma comment(lib, "winmm.lib")
using namespace cv;
using namespace std;



void Improc::shape_mom() {
	//function finds the properties that stop_go() uses to find the stop/go
	//signs. They are both hexogonal in shape. The function obtains the perimeter
	//aspect ratio, and the angle the vertices make with repect to the centroid.
	Mat gray, iblur, canny;
	
	Point c;
	double x, y, m_r, peri;
	int ar;
	const double pi = 3.141592653589;

	cvtColor(img, gray, COLOR_BGR2GRAY);
	GaussianBlur(gray, iblur, Size(11, 11), 0, 0);
	Canny(gray, canny, 255, 90);

	//imshow("mom func", img);

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(canny, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	vector<vector<Point> >hull(contours.size());
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());
	vector<vector<Point>> conPoly(contours.size());
	vector<Rect> boundRect(contours.size());

	vector<Point>vcs;
	Point vtx;

	for (size_t i = 0; i < contours.size(); i++) {
		Scalar color = Scalar(256, 180, 200);
		convexHull(contours[i], hull[i]);
		peri = arcLength(hull[i], true);
		//drawContours(img, hull, (int)i, color, 2, LINE_8, hierarchy, 0);
		//the centre is found using the image moments
		mu[i] = moments(hull[i]);
		mc[i] = Point2f(static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
			static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)));

		approxPolyDP(hull[i], conPoly[i], 0.04 * peri, true);

		vcs = conPoly[i];
		len = conPoly[i].size();

		boundRect[i] = boundingRect(hull[i]);
		ar = contourArea(hull[i]);


		as_r = ((double)boundingRect(hull[i]).width / (double)boundingRect(hull[i]).height);

		c.x = (mu[i].m10 / (mu[i].m00 + 1e-5));
		c.y = (mu[i].m01 / (mu[i].m00 + 1e-5));

		x = (mu[i].m10 / (mu[i].m00 + 1e-5));
		y = (mu[i].m01 / (mu[i].m00 + 1e-5));

		for (int i = 0; i < vcs.size(); i++) {

			vtx = vcs[i];
			double t_c; //theta wrt to center for each point
			t_c = atan((abs(vtx.y - c.y)) / (abs(vtx.x - c.x) + 1e-15)) * (180 / pi);
			t_w_c.push_back(t_c);
		}
	}

}

void Improc::get_mask() {
	//retrieves the mask of the arrow images for
	//Match_arrow()
	Mat HSV;
	Scalar l(0, 0, 0);
	Scalar u(179, 255, 158);

	cvtColor(img, HSV, COLOR_BGR2HSV);

	inRange(HSV, l, u, mask);

}

void Improc::get_hist() {
	//Creates a histogram used in stop_go() to find
	//red or green signs
	Mat hsv, hue;                  
	Mat dest;

	//Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

	int hbins = 30, sbins = 32;
	int histSize = MAX(hbins, sbins);
	float hue_range[] = { 0, 180 };
	const float* ranges = { hue_range };
	int ch[] = { 0, 0 };

	cvtColor(img, hsv, COLOR_BGR2HSV);
	hue.create(hsv.size(), hsv.depth());

	mixChannels(&hsv, 1, &hue, 1, ch, 1);
	calcHist(&hue, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false);
	normalize(hist, hist, 0, 255, NORM_MINMAX, -1, Mat());



}

Mat Improc::ret_mask() {
	return mask;
}

double Improc::get_len() {
	return len;
}

double Improc::get_as_r() {
	return as_r;
}

vector<double> Improc::get_t_w_c() {
	return t_w_c;
}

Mat Improc::ret_hist() {
	return hist;
}

Improc::Improc(Mat img) {
	 this-> img=img;
	 shape_mom();
	 get_mask();
	 get_hist();
	 mask = ret_mask();
	 len = get_len();
	 as_r = get_as_r();
	 t_w_c = get_t_w_c();
	 hist = ret_hist();
}
void Improc::operator()(Mat img) {
	
	this->img = img;
	shape_mom();
	get_mask();
	get_hist();
	mask = ret_mask();
	len = get_len();
	as_r = get_as_r();
	t_w_c = get_t_w_c();
	hist = ret_hist();

}
Improc::Improc() {
	

}



