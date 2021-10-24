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

#include"Improc.h"
#include"Lane.h"

#include "timer.h"
#include "Robot.h"
#include"SerialPort.h"
#include"Vision.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;
using namespace chrono;

Vision::Vision(Mat Right_sign, Mat Left_sign, Mat stop_sign, Mat green_sign, VideoCapture cap) {
	//Constructor for Vision class. initalises the signs and obtains
	//the video capture.
	this->Right_sign = Right_sign;
	this->Left_sign = Left_sign;
	this->stop_sign = stop_sign;
	this->green_sign = green_sign;
	this->c = cap;

	R(stop_sign);
	G(green_sign);
	L(Left_sign);
	Ri(Right_sign);
	
	R1.set_values(v1, v2);

}


Vision::~Vision() {

	cout << "\nFrame";

}


void Vision::capt() {

	//captures the video, checks 
	// the camera first.
	if (!c.isOpened()) {
		cerr << "ERROR: Could not open camera" << std::endl;

	}
	start = clock(); //For FPS counter
	c.read(fr);

}

void Vision::disp() {

	end = clock();//For FPS counter
	//displays frame
	fps_counter();
	if (!fr.empty()) {
		imshow("DISPLAY", fr);
	}

	else
		cerr << "Error Empty frame" << endl;

}


void Vision::manual() {

	R1.manual(); //manually control robot

};


void Vision::get_info() {

	R1.get_info();//Receive data from Arduino

};


void Vision::fps_counter() {
	//calculates fps using the start time between when the frame is
	//captured, processed, and displayed.
	//also calculates time taken
	const int num_frames = 1;
	double fpsLive;

	double seconds = abs((double(start) - double(end)) / double(CLOCKS_PER_SEC));
	fpsLive = abs(double(num_frames) / double(seconds));

	cout << "\nSec " << seconds << " FPS " << fpsLive;
	string FPS = to_string(int(fpsLive));
	putText(fr, "FPS: ", { 200,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
	putText(fr, FPS, { 255,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
}

void Vision::info() {
	//Displays the received data from
	//the arduino that is light intensity
	// and the distance received by the ultrasound sensor
	String L, D;
	short int li, di, vi1, vi2;
	li = R1.get_li();
	di = R1.get_dis();
	vi1 = R1.g_v1();
	vi2 = R1.g_v2();

	if (li >= 0 && di >= 0) {
		L = to_string(li);
		D = to_string(di);

		putText(fr, "Light", { 10,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, L, { 65,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, "Dist", { 400,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, D, { 465,15 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, "V1", { 300,25 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, "V2", { 360,25 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, to_string(vi1), { 300,35 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
		putText(fr, to_string(vi2), { 360,35 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);

	}
};

void Vision::shape_det(Mat& img, vector<vector<Point>> contours, vector<Vec4i> hierarchy, double len1, double as_r1, int ar_l, int ar_h, bool& match, vector<double>tet1) {
	//Detects the stop go signs that are hexagonal shaped
	// Matches the shape found with a template loaded by Improc 
	// Uses the perimeter (len1), aspect ratio (as_r1) 
	// an upper and lower limit for the area ar_l, ar_h and
	// the angle made by the vertices of the shape with respect to the center (tet1).
	double peri;
	double H_l, L_l;
	double thet_tol; //tolerance for the angle: +-0.1
	thet_tol = 6;
	const double pi = 3.141592653589;
	double an_tol = 6;
	double thet_match;
	thet_match = 0;

	vector<vector<Point> >hull(contours.size());
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());
	vector<vector<Point>> conPoly(contours.size());
	vector<Rect> boundRect(contours.size());

	vector<Point>vcs;
	vector<double>thet;
	vector<double>t_w_c;
	Point vtx;

	for (size_t i = 0; i < contours.size(); i++)
	{

		Scalar color = Scalar(256, 180, 200);
		convexHull(contours[i], hull[i]);


		peri = arcLength(hull[i], true);

		approxPolyDP(hull[i], conPoly[i], 0.04 * peri, true);
		boundRect[i] = boundingRect(hull[i]);

		mu[i] = moments(hull[i]);
		mc[i] = Point2f(static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)),
			static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)));

		double len = conPoly[i].size();

		double as_r = ((double)boundingRect(hull[i]).width / (double)boundingRect(hull[i]).height);

		H_l = as_r + 0.09;
		L_l = as_r - 0.09;

		int ar = contourArea(hull[i]);
		if ((ar >= ar_l && ar <= ar_h) && len == len1 && (as_r >= L_l && as_r <= H_l)) {

			vcs = conPoly[i];
			drawContours(img, hull, (int)i, color, 2, LINE_8, hierarchy, 0);

			Point c;
			c.x = (mu[i].m10 / (mu[i].m00 + 1e-5));
			c.y = (mu[i].m01 / (mu[i].m00 + 1e-5));

			double x_mom = (mu[i].m10 / (mu[i].m00 + 1e-5));
			double y_mom = (mu[i].m01 / (mu[i].m00 + 1e-5));

			double m_ratio = (x_mom / y_mom); // x/y moments


			circle(img, c, 4, color, -1);
			string Mx = to_string(c.x);
			string My = to_string(c.y);
			string A = to_string(ar);
			String R = to_string(as_r);
			string L = to_string(len);

			putText(img, A, { c.x + 30,c.y + 30 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
			putText(img, My, { c.x + 55,c.y + 10 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 2);
			putText(img, Mx, { c.x + 10,c.y + 10 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 2);
			putText(img, R, { c.x + 100,c.y + 10 }, FONT_HERSHEY_PLAIN, 1, Scalar(50, 10, 255), 2);
			putText(img, L, { c.x + 30,c.y + 50 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);

			for (int i = 0; i < vcs.size(); i++) {
				vtx = vcs[i];
				double the = atan(vtx.y / (vtx.x + (1e-10))) * (180 / pi);
				double t_c; //theta wrt to center for each point
				t_c = atan((abs(vtx.y - c.y)) / (abs(vtx.x - c.x) + 1e-15)) * (180 / pi);
				double t_diff;
				thet.push_back(the);
				t_w_c.push_back(t_c);
				t_diff = abs(t_w_c[i] - thet[i]);
				string N = to_string(i);
				string T = to_string(the);
				string T_c = to_string(t_diff);
				putText(img, N, { vtx.x + 10,vtx.y + 10 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
				putText(img, T, { vtx.x + 20,vtx.y + 20 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);
				putText(img, T_c, { vtx.x + 35,vtx.y + 35 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 2);

			}

			vector_match(t_w_c, tet1, thet_match, an_tol);
			string M = to_string(thet_match);
			putText(img, M, { c.x + 90,c.y + 90 }, FONT_HERSHEY_PLAIN, 1, Scalar(50, 150, 0), 2);

			if (thet_match >= 5)
			{
				match = true;
			}

		}
	}

}

double Vision::compare_hu(Mat& img, Mat& img2, double& match, double& Ma) {
	//compares the Hu moments of the img with img2
	//There are seven Hu moments for each shape
	

	Moments M = moments(img, false);
	double hM[7];
	HuMoments(M, hM);
	for (int i = 0; i < 7; i++)
	{
		hM[i] = -1 * copysign(1.0, hM[i]) * log10(abs(hM[i]));//The functions calculates the moments for each shape 
	}

	Moments M2 = moments(img2, false);
	double hM2[7];
	HuMoments(M2, hM2);
	for (int i = 0; i < 7; i++)
	{
		hM2[i] = -1 * copysign(1.0, hM2[i]) * log10(abs(hM2[i]));
	}
	for (int i = 0; i < 7; i++) {

		double d = abs(hM[i] - hM2[i]);

		if (d < 1) {
			match++; //if the absolute difference between the moments of the 
		}			 //shapes ae less than 1, it is counted as a match

	}
	Ma = match / 7; //number of matches out of the 7 moments


	if ((hM[6] < 0 && hM2[6]>0) || (hM[6] > 0 && hM2[6] < 0)) {

		Ma = 0;

	}

	return Ma; //returns the match found

}


void Vision::get_back_proj(Mat& color, Mat& color2, Mat& imgtresh, Mat& imgtresh2, int t_l, int t_h, int t_l2, int t_h2) {
	//Uses a provided histogram (color) and finds the 
	// tresholded image (imgtresh) based on the provided upper/lower limits
	// finds the tresholded images for both colored signs
	int hbins = 30, sbins = 32;
	int histSize = MAX(hbins, sbins);
	float hue_range[] = { 0, 180 };
	const float* ranges = { hue_range };
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5)); //size change to variable or not, check

	int channels[] = { 0,1 };
	float sranges[] = { 0,256 };

	Mat hsvfr, huefr, dest, dest2, imdia, imgblur, hsv, hsv2, mask, mask2, roi;
	MatND histfr;


	cvtColor(fr, hsvfr, COLOR_BGR2HSV); //change to imgblur 

	huefr.create(hsvfr.size(), hsvfr.depth());
	int ch[] = { 0, 0 };
	mixChannels(&hsvfr, 1, &huefr, 1, ch, 1);

	//Skipped the calchist and normalize funcs.

	calcBackProject(&huefr, 1, 0, color, dest, &ranges, 1, true); // calculates the back projection based on the 
	calcBackProject(&huefr, 1, 0, color2, dest2, &ranges, 1, true); // provided histogram

	threshold(dest, imgtresh, t_l, t_h, THRESH_BINARY); 
	threshold(dest2, imgtresh2, t_l2, t_h2, THRESH_BINARY);

	dilate(imgtresh, imgtresh, kernel);
	dilate(imgtresh2, imgtresh2, kernel);

}

void Vision::Get_back_proj(Mat& color, Mat& imgtresh, int t_l, int t_h) {
	//same as get_back_proj, except only finds for one histogram

	int hbins = 30, sbins = 32;
	int histSize = MAX(hbins, sbins);
	float hue_range[] = { 0, 180 };
	const float* ranges = { hue_range };
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5)); //size change to variable or not, check

	int channels[] = { 0,1 };
	float sranges[] = { 0,256 };

	Mat hsvfr, huefr, dest, imdia, imgblur, hsv, hsv2, mask, mask2, roi;
	MatND histfr;

	cvtColor(fr, hsvfr, COLOR_BGR2HSV); //change to imgblur 

	huefr.create(hsvfr.size(), hsvfr.depth());
	int ch[] = { 0, 0 };
	mixChannels(&hsvfr, 1, &huefr, 1, ch, 1);

	calcHist(&hsvfr, 1, channels, Mat(), histfr, 1, &histSize, &ranges, true, false); 
	normalize(histfr, histfr, 0, 255, NORM_MINMAX, -1, Mat());

	calcBackProject(&huefr, 1, 0, color, dest, &ranges, 1, true); //the color needs to be argument (Mat)
	

	threshold(dest, imgtresh, t_l, t_h, THRESH_BINARY); //needs to be an argument value

	dilate(imgtresh, imgtresh, kernel); 

}

void Vision::Match_arrow(double& as_rlR, double& as_rhR, int& arlR, bool& foundR, double& m_rR, double& as_rlL, double& as_rhL, int& arlL, bool& foundL, double& m_rL) {
	////Overloaded version finds both laft and right
	//Function finds an arrow sign using the compare_hu function.
	//It uses a provided aspect ratio range and area and invesigates
	//regions of interest that meets the dimensional requirements specified
	Mat HSV, mask, roi, HSV2, maskR, maskL;

	cvtColor(fr, HSV, COLOR_BGR2HSV);

	//cvtColor(arrow, HSV2, COLOR_BGR2HSV);
	maskR = Ri.ret_mask();  //the template of the right/left arrows from Improc, Match_arrow 
	maskL = L.ret_mask();   //calculates the Hu moments of these templates and matches them with 
							//the shapes found
	Scalar lower(0, 0, 0);
	Scalar upper(179, 255, 100); //120
	Scalar lower1(0, 0, 0);
	Scalar upper1(179, 255, 158);

	//inRange(HSV2, lower1, upper1, mask2);

	inRange(HSV, lower, upper, mask);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<vector<Point>> conPoly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Rect> boundRect2(contours.size());
	vector<vector<Point>> convexHulls(contours.size());
	vector<vector<Point> >hull(contours.size());

	for (int i = 0; i < contours.size(); i++) {  

		convexHull(contours[i], hull[i]);
		approxPolyDP(hull[i], conPoly[i], 1, true);
		int ar = contourArea(hull[i]);

		if (ar > 1500 && ar < 25000) { 
			boundRect[i] = boundingRect(hull[i]);
			double as_r = ((double)boundingRect(hull[i]).width / (double)boundingRect(hull[i]).height); //AR= W/H
			roi = mask(boundingRect(hull[i]));
			double MatchR = 0;
			double MR = 0;
			compare_hu(roi, maskR, MatchR, MR);  //calls the compare_hu with the arrow template
			double MatchL = 0;                   //from improc
			double ML = 0;
			compare_hu(roi, maskL, MatchL, ML);
			//imshow("roi", mask);
			if ((ar >= arlR) && ((as_r <= as_rhR) && (as_r >= as_rlR)) && (MR == m_rR)) { //requirments for the area,
				string a = to_string(ar);												  // aspect ratio, and the match required 
				string ma = to_string(MR);												  // (MR)
				string aR = to_string(as_r);
				putText(fr, a, { boundRect[i].x,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 2);
				putText(fr, aR, { boundRect[i].x + 75,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				putText(fr, ma, { boundRect[i].x + 75,boundRect[i].y + 80 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				rectangle(fr, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 0), 2);
				foundR = true;        //Its a positive match, arrow found
				break;
			}
			if ((ar >= arlL) && ((as_r <= as_rhL) && (as_r >= as_rlL)) && (ML == m_rL)) {//requirments for the area,
				string a = to_string(ar);												 // aspect ratio, and the match required 
				string ma = to_string(ML);												 // (ML)
				string aR = to_string(as_r);
				putText(fr, a, { boundRect[i].x,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 2);
				putText(fr, aR, { boundRect[i].x + 75,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				putText(fr, ma, { boundRect[i].x + 75,boundRect[i].y + 80 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				rectangle(fr, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 0), 2);
				foundL = true;         //Its a positive match, arrow found
				break;
			}
		}
	}

}


bool Vision::Match_arrow(Mat& fr, Mat& arrow, double& as_rl, double& as_rh, int& arl, bool& found, double& m_r) {
	//Function finds an arrow sign using the compare_hu function.
	//It uses a provided aspect ratio range and area and invesigates
	//regions of interest that meetes the dimensional requirements specified
	//Finds matches for one arrow.
	Mat HSV, mask, roi, HSV2, mask2;
	double l, h;
	l = as_rl;
	h = as_rh;
	cvtColor(fr, HSV, COLOR_BGR2HSV);

	cvtColor(arrow, HSV2, COLOR_BGR2HSV);

	Scalar lower(0, 0, 0);
	Scalar upper(179, 255, 100); //120
	Scalar lower1(0, 0, 0);
	Scalar upper1(179, 255, 158);

	inRange(HSV2, lower1, upper1, mask2);

	inRange(HSV, lower, upper, mask);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	vector<vector<Point>> conPoly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Rect> boundRect2(contours.size());
	vector<vector<Point>> convexHulls(contours.size());
	vector<vector<Point> >hull(contours.size());
	//imshow("masked",mask);
	//bool fin = false;
	//double time = 1.2;
	for (int i = 0; i < contours.size(); i++) {

		convexHull(contours[i], hull[i]);
		approxPolyDP(hull[i], conPoly[i], 1, true);
		int ar = contourArea(hull[i]);

		if (ar > 1500 && ar < 25000) {
			boundRect[i] = boundingRect(hull[i]);
			double as_r = ((double)boundingRect(hull[i]).width / (double)boundingRect(hull[i]).height); //AR= W/H
			roi = mask(boundingRect(hull[i]));

			double Match = 0;
			double M = 0;

			compare_hu(roi, mask2, Match, M);
			//imshow("roi", mask);
			if ((ar >= arl) && ((as_r <= as_rh) && (as_r >= as_rl)) && (M == m_r)) {
				string a = to_string(ar);
				string ma = to_string(M);
				string aR = to_string(as_r);
				putText(fr, a, { boundRect[i].x,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 2);
				putText(fr, aR, { boundRect[i].x + 75,boundRect[i].y }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				putText(fr, ma, { boundRect[i].x + 75,boundRect[i].y + 80 }, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				rectangle(fr, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 0, 0), 2);
				found = true;
				break;

			}
		}
	}
	return found;
}

double Vision::vector_match(vector<double>v1, vector<double>v2, double& m_v, double tol) {
	//iterates through the first vectors to determine if the values 
	// are within a provided tolerance to the second vector.
	for (int i = 0; i < v1.size(); i++) {
		if ((v1[i] <= v2[i] + tol) && (v1[i] >= v2[i] - tol)) {
			m_v++;
		}
	}
	return m_v; //return the match


}

void Vision::stop_go() {
	//Function finds the red/green signs based on a given histogram, shape perimeter
	//aspect ratio, and the angles made by the vertices of the shape with respect to the centre of the shape.
	// Uses these properties to find the signs

	//Green first
	double len_tempG, as_r_tempG;
	int ar_lG, ar_hG;
	ar_lG = 14000;
	ar_hG = 16500;
	bool matchG;
	matchG = false;
	Mat imgtreshG;
	int t_lG, t_hG;
	t_lG = 0;
	t_hG = 255;
	vector<double>tethG;
	vector<vector<Point>> contoursG;
	vector<Vec4i> hierarchyG;

	Mat histG = G.ret_hist();
	len_tempG = G.get_len();
	as_r_tempG = G.get_as_r();
	tethG = G.get_t_w_c();

	//Red now
	double len_tempR, as_r_tempR;
	int ar_lR, ar_hR;
	ar_lR = 14000;
	ar_hR = 16500; //58cm away
	bool matchR;
	matchR = false;
	Mat imgtreshR;
	int t_lR, t_hR;
	t_lR = 35;
	t_hR = 255;
	vector<double>tethR;
	vector<vector<Point>> contoursR;
	vector<Vec4i> hierarchyR;

	Mat histR = R.ret_hist();
	len_tempR = R.get_len();
	as_r_tempR = R.get_as_r();
	tethR = R.get_t_w_c();

	get_back_proj(histG, histR, imgtreshG, imgtreshR, t_lG, t_hG, t_lR, t_hR);   

	findContours(imgtreshG, contoursG, hierarchyG, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //finds the contours of the Treshold image found using 
	findContours(imgtreshR, contoursR, hierarchyR, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //the respective histograms

	shape_det(fr, contoursG, hierarchyG, len_tempG, as_r_tempG, ar_lG, ar_hG, matchG, tethG); //uses shape_detect to find the hexagonal shape 
	shape_det(fr, contoursR, hierarchyR, len_tempR, as_r_tempR, ar_lR, ar_hR, matchR, tethR); //based on the geometric properties specified.


	if (matchG == true) {  //GO sign found

		putText(fr, "GO", { 250,100 }, FONT_HERSHEY_PLAIN, 9, Scalar(0, 255, 0), 4);
		R1.robot_go();

	}

	if (matchR == true) { //STOP sign found

		putText(fr, "STOP", { 150,100 }, FONT_HERSHEY_PLAIN, 9, Scalar(0, 0, 255), 4);
		R1.robot_stop();
		stop_seen = true;
	}
	else {
		stop_seen = false;
	}



}


void Vision::Red(int& tresh_h_red, int& tresh_l_red) {
	//Similar to stop_go, but only finds 
	//the stop sign
	double len_temp, as_r_temp;
	int ar_l, ar_h;
	ar_l = 14000;
	ar_h = 16500; //58cm away
	bool match;
	match = false;
	Mat imgtresh;
	int t_l, t_h;
	t_l = 35;
	t_h = 255;
	vector<double>teth;
	Mat hist = R.ret_hist();

	len_temp = R.get_len();
	as_r_temp = R.get_as_r();
	teth = R.get_t_w_c();


	Get_back_proj(hist, imgtresh, t_l, t_h);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(imgtresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


	shape_det(fr, contours, hierarchy, len_temp, as_r_temp, ar_l, ar_h, match, teth); //uses the overloaded version of shape_det

	if (match == true) {

		putText(fr, "STOP", { 150,100 }, FONT_HERSHEY_PLAIN, 9, Scalar(0, 0, 255), 4);
		//robot_stop(ve1, ve2);

	}


}

void Vision::Green(int& tresh_h_green, int& tresh_l_green) {
	//Similar to stop_go, but only finds 
	//the go sign
	double len_temp, as_r_temp;
	int ar_l, ar_h;
	ar_l = 14000;
	ar_h = 16500;
	bool match;
	match = false;
	Mat imgtresh;
	int t_l, t_h;
	t_l = 0;
	t_h = 255;
	vector<double>teth;
	Mat hist = G.ret_hist();

	Get_back_proj(hist, imgtresh, t_l, t_h);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	len_temp = G.get_len();
	as_r_temp = G.get_as_r();
	teth = G.get_t_w_c();

	findContours(imgtresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	shape_det(fr, contours, hierarchy, len_temp, as_r_temp, ar_l, ar_h, match, teth);

	if (match == true) {

		putText(fr, "GO", { 250,100 }, FONT_HERSHEY_PLAIN, 9, Scalar(0, 255, 0), 4);
		//robot_go(ve1, ve2);
	}


}
void Vision::Road(int& hmin, int& hmax, int& smin, int& smax, int& vmin, int& vmax) {
	//Uses the lane class to find the alne lines and determine the response of the robot
	//More sensitive to changes in the HSV range, so this functions requires that these
	//values be manually controlled.
	namedWindow("Trackbars", WINDOW_AUTOSIZE);
	createTrackbar("Hue Min", "Trackbars", &hmin, 179);
	createTrackbar("Hue Max", "Trackbars", &hmax, 179);
	createTrackbar("Sat Min", "Trackbars", &smin, 255);
	createTrackbar("Sat Max", "Trackbars", &smax, 255);
	createTrackbar("Val Min", "Trackbars", &vmin, 255);
	createTrackbar("Val Max", "Trackbars", &vmax, 255);

	//hmin 0, hmax 179, sat min 0,sat max 255,val min 0, val max 70;

	Scalar lower(hmin, smin, vmin);
	Scalar upper(hmax, smax, vmax);

	vector<short int> vals;

	//Lane L(imgCanny,fr); //CONSTRUCTOR 1
	Lane L(fr); //CONSTRUCTOR 2

	//L.ref_line();  ///Use constructor 1 to use this
	//L.find_R_lane();
	//L.find_L_lane();
	//L.det_int();

	L.ref_line();
	L.virtual_road();
	L.road_can(lower, upper);
	L.find_R_lane_warped();
	L.find_L_lane_warped();
	L.det_warped_ang();
	//L.drw_ref_line();



	vals = L.spd_values();

	double t_tl, t_tr; //turning rate (sec/degree) left and right, established experimentally 
	t_tl = 0.0177611;  //by determining the average time taken for the robot to turn 90 degrees 
	t_tr = 0.0210633;  //then finding the time taken to turn one degree.	
	short int v1 = vals[0];
	short int v2 = vals[1];

	//the robot steers by powering one servo while the other remains statoinary.
	//It can turn a specified angle by steering for a specified time that is obtained 
	// by multipliying the angle it needs to turn by the turning rate in each direction

	if (vals.size() > 1 && stop_seen == false) {


		if (v1 == 180 && v2 == 0) {
			R1.robot_go();
		}


		if (v1 == 85 && v2 == 0) { 
								  
			short int th = vals[2];
			double thet = double(th);
			double th_t = (thet / 1000); //The lane function determines the deviation angle of the robot (th_t)
			double time = th_t * t_tl;
			putText(fr, to_string(time), Point(300, 400), FONT_HERSHEY_PLAIN, 1, Scalar(25, 75, 255), 1);
			R1.robot_st_left(time);  // The function then steers according to the deviation angle to correct itslef
									 // it steers the robot for the specified time to turn according to the deviation angle
		}

		if (v1 == 180 && v2 == 85) { //Same as above but to steer right.
			short int th = vals[2];
			double thet = double(th);
			double th_t = (thet / 1000);
			double time = th_t * t_tr;
			putText(fr, to_string(time), Point(300, 400), FONT_HERSHEY_PLAIN, 1, Scalar(25, 75, 255), 1);
			R1.robot_st_right(time);
		}

	}

}

void Vision::left_right() {
	// Function that detects an arrow sign and steers the robot by 90 degrees left or right.
	//Utilizes the Match_arrow function along with certain dimensional properties to recognize the sign
	//Right
	double as_rlR, as_rhR;
	int arlR;
	bool foundR = false;
	bool finR = false;
	double timeR = 0.7581;
	as_rlR = 0.4;   //Minimum aspect ratio
	as_rhR = 0.6;   //Maximum aspect ratio
	arlR = 6600;  //The area of the shape (about 50 cm (11500) away)
	double m_rR = 1.0; //Match needed
	static int r_times = 0; //number of times the robot has turned
	static int l_times = 0;
	//Left
	double as_rlL, as_rhL;
	int arlL;
	bool foundL = false;
	bool finL = false;
	double timeL = 0.70125;
	as_rlL = 0.4; //Minimum aspect ratio
	as_rhL = 0.6;  //Maximum aspect ratio
	arlL = 19500; //The area of the shape (abt 44 cm (12500) away)
	double m_rL = 1.0; //Match needed

	Match_arrow(as_rlR, as_rhR, arlR, foundR, m_rR, as_rlL, as_rhL, arlL, foundL, m_rL);


	if (foundR == true && r_times < 1) {
		foundR = false;
		putText(fr, "TURN RIGHT", { 300,250 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
		R1.turn_right(timeR);
		r_times++;
	}

	if (foundL == true && l_times < 1) {
		putText(fr, "TURN LEFT", { 300,250 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
		R1.turn_left(timeL);
		foundL = false;
		l_times++;
	}


}

void Vision::Right(int& match_method, int& treshM_l, int& treshM_H) {
	//This function finds the right sign only
	//the robots turns by 90 degrees by spining one servo in the opposite direction to the other
	double as_rl, as_rh;
	int arl;
	bool found = false;
	bool fin = false;
	double time = 2.0;
	as_rl = 0.4;
	as_rh = 0.6;
	arl = 11500;  //about 49 cm away
	double m_r = 1.0; //Match needed
	double timeR = 0.7581; //time needed to turn 90 to the right
	Match_arrow(fr, Right_sign, as_rl, as_rh, arl, found, m_r);

	if (found == true) {
		putText(fr, "TURN RIGHT", { 300,250 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
		R1.turn_right(timeR);
		found = false;

	}

}
void Vision::Left(int& treshM_l, int& treshM_H) {
	//This function finds the left sign only
	double as_rl, as_rh;
	int arl;
	bool found = false;
	bool fin = false;
	double time = 1.4;
	as_rl = 0.4;
	as_rh = 0.6;
	arl = 12500; //abt 51 cm away
	double m_r = 1.0; //Match needed
	double timeL = 0.70125; //time needed to turn 90 to the left
	Match_arrow(fr, Left_sign, as_rl, as_rh, arl, found, m_r);

	if (found == true) {
		putText(fr, "TURN LEFT", { 300,250 }, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
		R1.turn_left(timeL);
		found = false;

	}

}