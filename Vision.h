#pragma once
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

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )


#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;
using namespace chrono;
//The Vision class detects the stop/go and left/right signs, lanes and sends commands to the robot using the Robot class.
//It detects signs using preloaded images of the signs as templates. The Improc class is utilize to derieve properties 
//from the templates that the Vision class uses to identify the signs. It uses the Lane class to identify the lanes and
//for steering directions.

class Vision {
	short int v1, v2;
public:
	Mat fr,Right_sign, Left_sign, stop_sign, green_sign;
	int li;
	short unsigned int d;

	Improc R, G, L, Ri;
	VideoCapture c;
	clock_t start, end;
	Robot R1;

	bool stop_seen;

	Vision(Mat Right_sign, Mat Left_sign, Mat stop_sign, Mat green_sign, VideoCapture cap);

	//Vision Tools
	void fps_counter();

	//Helper functions
	void shape_det(Mat& img, vector<vector<Point>> contours, vector<Vec4i> hierarchy, double len1, double as_r1, int ar_l, int ar_h, bool& match, vector<double>tet1);
	void Get_back_proj(Mat& color, Mat& imgtresh, int t_l, int t_h);
	void get_back_proj(Mat& color, Mat& color2, Mat& imgtresh, Mat& imgtresh2, int t_l, int t_h, int t_l2, int t_h2);
	bool Match_arrow(Mat& fr, Mat& arrow, double& as_rl, double& as_rh, int& arl, bool& found, double& m_r);
	void Match_arrow(double& as_rlR, double& as_rhR, int& arlR, bool& foundR, double& m_rR, double& as_rlL, double& as_rhL, int& arlL, bool& foundL, double& m_rL);
	double vector_match(vector<double>v1, vector<double>v2, double& m_v, double tol);
	double compare_hu(Mat& img, Mat& img2, double& match, double& Ma);

	//Vision funcitions
	void capt();
	void disp();
	void info();
	void Red(int& tresh_h_red, int& tresh_l_red); //Stop robot, Red light. 
	void Green(int& tresh_h_green, int& tresh_l_green); //Robot go, Green light.
	void Road(int& hmin, int& hmax, int& smin, int& smax, int& vmin, int& vmax);
	void Right(int& match_method, int& treshM_l, int& treshM_H); // Turn right at sign
	void Left(int& treshM_l, int& treshM_H); //Turn left at sign
	void stop_go();
	void left_right();


	//Robot Functions
	void manual();
	void get_info();


	~Vision();

};
