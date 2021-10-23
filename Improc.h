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

#include "Line.h"
#pragma comment(lib, "winmm.lib")


using namespace cv;
using namespace std;

class Improc {

	Mat mask, img, hist;
	
	double len, as_r;
	int t_l, t_h;
	vector<double>t_w_c;

	void shape_mom();
	void get_mask();
	void get_hist();
	
public:
	Improc();
	Improc(Mat img);
	void operator()(Mat F);
	Mat ret_mask();
	Mat ret_hist();
	double get_len();
	double get_as_r();
	vector<double> get_t_w_c();

	//~Improc();

};

