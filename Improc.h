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
//This class provides preprocessing for the stop/go, arrow sign templates 
//that the vision class uses to detect signs. It uses Canny edge detection
//along with a treshold operation to detect the shape and obtain properties
//such as the perimeter of the shape, aspect ratio and angles made by its vetices with respect to
//the center. It reterives an image that isolates the black arrow sign as a template for arrow detection
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

