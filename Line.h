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


#pragma comment(lib, "winmm.lib")


using namespace cv;
using namespace std;
//This class uses either a Point or a Vec4i and finds line
//propeties such as gradient, y-intercept as well as mid point and
//distance
//the correction factor is used to draw the line, since the detect lanes function isolates
//a quater of the screen. the correction factor translates the line to its correct position
//Its mostly used in the Lane class
class Line {
	int x1, y1, x2, y2, m, c, mdx, mdy, yc, xc; //top and bottom cordinates, gradient, y intercept,mid x&y, y val for x arg, x val y arg
	int x_c, y_c;
	double m_d, c_d;
public:
	Line();
	Line(Vec4i Cor);
	Line(Point t, Point b);
	void set_cords(Vec4i& Cor);
	void cor_val();
	void cor_val(int x, int y); //the correction value, when drawing on the screen
	void operator()(Vec4i& Cor);
	void operator()(Point t, Point b);
	void grad_m();
	void grad_d();
	void mid_point();
	void c_int(); //c int, c double
	void c_dub();
	void c_int_cor(); //c int, c double corrected to draw on screen
	void c_dub_cor();
	double y_val(int x); //y for each 
	double x_val(int y);
	Point p1();//returns point x1,y1
	Point p2();
	Point p1(int cx, int cy);//returns point x1,y1 the arguments are the coorection, when drawing on the screen
	Point p2(int cx, int cy);//returns point x2,y2
	Point mid();
	Point mid(int cx, int cy);
	int dist();
	int x_cor_1();     //return x y cords, corrected or no
	int x_cor_1_c();
	int y_cor_1();
	int y_cor_1_c();
	int x_cor_2();     //return x y cords, corrected or no
	int x_cor_2_c();
	int y_cor_2();
	int y_cor_2_c();
	int get_m();
	double get_md();
	int get_c();
	double get_cd();
	int get_c_cor();
	double get_cd_cor();
	int line_dist();

};

