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

using namespace cv;
using namespace std;

void Line::cor_val() {

	x_c = 0;
	y_c = 0;

}

Line::Line() {
	cor_val();
}


void Line::cor_val(int x, int y) {

	x_c = x;
	y_c = y;

}

void Line::grad_m() {

	int nume = (y2 - y1);
	int deno = ((x2 - x1) + 1e-7);

	m = nume / ((deno)+1e-7);


}

void Line::grad_d() {

	double nume = (y2 - y1);
	double deno = ((x2 - x1) + 1e-7);

	m_d = nume / ((deno)+1e-7);


}

void Line::mid_point() {

	mdx = ((x1 + x2) / 2);
	mdy = ((y1 + y2) / 2);

}

void Line::c_int() {

	c = y2 - (m * x2);

}

void Line::c_dub() {

	c_d = y2 - (m_d * x2);

}

void Line::c_int_cor() {

	c = (y2 + y_c) - (m * (x2 + x_c));

}

void Line::c_dub_cor() {
	double a = y2 + y_c;
	double b = x2 + x_c;

	c_d = (a)-(m_d * (b));

}

Line::Line(Vec4i Cor) {

	x1 = Cor[0];
	y1 = Cor[1];
	x2 = Cor[2];
	y2 = Cor[3];

	grad_m();
	grad_d();
	mid_point();
	c_int(); //c int, c double
	c_dub();

}

Line::Line(Point t, Point b) {

	x1 = t.x;
	y1 = t.y;
	x2 = b.x;
	y2 = b.y;

	grad_m();
	grad_d();
	mid_point();
	c_int(); //c int, c double
	c_dub();

}

void Line::set_cords(Vec4i& Cor) {
	x1 = Cor[0];
	y1 = Cor[1];
	x2 = Cor[2];
	y2 = Cor[3];

	grad_m();
	grad_d();
	mid_point();
	c_int(); //c int, c double
	c_dub();

}

void Line:: operator()(Vec4i& Cor) {

	x1 = Cor[0];
	y1 = Cor[1];
	x2 = Cor[2];
	y2 = Cor[3];

	//set_cords(Cor);

	grad_m();
	grad_d();
	mid_point();
	c_int(); //c int, c double
	c_dub();

};

void Line:: operator()(Point t, Point b) {

	x1 = t.x;
	y1 = t.y;
	x2 = b.x;
	y2 = b.y;

	grad_m();
	grad_d();
	mid_point();
	c_int(); //c int, c double
	c_dub();

};

double Line::y_val(int x) {

	c_int_cor();

	double yc;

	yc = (m * x) + c;

	return yc;

}

double Line::x_val(int y) {

	//c_int_cor();
	//c_dub_cor();

	double xc;

	xc = (y / (m_d + 1e-6)) - (c_d / (m_d + 1e-6));

	return xc;

}

Point Line::p1() {
	//removed the x_c,y_c 
	//11/10/2021
	Point l1;
	l1.x = x1;
	l1.y = y1;

	return l1;

}

Point Line::p2() {

	Point l2;
	l2.x = x2;
	l2.y = y2;

	return l2;

}

Point Line::p1(int cx, int cy) {

	Point l1;
	l1.x = x1 + cx;
	l1.y = y1 + cy;

	return l1;

}

Point Line::p2(int cx, int cy) {

	Point l2;
	l2.x = x2 + cx;
	l2.y = y2 + cy;

	return l2;

}

Point Line::mid(int cx, int cy) {

	Point m;
	m.x = mdx + cx;
	m.y = mdy + cy;

	return m;
}

Point Line::mid() {

	Point m;
	m.x = mdx;
	m.y = mdy;

	return m;
}

int Line::get_m() {

	return m;

}

int Line::dist() {
	int dist;

	dist = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));

	return dist;
}

int Line::x_cor_1() {

	return x1;
}

int Line::x_cor_1_c() {

	return (x1 + x_c);
}

int Line::y_cor_1() {

	return y1;
}

int Line::y_cor_1_c() {

	return (y1 + y_c);
}

int Line::x_cor_2() {

	return x2;
}

int Line::x_cor_2_c() {

	return (x2 + x_c);
}

int Line::y_cor_2() {

	return y2;
}

int Line::y_cor_2_c() {

	return (y2 + y_c);
}

double Line::get_md() {

	return m_d;

}

int Line::get_c() {

	return c;

}

double Line::get_cd() {

	return c_d;

}
int Line::get_c_cor() {
	c_int_cor();
	return c;

}

double Line::get_cd_cor() {
	c_dub_cor();
	return c_d;

}

int Line::line_dist() {


	int di = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));

	return di;
}