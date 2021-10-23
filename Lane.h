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

class Lane {
	Mat can, fr;
	Mat road, can_road;
	Mat cr;

	int x, y, width, height;
	Point tp, bt, lt, lb;//points for the top and bottom 

	Point tp_adj, bt_adj;
	Point tp_adj_r, bt_adj_r; //top and bottom of cropped img //ref line adjusted for drawing the line for warped perspect

	vector<Line>left_L;
	vector<Line>right_R;

	Line R_i, R_o, L_i, L_o;

	short int v1, v2, th;

public:
	Lane(Mat imCan, Mat F);
	Lane(Mat F);

	void ref_line();

	int grad(Point t1, Point t2, double& mg);
	double grad(Vec4i l);
	void get_eqn(Point t1, Point t2, double& m, int& c); //gets c and m
	int vert_point(double m, int c, int x); //returns the y (vertical) value for specified y=mx+c eqn

	int point_distance(Point t1, Point t2);
	int point_distance(Vec4i l1);
	Point mid_point(Point l1, Point l2);
	double ang_twolines(Line l1, Line l2); //returns the angle between two lines
	double ang_w_yaxis(Line l);
	double left_ang_actual(double ang); // returns the corrected real world left angle (8th order poly regression)
	double right_ang_actual(double ang); // returns the corrected real world right angle (8th order poly regression)
	bool m_diff(Line l1, Line l2, double d);
	bool m_diff_tol(Line l1, Line l2, double d);
	bool m_diff(Line l1, Line l2, double d, double& md);
	bool similar_line(Vec4i l1, Vec4i l2, int d_req);
	double dist_point_line(Point p, Vec4i l);
	double dist_point_line(Point p, int cor, bool vert);

	vector<Line>lane_lines(vector<Vec4i>& lines, double m_tol, int th_l, int th_h);
	vector<Line>lane_lines(vector<Vec4i>& lines);
	vector<Line>line_aggr(vector<Line>lines, double d); // adds up lines 
	vector<Line>line_aggr(vector<Line>lines, double d, double tol); //overloaded version, check min parralel dist and m in that order
	vector<Line>line_aggr(vector<Line>lines, vector<Line>lines2, double d);
	vector<Line>line_aggr(vector<Line>lines, vector<Line>lines2, double d, double tol);

	bool min_parrallel_dist(Line l1, Line l2, double d); //returns true if the dist between the lines is greater than the specified std m tol=0.05
	bool min_parrallel_dist(Line l1, Line l2, double d, double tol); //over loaded ver. of above func w tol for the m
	Point poly_ctr(Vec4i& ln1, Vec4i& ln2); //takes four points int the form of Vec4i, gets points of the moments, makes sure dist between lines is >0
	vector<Line>det_in_out(Line l1, Line l2, Point cntrd, int xc, int yc); //determines the inner and outter lanes


	double parralel_dist_lines(Line l1, Line l2);
	vector<Line>parrallel_lines(vector<Vec4i>& lines, double dis_min); //return a vector of parrallel lines
	Point lane_poly_ctr(Line l1, Line l2, int xc, int yc);

	void right_dist_check(int d1, int d2, int dm, double ang);
	void left_dist_check(int d1, int d2, int dm, double ang);
	void left_dist_check(int d1, int dm, double ang);
	void right_dist_check(int d1, int dm, double ang);
	void virtual_road();
	void drw_ref_line();
	void virtual_road(Line l1, Line l2);

	void road_can(Scalar lower, Scalar upper);
	void find_R_lane_warped();
	void find_L_lane_warped();

	double get_bigger_angle(double a1, double a2);
	void det_warped_ang();
	vector<short int> spd_values();


	//~Lane();

};
