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
#include "Lane.h"
#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;

Lane::Lane(Mat imCan, Mat F) {
	can = imCan;
	fr = F;

}

Lane::Lane(Mat F) {

	fr = F;
}

int Lane::point_distance(Point t1, Point t2) {
	//Finds the distance between two points
	int d;

	d = sqrt(((t2.x - t1.x) * (t2.x - t1.x)) + ((t2.y - t1.y) * (t2.y - t1.y)));

	return abs(d);

}

int Lane::point_distance(Vec4i l1) {
	//Finds the distance of the provided Vec4i
	int d;
	Point t1, t2;
	t1.x = l1[0];
	t1.y = l1[0];
	t2.x = l1[0];
	t2.y = l1[0];

	d = sqrt(((t2.x - t1.x) * (t2.x - t1.x)) + ((t2.y - t1.y) * (t2.y - t1.y)));


	return abs(d);

}

Point Lane::mid_point(Point l1, Point l2) {
	//returns mid point of the line created by the 
	// provided two points
	int x1 = l1.x;
	int x2 = l2.x;

	int y1 = l1.y;
	int y2 = l2.y;

	int m_x1 = ((x1 + x2) / 2);
	int m_y1 = ((y1 + y2) / 2);

	Point mid;
	mid.x = m_x1;
	mid.y = m_y1;

	return mid;

}

void Lane::ref_line() {
	//Creates the reference line
	Rect roi;
	roi.x = 0;
	roi.y = (fr.size().height) * (0.75);
	roi.width = fr.size().width;
	roi.height = (fr.size().height) * (0.25);


	cr = fr(roi); //Change this to can if using the 1st constructor
	//Point tp, bt; //top and bottom of cropped img //ref line
	tp.x = (cr.size().width) / 2;
	tp.y = cr.size().height;
	bt.x = (cr.size().width) / 2;
	bt.y = 1;


	tp_adj.x = (cr.size().width) / 2;
	tp_adj.y = cr.size().height + (cr.size().height * 3);
	bt_adj.x = (cr.size().width) / 2;
	bt_adj.y = 1 + (cr.size().height * 3);


}

int Lane::grad(Point t1, Point t2, double& mg) {//bt,tp
	//returns the gradient of the line created by the two points
	//also provides the gradient as a double for accuaracy
	int m;
	int nume = (t2.y - t1.y);
	int deno = ((t2.x - t1.x) + 1e-7);

	mg = nume / (deno + 1e-7);

	m = nume / ((deno)+1e-7);


	return m;

}

double Lane::grad(Vec4i l) {
	//Finds the gradient of a line 
	//created by the Vec4i
	int x1, x2, y1, y2;

	double m;

	x1 = l[0];
	y1 = l[1];
	x2 = l[2];
	y2 = l[3];

	int nume = (y2 - y1);
	int deno = ((x2 - x1) + 1e-7);

	m = nume / ((deno)+1e-7);

	return m;

}

void Lane::get_eqn(Point t1, Point t2, double& m, int& c) {
	//finds gradient and y intercept of the line
	//described by two points
	int nume = (t2.y - t1.y);
	int deno = ((t2.x - t1.x) + 1e-7);

	m = nume / ((deno)+1e-7);

	c = t2.y - (m * t2.x);

}

bool Lane::m_diff(Line l1, Line l2, double d) {

	//returns true if the difference between the m of the two lines is at least the specified value

	bool diff;
	diff = false;
	double m1 = l1.get_md();
	double m2 = l2.get_md();

	double m_dif = abs(m1 - m2);

	if (m_dif >= d) {
		diff = true;
	}

	return diff;
}

bool Lane::m_diff_tol(Line l1, Line l2, double d) {

	//returns true if the difference between the m of the two lines is within the tolerance
	//specified

	bool diff;
	diff = false;
	double m1 = abs(l1.get_md());
	double m2 = abs(l2.get_md());

	double m_dif = m1 - m2;
	//cout << "\nm diff " << m_dif;
	if (m_dif <= d && m_dif >= -d) {
		diff = true;
	}

	return diff;
}

bool Lane::m_diff(Line l1, Line l2, double d, double& md) {
	//returns true if the difference between the m of the two lines is within the tolerance
	//specified, also provides the difference in the gradients (md)
	bool diff;
	diff = false;
	double m1 = abs(l1.get_md());
	double m2 = abs(l2.get_md());

	double m_dif = abs(m1 - m2);

	if (m_dif >= d) {
		diff = true;
	}

	md = m_dif;

	return diff;
}

bool Lane::similar_line(Vec4i l1, Vec4i l2, int d_req) { // fist line to compare against, second line found, and minimum distance
//first line is found, so this func should iterate thru the lines agian to find the second longest line that meets the
//grad and min. dist requirements.
	Point l1t, l1b, l2t, l2b;

	l1t.x = l1[0];
	l1t.y = l1[1];
	l1b.x = l1[2];
	l1b.y = l1[3];
	double m1;
	int m_1 = grad(l1t, l1b, m1);


	l2t.x = l2[0];
	l2t.y = l2[1];
	l2b.x = l2[2];
	l2b.y = l2[3];
	double m2;
	int m_2 = grad(l2t, l2b, m1);

	bool parallel, dist, similar;

	dist = false;
	parallel = false;
	similar = false;

	if (m_1 == m_2) {

		parallel = true;

	}


	int c1, c2;

	get_eqn(l1t, l1b, m1, c1);
	get_eqn(l2t, l2b, m2, c2);

	int d_deno, d_num, d;

	d_num = abs(c1 - c2);
	d_deno = sqrt((m1 * m1) + 1);
	d = d_num / d_deno;

	if (d >= d_req) {

		dist = true;

	}

	if (parallel == true && dist == true) {

		similar = true;

	}

	return similar;
}

int Lane::vert_point(double m, int c, int x) {
	//returns the y cordinate of a corresponding x cordinate for a provided line
	//Line is decribed by gradient and y intercept
	int y;

	y = (m * x) + c;

	return y;
}


double Lane::dist_point_line(Point p, Vec4i l) {
	// line eqn is in the form aX+bY+c=0
	//get_eqn gives m&c for y=mx+c so need rearranging
	//https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

	double m;
	int c;
	Point lt, lb;

	lt.x = l[0];
	lt.y = l[1];
	lb.x = l[2];
	lb.y = l[3];

	get_eqn(lt, lb, m, c);

	double a, b, c_d, d; //d is the distance
	a = 1 / (m + 1e-17);
	b = 1;
	c_d = 1 / (c + 1e-17);

	double d_num = abs((p.x * a) + (p.y * b) + c_d);
	double d_deno = sqrt((a * a) + (b * b));

	d = d_num / d_deno;

	return d;

}

double Lane::dist_point_line(Point p, int cor, bool vert) {
	//overloaded version for vert/horizontal lines
	// for this, the eqn is x=c or y=c
	// so a or b=1.
	//https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

	double d;
	int l_eq; // this is either y or x for the num of the eqn (see link)
	if (vert == false) {
		l_eq = p.y; // horizontal line, y=c
	}
	//if (vert == true) {
	l_eq = p.x;
	//}
	d = abs(l_eq - cor);

	return d;

}

double Lane::ang_twolines(Line l1, Line l2) {
	//Finds the angle between two lines
	//https://www.cuemath.com/geometry/angle-between-two-lines/
	Point t, b;
	double thet, m1, m2;

	t = l1.mid();
	b = l2.mid();

	m1 = l1.get_md();
	m2 = l2.get_md();

	double x, y; //x and y lenghts
	x = m1 - m2;
	y = 1 + (m1 * m2);
	thet = (atan(y / (x + 1e-7))) * (180 / (CV_PI));


	return thet;
}

double Lane::ang_w_yaxis(Line l) {
	// Finds the angle made by a line and the y axis (ie angle with respect to the vertical
	double x, y;
	Point t1, t2;
	t1 = l.p1();
	t2 = l.p2();

	x = abs((t1.x - t2.x) + 1e-7);
	y = abs((t1.y - t2.y) + 1e-7);

	double ang = atan(x / y) * (180 / CV_PI);

	return ang;
}

double Lane::left_ang_actual(double ang) {
	//8th polynomial 

	 ///LEFT HAND POLYNOMIAL corrections
	double x8 = -0.00000000002762 * pow(ang, 8);
	double x7 = 0.00000000723596 * pow(ang, 7);
	double x6 = -0.00000077531081 * pow(ang, 6);
	double x5 = 0.00004379573716 * pow(ang, 5);
	double x4 = -0.00139864316490 * pow(ang, 4);
	double x3 = 0.02503948180878 * pow(ang, 3);
	double x2 = -0.22892493875966 * pow(ang, 2);
	double x1 = 1.06950692165667 * ang;

	double ang_c = x8 + x7 + x6 + x5 + x4 + x3 + x2 + x1 - 0.00160337963045;

	return ang_c;
}

double Lane::right_ang_actual(double ang) {
	//8th polynomial actually 

	///RIGHT HAND POLYNOMIAL corrections
	double x8 = -0.000000000046327 * pow(ang, 8);
	double x7 = 0.000000010856502 * pow(ang, 7);
	double x6 = -0.000001033707636 * pow(ang, 6);
	double x5 = 0.000051166289673 * pow(ang, 5);
	double x4 = -0.001395636167556 * pow(ang, 4);
	double x3 = 0.020570996276171 * pow(ang, 3);
	double x2 = -0.153209554827159 * pow(ang, 2);
	double x1 = 0.822589150688026 * ang;

	double ang_c = x8 + x7 + x6 + x5 + x4 + x3 + x2 + x1 - 0.016082490707942;

	return ang_c;
}

bool Lane::min_parrallel_dist(Line l1, Line l2, double d) {
	//retuns true if the two parrallel lines are within a specified distance
	bool dist_l = false;

	double c1, c2, m1, m2, dist;

	m1 = l1.get_md();
	m2 = l2.get_md();

	if (abs(m1 - m2) > 0.05) { //minimum difference in gradient between the lines
		dist_l = false; //lines arent parralel if the gradients are different
	}
	else {
		c1 = l1.get_cd();
		c2 = l2.get_cd();

		double nume, deno;

		nume = abs(c1 - c2);
		deno = sqrt(1 + (m1 * m1));

		dist = nume / deno;

		if (dist == d) {
			dist_l = true;
		}
	}
	return dist_l;

}

bool Lane::min_parrallel_dist(Line l1, Line l2, double d, double tol) {
	//overloaded version
	//retuns true if the two parrallel lines are within a specified distance
	// allows the required difference in gradient to be specified
	bool dist_l = false;

	double c1, c2, m1, m2, dist;

	m1 = l1.get_md();
	m2 = l2.get_md();

	if (abs(m1 - m2) > tol) {
		dist_l = false; //lines arent parralel if the gradients are different
	}
	else {
		c1 = l1.get_cd();
		c2 = l2.get_cd();

		double nume, deno;

		nume = abs(c1 - c2);
		deno = sqrt(1 + (m1 * m1));

		dist = nume / deno;

		if (dist < d) {
			dist_l = true;
		}

	}
	return dist_l;

}



vector<Line> Lane::lane_lines(vector<Vec4i>& lines) {
	//finds the lane lines, the second line found is based on the angles the two lines will make. If its 0, its likely on the same
	//line
	Line L1, L2; //longest and second longest lines

	Vec4i l_f, l_f2; //longest and second longest
	double longest, longest_2;
	vector<Vec4i> lane;
	vector<Vec4i> lane2; //vector of 4ints that have a angle with L1 of > specified value
	vector<vector<int>>lanes_fd;
	vector<Line>lns;

	lane = lines;

	Vec4i l_i = lines[0]; //fist line in the vector
	double disti = sqrt(((l_i[0] - l_i[2]) * (l_i[0] - l_i[2])) + ((l_i[1] - l_i[3]) * (l_i[1] - l_i[3])));//initial distance
	l_f = l_i; //longest line
	longest = disti;

	bool sz;
	sz = false;


	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l_R = lines[i];
		double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));


		if (dist >= longest) {
			longest = dist;
			L1(l_R);

		}
	}

	lns.push_back(L1);


	if (lane.size() > 1) {
		sz = true;
	}

	if (sz == true) {//if the size of lanes is greater than 1, it runs again to find second longest line

		for (size_t i = 0; i < lane.size(); i++) {
			Vec4i l_R = lane[i];
			double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));

			Line lf;

			lf(l_R);
			double th = ang_twolines(L1, lf);
			double md;
			bool m_d = m_diff(L1, lf, 0.2, md);

			Point t = lf.p1(0, 0);
			double m = abs(lf.get_md());

			if ((th >= 5 && th <= 45) && (m_d == true) && m > 0.45) {
				lane2.push_back(l_R);
			}


		}


		if (lane2.size() > 0) {

			Vec4i l_i = lane2[0]; //fist line in the vector
			double disti = sqrt(((l_i[0] - l_i[2]) * (l_i[0] - l_i[2])) + ((l_i[1] - l_i[3]) * (l_i[1] - l_i[3])));//initial distance
			l_f = l_i; //longest line
			double longest = disti;

			for (size_t i = 0; i < lane2.size(); i++) {

				Vec4i l_R = lane2[i];
				double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));

				if (dist >= longest) {
					longest = dist;
					//L2.set_cords(l_R);
					L2(l_R);
					//l_f = l_R;
					lns.push_back(L2);

				}

			}
		}


	}

	return lns;

}

vector<Line> Lane::lane_lines(vector<Vec4i>& lines, double m_tol, int th_l, int th_h) {
	// m_tol is the minimum gradient, th_l and th_h are the upper and lower limits
	// for the angle betwwen the lines found.
	Line L1, L2; //longest and second longest lines

	Vec4i l_f, l_f2; //longest and second longest
	double longest, longest_2;
	vector<Vec4i> lane;
	vector<Vec4i> lane2; //vector of 4ints that have a angle with L1 of > specified value
	vector<vector<int>>lanes_fd;
	vector<Line>lns;

	lane = lines;

	Vec4i l_i = lines[0]; //fist line in the vector
	double disti = sqrt(((l_i[0] - l_i[2]) * (l_i[0] - l_i[2])) + ((l_i[1] - l_i[3]) * (l_i[1] - l_i[3])));//initial distance
	l_f = l_i; //longest line
	longest = disti;

	bool sz;
	sz = false;


	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l_R = lines[i];
		double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));



		if (dist >= longest) {
			longest = dist;

			L1(l_R);


		}


	}

	lns.push_back(L1);


	if (lane.size() > 1) {
		sz = true;
	}

	if (sz == true) {//if the size of lanes is greater than 1, it runs again to find second longest line

		for (size_t i = 0; i < lane.size(); i++) {
			Vec4i l_R = lane[i];
			double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));

			Line lf;
			//lf.set_cords(l_R);
			lf(l_R);
			double th = ang_twolines(L1, lf);
			double md;
			bool m_d = m_diff(L1, lf, 0.2, md);

			Point t = lf.p1(0, 0);
			double m = abs(lf.get_md());


			if ((th_l >= 5 && th <= th_h) && (m_d == true) && m > m_tol) {
				lane2.push_back(l_R);
			}

		}


		if (lane2.size() > 0) {

			Vec4i l_i = lane2[0]; //fist line in the vector
			double disti = sqrt(((l_i[0] - l_i[2]) * (l_i[0] - l_i[2])) + ((l_i[1] - l_i[3]) * (l_i[1] - l_i[3])));//initial distance
			l_f = l_i; //longest line
			double longest = disti;

			for (size_t i = 0; i < lane2.size(); i++) {

				Vec4i l_R = lane2[i];
				double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));

				if (dist >= longest) {
					longest = dist;
					//L2.set_cords(l_R);
					L2(l_R);
					//l_f = l_R;
					lns.push_back(L2);

				}

			}
		}


	}


	return lns;
}

vector<Line> Lane::det_in_out(Line l1, Line l2, Point cntrd, int xc, int yc) {
	//Determines which line is the outer and inner line determined by the centroid point (cntrd)
	//lns[0]=inner, lns[1]=outer
	int tpx;
	tpx = (fr.size().width) / 2;
	bool v = true;

	Point l1_m = l1.mid(xc, yc);
	Point l2_m = l2.mid(xc, yc);
	vector<Line> lns;

	double d1m, d2m, d_line;
	d_line = dist_point_line(cntrd, tpx, v);
	d1m = dist_point_line(l1_m, tpx, v);
	d2m = dist_point_line(l2_m, tpx, v);

	//putText(fr, to_string(dt1), l1t, FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
	//putText(fr, to_string(d1m), Point(l1t.x,l1t.y-20), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);

	if (((d1m) < d_line) && ((d2m) > d_line)) {
		//l1 is the inner
		lns.push_back(l1);
		lns.push_back(l2);

	}

	if (((d1m > d_line) && ((d2m) < d_line))) {
		//l1 is the outer
		lns.push_back(l2);
		lns.push_back(l1);

	}

	return lns;

}


Point Lane::poly_ctr(Vec4i& ln1, Vec4i& ln2) {
	//returns the centroid of a 4 sided polygon
	//The polygon is defines by the two lines porvided, therefore the polygon 
	//is the lane found, the centroid is the center of the line
	vector<vector<Point>> pts;
	vector<Point>l1, l2;

	Point l1t, l1b, l2t, l2b;
	vector<int> xpts, ypts;

	Point road_cord[1][4];


	road_cord[0][0] = Point(ln1[0] + cr.size().width * 0.5, ln1[1] + (cr.size().height * 3));//1 Point(l1[2] + cr.size().width * 0.5, l1[3] + (cr.size().height * 3));
	road_cord[0][1] = Point(ln2[0] + cr.size().width * 0.5, ln2[1] + (cr.size().height * 3));//2 Point(l2[2] + cr.size().width * 0.5, l2[3] + (cr.size().height * 3));
	road_cord[0][2] = Point(ln1[2] + cr.size().width * 0.5, ln1[3] + (cr.size().height * 3));//3 Point(l1[0] + cr.size().width * 0.5, l1[1] + (cr.size().height * 3))
	road_cord[0][3] = Point(ln2[2] + cr.size().width * 0.5, ln2[3] + (cr.size().height * 3));//4 Point(l2[0] + cr.size().width * 0.5, l2[1] + (cr.size().height * 3));

	int d1 = point_distance(road_cord[0][0], road_cord[0][2]);
	int d2 = point_distance(road_cord[0][1], road_cord[0][3]);

	
	if (d1 > d2) {
		xpts.push_back(ln1[0]);
		xpts.push_back(ln2[0]);
		xpts.push_back(ln2[2]);
		xpts.push_back(ln1[2]);

		ypts.push_back(ln1[1]);
		ypts.push_back(ln2[1]);
		ypts.push_back(ln1[3]);
		ypts.push_back(ln2[3]);
	}
	if (d2 > d1) {

		xpts.push_back(ln2[0]);
		xpts.push_back(ln1[0]);
		xpts.push_back(ln1[2]);
		xpts.push_back(ln2[2]);

		ypts.push_back(ln2[1]);
		ypts.push_back(ln1[1]);
		ypts.push_back(ln1[3]);
		ypts.push_back(ln2[3]);
	}

	//cout << "\nHERE IN POLY_CTR x y size "<<" "<<xpts.size()<<" "<< ypts.size();

	if (xpts.size() >= 4 && ypts.size() >= 4) {


		double a, as, cxa, cxas, cya, cyas;
		float cx, cy;
		long double mul;
		a = 0;
		cxa = 0;
		cya = 0;
		for (int i = 0; i <= 2; i++) {
			as = abs((0.5) * ((xpts[i] * ypts[i + 1]) - (xpts[i + 1] * ypts[i])));
			a += as;
		}

		int n = 0;
		for (int i = 0; i <= 10e6; i++) {
			a = (a / 10);
			n++;
			if (a < 1) {
				break;
			}
		}

		mul = ((1 / (6 * a)) * pow(10, -n));

		for (int i = 0; i <= 2; i++) {
			cxas = abs(((xpts[0] + xpts[i + 1]) * ((xpts[i] * ypts[i + 1]) - (xpts[i + 1] * ypts[i]))));
			cxa += cxas;
		}
		//cx = (1 / (6 * a)) * cxa;
		cx = mul * cxa;

		for (int i = 0; i <= 2; i++) {
			cyas = abs(((ypts[0] + ypts[i + 1]) * ((xpts[i] * ypts[i + 1]) - (xpts[i + 1] * ypts[i]))));
			cya += cyas;
		}
		//cy = (1 / (6 * a)) * cya;
		cy = mul * cya;

		//cx = 0;
		//cy = 0;

		Point cntrd;
		cntrd.x = cx;
		cntrd.y = cy;

		cout << "\nHERE IN POLY_CTR x y size " << " " << cx << " " << cy;
		return cntrd;
	}
}

Point Lane::lane_poly_ctr(Line l1, Line l2, int xc, int yc) {
	//Function takes two lines and draws a polygon, finds the centroid
	//and determines which is the inner and outer lane.
	//overloaded version, take Line as argument instead of Vec4i
	vector<int>xs;
	vector<int>ys;

	Point l1t, l1b, l2t, l2b;
	l1t = l1.p1(xc, yc);
	l1b = l1.p2(xc, yc);

	l2t = l2.p1(xc, yc);
	l2b = l2.p2(xc, yc);

	xs.push_back(l1t.x);
	xs.push_back(l2t.x);
	xs.push_back(l2b.x);
	xs.push_back(l1b.x);
	xs.push_back(l1t.x); //the last value, n=5, is the starting point n=1

	ys.push_back(l1t.y);
	ys.push_back(l2t.y);
	ys.push_back(l2b.y);
	ys.push_back(l1b.y);
	ys.push_back(l1t.y); //the last value, n=5, is the starting point n=1

	double a, as, cxa, cxas, cya, cyas;
	double cx, cy;
	long double mul;
	a = 0;
	cxa = 0;
	cya = 0;
	Point l1_mi = l1.mid();
	Point l2_mi = l2.mid();
	Point cntrd;

	if (xs.size() >= 5 && ys.size() >= 5) {


		Point scr_ctr; //screen centre based on dimensions of fr
		scr_ctr.x = (fr.size().width) * 0.5;
		scr_ctr.y = (fr.size().height) * 0.5;


		int ni = xs.size();

		for (int i = 0; i <= 3; i++) {
			as = (0.5) * ((xs[i] * ys[i + 1]) - (xs[i + 1] * ys[i]));
			a += as;
		}

		int n = 0;
		for (int i = 0; i <= 10e6; i++) {
			a = (a / 10);
			n++;
			if (a < 1) {
				break;
			}
		}

		mul = ((1 / (6 * a)) * pow(10, -n));

		for (int i = 0; i <= 3; i++) {
			cxas = (xs[i] + xs[i + 1]) * ((xs[i] * ys[i + 1]) - (xs[i + 1] * ys[i]));
			cxa += cxas;
		}
		//cx = (1 / (6 * a)) * cxa;
		cx = mul * cxa;

		for (int i = 0; i <= 3; i++) {
			cyas = (ys[i] + ys[i + 1]) * ((xs[i] * ys[i + 1]) - (xs[i + 1] * ys[i]));
			cya += cyas;
		}
		//cy = (1 / (6 * a)) * cya;
		cy = mul * cya;



		cntrd.x = cx;
		cntrd.y = cy;


		circle(fr, cntrd, 5, Scalar(255, 255, 255), -1);



	}
	return cntrd;

}

void Lane::virtual_road() {
	//Applies the perspective transform function provided by OpenCV
	//to detect the lanes
	Rect roi;
	roi.x = 0;
	roi.y = (fr.size().height) * (0.75);
	roi.width = fr.size().width;
	roi.height = (fr.size().height) * (0.25);

	Point cp1, cp2, cp3, cp4;

	///virtual_road()////
	vector<Point2f>L2, R1;
	Point2f r1, r2, r3, r4;

	//two lines on either side of the screen where the lanes should appear
	r1.x = 63;
	r1.y = 375;
	r2.x = 2;  ///includes the outer lanes
	r2.y = 415;
	r3.x = 553;
	r3.y = 377;
	r4.x = 640;
	r4.y = 430;


	R1.push_back(r1);
	R1.push_back(r2);
	R1.push_back(r3);
	R1.push_back(r4);

	cp1.x = 0;
	cp1.y = (fr.size().height) * (0.75);
	cp2.x = 0;
	cp2.y = (fr.size().height);
	cp3.x = fr.size().width;
	cp3.y = (fr.size().height) * (0.75);
	cp4.x = fr.size().width;
	cp4.y = (fr.size().height);

	L2.push_back(cp1);
	L2.push_back(cp2);
	L2.push_back(cp3);
	L2.push_back(cp4);

	//the two lines R1 and L2 describe the region where
	//the transformation would take place
	Mat mtrix = getPerspectiveTransform(R1, L2);
	Mat res;
	warpPerspective(fr, res, mtrix, fr.size()); 

	Rect roi2;
	roi2.x = 0;
	roi2.y = (fr.size().height) * (0.75);
	roi2.width = fr.size().width;
	roi2.height = (fr.size().height) * (0.25);

	road = res(roi2);

	//The reference line vertical line drawn at the mid point
	//of the screen
	tp_adj_r.x = (cr.size().width) / 2;
	tp_adj_r.y = cr.size().height;
	bt_adj_r.x = (cr.size().width) / 2;
	bt_adj_r.y = 1;

	line(fr, r1, r2, Scalar(255, 255, 255), 3, LINE_AA); //left line
	line(fr, r3, r4, Scalar(255, 255, 255), 3, LINE_AA);


	if (!road.empty()) {
		imshow("virtual road", road);
	}

}
void Lane::drw_ref_line() {

	line(fr, tp_adj_r, bt_adj_r, Scalar(255, 0, 255), 3, LINE_AA);

}

void Lane::virtual_road(Line l1, Line l2) {
	///Applies the perspective transform function provided by OpenCV
	//to detect the lanes
	//Overloaded version allows R1 and L2 to be specified
	int d1, d2;


	d1 = l1.dist();
	d2 = l2.dist();

	int xcL, ycL;
	xcL = 0;
	ycL = (cr.size().height * 3);

	int xcR, ycR;
	xcR = cr.size().width * 0.5;
	ycR = (cr.size().height * 3);


	Point r1L, r2L, r1R, r2R;
	///r2L    r2R////
	///       ////
	///r1L    r1R////
	double m1, m2;
	m1 = l1.get_md();
	m2 = l2.get_md();

	if (m1 < 0 && m2>0) {
		//l1 left l2 right
		r1L = l1.p1(xcL, ycL);
		r2L = l1.p2(xcL, ycL);

		r1R = l2.p1(xcR, ycR);
		r2R = l2.p2(xcR, ycR);

	}

	if (m2 < 0 && m1>0) {
		//l1 right l2 left
		r1R = l1.p1(xcR, ycR);
		r2R = l1.p2(xcR, ycR);

		r1L = l2.p1(xcL, ycL);
		r2L = l2.p2(xcL, ycL);

	}


	Point2f cp1, cp2, cp3, cp4;

	cp1.x = 0;
	cp1.y = (can.size().height) * (0.75);

	cp2.x = 0;
	cp2.y = (can.size().height);

	cp3.x = can.size().width;
	cp3.y = (can.size().height) * (0.75);

	cp4.x = can.size().width;
	cp4.y = (can.size().height);

	vector<Point2f>L1, L2;

	L1.push_back(r2L);
	L1.push_back(r1L);
	L1.push_back(r1R);
	L1.push_back(r2R);

	L2.push_back(cp1);
	L2.push_back(cp2);
	L2.push_back(cp3);
	L2.push_back(cp4);

	circle(fr, r1L, 8, Scalar(255, 255, 255), FILLED, LINE_8);
	circle(fr, r2L, 8, Scalar(0, 0, 255), FILLED, LINE_8);
	circle(fr, r1R, 8, Scalar(255, 0, 255), FILLED, LINE_8);
	circle(fr, r2R, 8, Scalar(0, 255, 0), FILLED, LINE_8);

	int x1, x2, x3, x4, y1, y2, y3, y4;
	x1 = r1L.x;
	x2 = r2L.x;
	x3 = r1R.x;
	x4 = r2R.x;

	y1 = r1L.y;
	y2 = r2L.y;
	y3 = r1R.y;
	y4 = r2R.y;

	putText(fr, to_string(x1), Point(250, 255), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
	putText(fr, to_string(y1), Point(290, 255), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1);
	putText(fr, to_string(x2), Point(250, 295), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
	putText(fr, to_string(y2), Point(290, 295), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
	putText(fr, to_string(x3), Point(250, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1);
	putText(fr, to_string(y3), Point(290, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1);
	putText(fr, to_string(x4), Point(250, 375), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1);
	putText(fr, to_string(y4), Point(290, 375), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1);

	vector<Point2f> R1, R2;
	Point2f r1, r2, r3, r4;

	r1.x = 150;
	r1.y = 355;
	r2.x = 20;
	r2.y = 479;
	r3.x = 510;
	r3.y = 355;
	r4.x = 639;
	r4.y = 470;

	R1.push_back(r1);
	R1.push_back(r2);
	R1.push_back(r3);
	R1.push_back(r4);

	int di1 = point_distance(r1, r2);
	int di2 = point_distance(r3, r4);
	int di3 = point_distance(r2, r4);

	putText(fr, to_string(di1), Point(250, 205), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);
	putText(fr, to_string(di2), Point(290, 225), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);
	putText(fr, to_string(di3), Point(250, 235), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);


	line(fr, r1, r2, Scalar(255, 255, 255), 3, LINE_AA);
	line(fr, r3, r4, Scalar(255, 255, 255), 3, LINE_AA);

	Rect roi;
	roi.x = 0;
	roi.y = 0;
	roi.width = 615;
	roi.height = 175;


	Mat mtrix = getPerspectiveTransform(R1, L2);

	Mat res;
	warpPerspective(fr, res, mtrix, fr.size());

	Rect roi2;
	roi2.x = 0;
	roi2.y = (can.size().height) * (0.75);
	roi2.width = can.size().width;
	roi2.height = (can.size().height) * (0.25);


	Mat road = res(roi2);

	if (!road.empty()) {
		imshow("virtual road", road);
	}

}

double Lane::parralel_dist_lines(Line l1, Line l2) {
	//returns the parralel dist between the two lines
	//https://byjus.com/jee/distance-between-2-parallel-lines/

	double c1, c2, m1, m2, dist;

	m1 = l1.get_md();
	m2 = l2.get_md();

	if (abs(m1 - m2) > 0.3) {
		dist = 0; //lines arent parralel if the gradients are different
	}

	c1 = l1.get_cd();
	c2 = l2.get_cd();

	double nume, deno;

	nume = abs(c1 - c2);
	deno = sqrt(1 + (m1 * m1));

	dist = nume / deno;

	return dist;

}

vector<Line> Lane::parrallel_lines(vector<Vec4i>& lines, double dis_min) {
	// finds the lines parralel to the first line found, based on the minmum parrallel dist
	// provided in the argument
	//lines.size() must be >1

	vector<vector<int>>lanes_fd;
	vector<Line>lns;

	Vec4i l_i = lines[0];
	Line I(l_i);
	lns.push_back(I);

	for (size_t i = 0; i < lines.size(); i++) {
		Vec4i l_R = lines[i];
		Line L(l_R);

		double p_dist = parralel_dist_lines(L, I);
		//cout << "\nParrallel dist " << dis_min;
		if (p_dist >= dis_min) {

			lns.push_back(L);

		}

	}
	return lns;
}

void Lane::road_can(Scalar lower, Scalar upper) {
	Mat HSV, mask;
	
	cvtColor(road, HSV, COLOR_BGR2HSV);

	inRange(HSV, lower, upper, mask);

	Canny(mask, can_road, 0, 255);

	imshow("road can", can_road);

}


vector<Line> Lane::line_aggr(vector<Line>lines, double d) {
	vector<Line> l_1;
	vector<Line> l_2;
	vector<Line> lane;
	Line L1f, L2f; //the longest aggregate line

	if (!lines.empty()) {

		Line Li1 = lines[0];
		for (size_t i = 0; i < lines.size(); i++) {
			Line l1 = lines[i];
			bool m_sim = m_diff_tol(Li1, l1, d);
			if (m_sim == true) {
				l_1.push_back(l1);
			}
		}

		if (l_1.size() > 1) {
			Line L_1 = l_1[0];
			int d_y;
			Point md1 = L_1.mid();
			d_y = md1.y;
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_1.size(); i++) {
				Line L = l_1[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_1[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L1f(l1t, l12);
		}
		else {
			L1f = l_1[0];
		}

		lane.push_back(L1f);

		return lane;
	}

}


vector<Line> Lane::line_aggr(vector<Line>lines, double d, double tol) {
	vector<Line> l_1;
	vector<Line> l_2;
	vector<Line> lane;
	Line L1f, L2f; //the longest aggregate line

	if (!lines.empty()) {

		Line Li1 = lines[0];
		for (size_t i = 0; i < lines.size(); i++) {
			Line l1 = lines[i];
			bool m_sim = min_parrallel_dist(Li1, l1, d, tol);
			if (m_sim == true) {
				l_1.push_back(l1);
			}
		}

		if (l_1.size() > 1) {
			Line L_1 = l_1[0];
			int d_y;
			Point md1 = L_1.mid();
			d_y = md1.y;
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_1.size(); i++) {
				Line L = l_1[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_1[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L1f(l1t, l12);
		}
		else {
			L1f = l_1[0];
		}

		lane.push_back(L1f);

		return lane;
	}

}

vector<Line> Lane::line_aggr(vector<Line>lines, vector<Line>lines2, double d) {
	//This function combines the lines of each vector to create the longest lines for each vector.
	//of the lines found, the line closest to the y-axis(top of the screen)
	// and the line furthest from the y-axis are chosen.
	// Then, for the aggreagte line, the top point is taken from the line closest to the y-axis
	// and the bottom point is taken from the line furthest from the y-axis are chosen.

	vector<Line> l_1;
	vector<Line> l_2;
	vector<Line> lane;
	Line L1f, L2f; //the longest aggregate line

	if (!lines.empty() && !lines2.empty()) {


		Line Li1 = lines[0];
		for (size_t i = 0; i < lines.size(); i++) {
			Line l1 = lines[i];
			bool m_sim = m_diff_tol(Li1, l1, d);
			if (m_sim == true) {
				l_1.push_back(l1);
			}
		}

		Line Li2 = lines2[0];
		for (size_t i = 0; i < lines2.size(); i++) {
			Line l2 = lines2[i];
			bool m_sim = m_diff_tol(Li1, l2, d);
			if (m_sim == true) {
				l_2.push_back(l2);
			}
		}


		if (l_1.size() > 1) {
			Line L_1 = l_1[0];
			int d_y;
			Point md1 = L_1.mid();
			d_y = md1.y;
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_1.size(); i++) {
				Line L = l_1[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_1[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L1f(l1t, l12);
		}
		else {
			L1f = l_1[0];
		}


		if (l_2.size() > 1) {
			Line L_1 = l_2[0];
			int d_y;
			Point md1 = L_1.mid();
			d_y = md1.y;
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_2.size(); i++) {
				Line L = l_2[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_2[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L2f(l1t, l12);
		}
		else {
			L2f = l_1[0];
		}


		lane.push_back(L1f);
		lane.push_back(L2f);

		return lane;
	}



}

vector<Line> Lane::line_aggr(vector<Line>lines, vector<Line>lines2, double d, double tol) {
	//This function combines the lines of each vector to create the longest lines for each vector.
	//overloaded version that uses the min parallel dist check

	vector<Line> l_1;
	vector<Line> l_2;
	vector<Line> lane;
	Line L1f, L2f; //the longest aggregate line

	if (!lines.empty() && !lines2.empty()) {


		Line Li1 = lines[0]; //ensures that all lines have the same gardient and acceptable minimum parralel distance
		for (size_t i = 0; i < lines.size(); i++) {
			Line l1 = lines[i];
			bool m_sim = min_parrallel_dist(Li1, l1, d, tol); //minimum dist, m tolerance
			if (m_sim == true) {
				l_1.push_back(l1);
			}
		}

		Line Li2 = lines2[0];
		for (size_t i = 0; i < lines2.size(); i++) {
			Line l2 = lines2[i];
			bool m_sim = min_parrallel_dist(Li1, l2, d, tol); //minimum dist, m tolerance
			if (m_sim == true) {
				l_2.push_back(l2);
			}
		}


		if (l_1.size() > 1) { //of the lines found, the line closest to the y-axis(top of the screen)
			Line L_1 = l_1[0];// and the line furthest from the y-axis are chosen.
			int d_y;          // Then, for the aggreagte line, the top point is taken from the line closest to the y-axis
			Point md1 = L_1.mid(); // and the bottom point is taken from the line furthest from the y-axis are chosen.
			d_y = md1.y;           
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_1.size(); i++) {
				Line L = l_1[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_1[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L1f(l1t, l12);
		}
		else {
			L1f = l_1[0];
		}
		//int l = L1f.get_c();

		if (l_2.size() > 1) {//Same for the second line
			Line L_1 = l_2[0]; 
			int d_y;
			Point md1 = L_1.mid();
			d_y = md1.y;
			int longest = d_y;
			Point l1t;
			Point l12;

			for (size_t i = 0; i < l_2.size(); i++) {
				Line L = l_2[i];
				Line l1;
				int d_y1;
				Point md1 = L.mid();
				d_y1 = md1.y;
				if (d_y1 > longest) {
					longest = d_y1;
					l1 = L;
				}
				else {
					l1 = L_1;
				}
				l1t = l1.p1(); //line is furtherst away y axis

				Line L2 = l_2[i];
				Line l2;
				int d_y2;
				Point md2 = L2.mid();
				d_y2 = md2.y;
				int shortest = d_y2;
				if (d_y2 < shortest) {
					shortest = d_y2;
					l2 = L2;
				}
				else {
					l2 = L2;
				}
				l12 = l2.p2(); //line is closest to y axis
			}
			L2f(l1t, l12);
		}
		else {
			L2f = l_1[0];
		}


		lane.push_back(L1f);
		lane.push_back(L2f);

		return lane;
	}



}

void Lane::find_R_lane_warped() {
	//tp_adj, bt_adj is ref line uses the probablistic Hough line transform
	//This function identifies the Right lane
	if (!can_road.empty()) {

		Rect roiR;
		roiR.x = can_road.size().width * (0.5);
		roiR.y = 0;
		roiR.width = can_road.size().width * (0.5);
		roiR.height = can_road.size().height;

		Mat croppedR = can_road(roiR);

		vector<Vec4i> linesPR;
		vector<Vec4i> linesPRm;
		//vector<Vec4i> linesPR1; //first set of lines
		vector<Line> linesPR1; //first set of lines
		vector<Line> linesPR2; // second set of lines 
		vector<vector<Vec4i>>lane;
		vector<Line>p_lines; //parallel lines
		vector<Line>long_lines; //longest 2 lines
		vector<Line>similar_lines;
		vector<Line>lanes;
		vector<Line>lanes2;
		vector<Line>lanesf;
		Line lr1, lr2; //the two individual, aggregate lines found

		bool l = false;

		HoughLinesP(croppedR, linesPR, 1, CV_PI / 180, 30, 30, 4);
		//cout << "R_LANE_WARPED "<< linesPR.size();

		if (!croppedR.empty()) {
			//imshow("right roi", croppedR);
		}
		for (size_t i = 0; i < linesPR.size(); i++) {

			Vec4i l_R = linesPR[i];
			double dist = sqrt(((l_R[0] - l_R[2]) * (l_R[0] - l_R[2])) + ((l_R[1] - l_R[3]) * (l_R[1] - l_R[3])));  //finds the longest lines
			//if (dist >= 70) {
			double m = grad(l_R);
			Line l0 = linesPR[0];
			Line l1 = linesPR[i];

			if ((abs(m) > 0.5) && m > 0) {  //isolates the lines of specified gradients

				linesPRm.push_back(linesPR[i]);

			}


		}

		for (size_t i = 0; i < linesPRm.size(); i++) {

			Line l1(linesPRm[i]);
			Line l0 = linesPRm[0];
			Point r1, r2;
			r1 = l1.p1(cr.size().width * 0.5, 0);
			r2 = l1.p2(cr.size().width * 0.5, 0);

			double p_dist = parralel_dist_lines(l0, l1);
			double m;
			bool m_d = m_diff(l0, l1, 0.01, m);

			//Checks the first line found against all successive lines, 
			//then if they are parrallel and within a specified distance
			//they are loaded into linesPR1, if not they are loaded into linesPR2
			bool is_parallel = min_parrallel_dist(l0, l1, 30.0, 0.2);
			if (is_parallel == true) {
				linesPR1.push_back(l1);
			}
			else {
				linesPR2.push_back(l1);
			}

		}

		Line ref(tp_adj_r, bt_adj_r);

		if (!linesPR1.empty()) {

			for (size_t i = 0; i < linesPR1.size(); i++) {
				Line l0 = linesPR1[0];
				Line l1 = linesPR1[i];
				Point rt, rb, md;
				rt = l1.p1(cr.size().width * 0.5, 0);
				rb = l1.p2(cr.size().width * 0.5, 0);
				md = l1.mid(cr.size().width * 0.5, 0);

				double p_dist = parralel_dist_lines(l0, l1);
			}
		}

		if (!linesPR2.empty()) {

			for (size_t i = 0; i < linesPR2.size(); i++) {
				Line l0 = linesPR2[i];
				Line l1 = linesPR2[i];
				Point rt, rb, md;
				rt = l1.p1(cr.size().width * 0.5, 0);
				rb = l1.p2(cr.size().width * 0.5, 0);
				md = l1.mid(cr.size().width * 0.5, 0);

				double p_dist = parralel_dist_lines(l0, l1);

			}
		}

		if (!linesPR1.empty()) {

			lanes = line_aggr(linesPR1, 4.0, 0.2); //of the lines in each linesPR vector
			if (!lanes.empty()) {                  // the line_aggr is used to find a single line
				Line l1;
				for (size_t i = 0; i < lanes.size(); i++) {
					l1 = lanes[0];
					//lr1 = lanes[0];
					Point rt, rb;
					rt = l1.p1(cr.size().width * 0.5, 0);
					rb = l1.p2(cr.size().width * 0.5, 0);
					line(fr, rt, rb, Scalar(0, 0, 255), 3, LINE_AA);
					putText(fr, to_string(lanes.size()), Point(100, 100), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 2);
				}
				lanesf.push_back(l1);

			}

		}

		if (!linesPR2.empty()) {
			//cout << "\nHERE ";
			lanes2 = line_aggr(linesPR2, 4.0, 0.2); //of the lines in each linesPR vector
			if (!lanes2.empty()) {                   // the line_aggr is used to find a single line
				Line l1;
				for (size_t i = 0; i < lanes2.size(); i++) {
					l1 = lanes2[0];
					//lr2 = lanes2[0];
					Point rt, rb;
					rt = l1.p1(cr.size().width * 0.5, 0);
					rb = l1.p2(cr.size().width * 0.5, 0);
					line(fr, rt, rb, Scalar(0, 255, 255), 3, LINE_AA);
					putText(fr, to_string(lanes2.size()), Point(120, 100), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
				}
				lanesf.push_back(l1);
			}

			double f_l_dist = parralel_dist_lines(lr1, lr2);
			putText(fr, to_string(f_l_dist), Point(120, 130), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
		}

		if (lanesf.size() == 1) {
			right_R.push_back(lanesf[0]); //one line found
		}
		if (lanesf.size() > 1) { //more than one, the inner and outer is determined 
			Line l1, l2;		 //using the first two lines
			l1 = lanesf[0];
			l2 = lanesf[1];
			Point cntr = lane_poly_ctr(l1, l2, cr.size().width * 0.5, 0);

			right_R = det_in_out(l1, l2, cntr, cr.size().width * 0.5, 0);

			putText(fr, to_string(right_R.size()), Point(105, 130), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);

			Line in, out;

			if (right_R.size() > 1) {
				in = right_R[0];
				out = right_R[1];

				putText(fr, "INNER", in.p2(cr.size().width * 0.5, 0), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
				putText(fr, "OUTER", out.p2(cr.size().width * 0.5, 0), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
			}

		}

	}
}

void Lane::find_L_lane_warped() {
	//Find the lane on the left side.
	//Works the same as the find_R_lane_warped()
	if (!can_road.empty()) {

		Rect roiL;
		roiL.x = 0;
		roiL.y = 0;
		roiL.width = can_road.size().width * (0.5);
		roiL.height = can_road.size().height;

		Mat croppedL = can_road(roiL);

		vector<Vec4i> linesPL;
		vector<Vec4i> linesPLm;
		//vector<Vec4i> linesPL1; //first set of lines
		vector<Line> linesPL1; //first set of lines
		vector<Line> linesPL2; // second set of lines 
		vector<vector<Vec4i>>lane;
		vector<Line>p_lines; //parallel lines
		vector<Line>long_lines; //longest 2 lines
		vector<Line>similar_lines;
		vector<Line>lanes;
		vector<Line>lanes2;
		vector<Line>lanesf;
		bool l = false;

		Line ll1, ll2; //the two individual, aggregate lines found

		HoughLinesP(croppedL, linesPL, 1, CV_PI / 180, 30, 30, 4);
		//cout << "R_LANE_WARPED "<< linesPR.size();

		if (!croppedL.empty()) {
			//imshow("right roi", croppedR);
		}
		for (size_t i = 0; i < linesPL.size(); i++) {

			Vec4i l_L = linesPL[i];
			double dist = sqrt(((l_L[0] - l_L[2]) * (l_L[0] - l_L[2])) + ((l_L[1] - l_L[3]) * (l_L[1] - l_L[3])));  //finds the longest lines
			//if (dist >= 70) {
			double m = grad(l_L);
			Line l0 = linesPL[0];
			Line l1 = linesPL[i];

			if ((abs(m) > 0.5) && m < 0) {

				linesPLm.push_back(linesPL[i]);

			}

		}

		for (size_t i = 0; i < linesPLm.size(); i++) {

			Line l1(linesPLm[i]);
			Line l0 = linesPLm[0];
			Point r1, r2;
			r1 = l1.p1();
			r2 = l1.p2();

			double p_dist = parralel_dist_lines(l0, l1);
			double m;
			bool m_d = m_diff(l0, l1, 0.01, m);

			bool is_parallel = min_parrallel_dist(l0, l1, 25.0, 0.2);
			if (is_parallel == true) {
				linesPL1.push_back(l1);
			}
			else {
				linesPL2.push_back(l1);
			}

		}

		Line ref(tp_adj_r, bt_adj_r);

		if (!linesPL1.empty()) {

			for (size_t i = 0; i < linesPL1.size(); i++) {
				Line l0 = linesPL1[0];
				Line l1 = linesPL1[i];
				Point rt, rb, md;
				rt = l1.p1();
				rb = l1.p2();
				md = l1.mid();

				double p_dist = parralel_dist_lines(l0, l1);

			}
		}

		if (!linesPL2.empty()) {

			for (size_t i = 0; i < linesPL2.size(); i++) {
				Line l0 = linesPL2[i];
				Line l1 = linesPL2[i];
				Point rt, rb, md;
				rt = l1.p1();
				rb = l1.p2();
				md = l1.mid();

				double p_dist = parralel_dist_lines(l0, l1);

			}
		}

		if (!linesPL1.empty()) {
			//cout << "\nHERE ";
			lanes = line_aggr(linesPL1, 4.0, 0.2);
			if (!lanes.empty()) {
				Line l1;
				for (size_t i = 0; i < lanes.size(); i++) {
					l1 = lanes[0];
					ll1 = lanes[0];
					Point rt, rb;
					rt = l1.p1();
					rb = l1.p2();
					line(fr, rt, rb, Scalar(255, 0, 0), 3, LINE_AA);
					putText(fr, to_string(lanes.size()), Point(400, 100), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 2);
				}
				lanesf.push_back(l1);
			}

		}

		if (!linesPL2.empty()) {
			//cout << "\nHERE ";
			lanes2 = line_aggr(linesPL2, 4.0, 0.2);
			if (!lanes2.empty()) {
				Line l1;
				for (size_t i = 0; i < lanes2.size(); i++) {
					l1 = lanes2[0];
					ll2 = lanes2[0];
					Point rt, rb;
					rt = l1.p1();
					rb = l1.p2();
					line(fr, rt, rb, Scalar(255, 255, 0), 3, LINE_AA);
					putText(fr, to_string(lanes2.size()), Point(420, 100), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				}
				lanesf.push_back(l1);
			}

			double f_l_dist = parralel_dist_lines(ll1, ll2);

			putText(fr, to_string(f_l_dist), Point(420, 130), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);


		}

		if (lanesf.size() == 1) {
			left_L.push_back(lanesf[0]);

		}

		if (lanesf.size() > 1) {
			Line l1, l2;
			l1 = lanesf[0];
			l2 = lanesf[1];

			Point cntr = lane_poly_ctr(ll1, ll2, 0, 0);

			left_L = det_in_out(ll1, ll2, cntr, 0, 0);

			putText(fr, to_string(left_L.size()), Point(105, 140), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);

			Line in, out;

			if (left_L.size() > 1) {
				in = left_L[0];
				out = left_L[1];

				putText(fr, "INNER", in.p2(0, 0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
				putText(fr, "OUTER", out.p2(0, 0), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 2);
			}
		}

	}


}

void Lane::right_dist_check(int d1, int d2, int dm, double ang) {
	//dm is the mimnmum dist. between the ref line and the lane
	// md of 75 means an angle of 10, if md changes get corrseponding
	//ang by measuring

	if (d1 < dm || d2 < dm) {

		v1 = 180;
		v2 = 85;
		th = (ang * 1000);

	}

}

void Lane::left_dist_check(int d1, int d2, int dm, double ang) {
	//dm is the mimnmum dist. between the ref line and the lane
	// md of 75 means an angle of 10, if md changes get corrseponding
	//ang by measuring

	if (d1 < dm || d2 < dm) {

		v1 = 85;
		v2 = 0;
		th = (ang * 1000);

	}

}

void Lane::right_dist_check(int d1, int dm, double ang) {
	//dm is the mimnmum dist. between the ref line and the lane
	// md of 75 means an angle of 10, if md changes get corrseponding
	//ang by measuring

	if (d1 < dm) {

		v1 = 180;
		v2 = 85;
		th = (ang * 1000);

	}

}

void Lane::left_dist_check(int d1, int dm, double ang) {
	//dm is the mimnmum dist. between the ref line and the lane
	// md of 75 means an angle of 10, if md changes get corrseponding
	//ang by measuring

	if (d1 < dm) {

		v1 = 85;
		v2 = 0;
		th = (ang * 1000);

	}

}

double Lane::get_bigger_angle(double a1, double a2) {
	if (a1 > a2) {
		return a1;
	}
	if (a1 < a2) {
		return a2;
	}

}

void Lane::det_warped_ang() {
	Line ref(tp_adj_r, bt_adj_r);
	Point ref_md = ref.mid();
	//determines the angles made by the lines found on the left and right with respect
	// to the vertical. if its 0, it means the robots going straight, anymore is a deviations from the track and
	// needs to be corrected.
	//the program will only turn if the detected angle is >10

	if (right_R.size() > 1) { //two lines found, they are parrallel so the angle with respect to the vertical
		Line lane = right_R[0]; //are only slightly different
		Line lane2 = right_R[1];

		double ang_md = ang_w_yaxis(lane);
		double ang_cor = left_ang_actual(ang_md); // the warp function is not perfect and the perspective 
		double ang_md2 = ang_w_yaxis(lane2);      // found is not the ideal of looking down directly from above
		double ang_cor2 = left_ang_actual(ang_md2); //so, the angle shown by the program was compared to the angle made
													//in real life and a relation ship was determined by polynomial regression
		Point l1m, l2m;								//the left_ang_actual function corrects the found angle.
		l1m = lane.mid(cr.size().width * 0.5, 0);   //the same function is done to the laft side lane.
		l2m = lane2.mid(cr.size().width * 0.5, 0);   

		int d1, d2;

		d1 = point_distance(ref_md, l1m);
		d2 = point_distance(ref_md, l2m);

		putText(fr, "Angle right lane ", Point(255, 270), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_md), Point(255, 285), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_cor), Point(255, 300), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_md2), Point(155, 285), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_cor2), Point(155, 300), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(d1), Point(155, 315), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(d2), Point(155, 330), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

		if (ang_md >= 10 && ang_md2 >= 10) {
			//tracking to the right correct left
			double angc = get_bigger_angle(ang_cor, ang_cor2);  //uses the largest of the angles between inner/outer
			v1 = 180;
			v2 = 85;
			th = (ang_cor * 1000);
		}

		//right_dist_check(d1,d2,75,10.0);
	}
	

	if (right_R.size() > 0) {
		Line lane = right_R[0];


		double ang_md = ang_w_yaxis(lane);
		double ang_cor = left_ang_actual(ang_md);

		Point l1m;
		l1m = lane.mid(cr.size().width * 0.5, 0);


		int d1;

		d1 = point_distance(ref_md, l1m);


		putText(fr, "Angle right lane ", Point(255, 270), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_md), Point(255, 285), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_cor), Point(255, 300), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(d1), Point(155, 315), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);

		if (ang_md >= 10) {
			//tracking to the right correct left
			v1 = 180;
			v2 = 85;
			th = (ang_cor * 1000);
		}

		//right_dist_check(d1,75, 10.0);
	}

	if (left_L.size() > 1) {
		Line lane = left_L[0];
		Line lane2 = left_L[1];

		double ang_md = ang_w_yaxis(lane);
		double ang_cor = left_ang_actual(ang_md);

		double ang_md2 = ang_w_yaxis(lane2);
		double ang_cor2 = left_ang_actual(ang_md2);

		Point l1m, l2m;
		l1m = lane.mid();
		l2m = lane2.mid();

		int d1, d2;

		d1 = point_distance(ref_md, l1m);
		d2 = point_distance(ref_md, l2m);

		putText(fr, "Angle left lane ", Point(255, 320), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_md), Point(255, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_cor), Point(255, 350), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_md), Point(155, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_cor), Point(155, 350), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(d1), Point(155, 365), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(d2), Point(155, 380), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);


		if (ang_md >= 10 && ang_md2 >= 10) {
			//tracking to the right correct left
			//the angle used is the largest one
			double angc = get_bigger_angle(ang_cor, ang_cor2);
			v1 = 85;
			v2 = 0;
			th = (angc * 1000);
		}



	}

	if (left_L.size() > 0) {
		Line lane = left_L[0];


		double ang_md = ang_w_yaxis(lane);
		double ang_cor = right_ang_actual(ang_md);

		Point l1m;
		l1m = lane.mid();

		int d1;

		d1 = point_distance(ref_md, l1m);

		putText(fr, "Angle left lane ", Point(255, 320), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_md), Point(255, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_cor), Point(255, 350), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(d1), Point(255, 365), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);

		if (ang_md >= 10) {
			//tracking to the right correct left
			v1 = 85;
			v2 = 0;
			th = (ang_cor * 1000);
		}


	}

	if (right_R.size() > 0 && left_L.size() > 0) {

		Line lane = right_R[0];


		double ang_md = ang_w_yaxis(lane);
		double ang_cor = right_ang_actual(ang_md);

		putText(fr, "Angle right lane ", Point(255, 270), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_md), Point(255, 285), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		putText(fr, to_string(ang_cor), Point(355, 300), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 1);

		if (ang_md >= 10) {
			//tracking to the right correct left
			v1 = 180;
			v2 = 85;
			th = (ang_cor * 1000);
		}

		Line lane2 = left_L[0];


		double ang_md2 = ang_w_yaxis(lane2);
		double ang_cor2 = left_ang_actual(ang_md2);

		if (ang_md2 >= 10) {
			//tracking to the right correct left
			v1 = 180;
			v2 = 85;
			th = (ang_cor2 * 1000);
		}

		putText(fr, "Angle left lane ", Point(255, 320), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_md2), Point(255, 335), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1);
		putText(fr, to_string(ang_cor2), Point(355, 350), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 0), 1);

	}


}


vector <short int> Lane::spd_values() {
	//Function returns a vector with the direction and angle to
	//turn to the Vision class.
	vector <short int> val;
	val.push_back(v1);
	val.push_back(v2);
	val.push_back(th);
	return val;

}
