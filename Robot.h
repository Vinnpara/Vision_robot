
#pragma once

#include"SerialPort.h"
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )


#pragma comment(lib, "winmm.lib")


class Robot {
	short int v1, v2;
	short unsigned int d,li;
	//char v;

public:	
	Robot(short int ve1, short int ve2);
	Robot();
	void set_values(short int ve1, short int ve2);
	void manual();
	void em_stop();
	void robot_go();
	void robot_stop();
	void robot_right();
	void robot_left();
	void robot_ste_right();
	void robot_ste_left();
	void robot_st_right(double t);
	void robot_st_left(double t);
	void robot_turn_right(short int& v1, short int& v2);
	void serial_r_w();
	void send_info();
	void get_info();
	void turn_right(double t);
	void turn_left(double t);
	short int g_v1();
	short int g_v2();
	short int get_dis();
	short int get_li();
	~Robot();
};