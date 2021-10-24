
#include <iostream>

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

#include "timer.h"
#include "Robot.h"
#include"SerialPort.h"


#pragma comment(lib, "winmm.lib")
using namespace std;
using namespace chrono;


#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )


char output[MAX_DATA_LENGTH];
char incomingData[MAX_DATA_LENGTH];
char* port = "\\\\.\\COM3";
char input[MAX_DATA_LENGTH];
SerialPort arduino(port);

Robot::Robot(short int ve1,short int ve2) {

	v1=ve1;
	v2=ve2;

}

Robot::Robot() {

}

void Robot::set_values(short int ve1, short int ve2) {
	v1 = ve1;
	v2 = ve2;
}

Robot::~Robot() {
	//cout << "\nDest ";
}

void Robot::serial_r_w() {
	
	//writing part
	const int NMAX = 64;
	short int* ve1, * ve2;
	//short int ve1, ve2;
	char buffin[NMAX];
	

	char* pi, * pb, * e, * st, * en;
	pb = buffin;
	pi = pb;

	ve1 = (short int*)pi;
	*ve1 = v1;
	pi += sizeof(short int);

	ve2 = (short int*)pi;
	*ve2 = v2;
	pi += sizeof(short int);

	/*e = (char*)pi;
	*e = v;*/
	
	arduino.writeSerialPort(buffin, 4);
	
	Sleep(10);
	arduino.readSerialPort(output, MAX_DATA_LENGTH);
	//reading part
	char* p_buffer, * p;
	short unsigned int* l, * di;

	short unsigned int light, dist;
	int lig, dis;

	p_buffer = output;
	p = p_buffer;

	l = (short unsigned int*)p;
	p += sizeof(short unsigned int);
	di = (short unsigned int*)p;

	light = l[0];
	dist = di[0];
	lig = (int)light;
	dis = (int)dist;
	d = dist;
	
	cout << "\nlight " << light << "\tDist " << dist <<' '<<"RIGHT_MOTOR " << v1 << "\tLEFT_MOTOR " << v2 <<' '<< buffin[0]<<' '<< buffin[1] << ' ' << buffin[2] << buffin[3] <<endl;
};

void Robot::send_info() {
	//writing part, Send data to the arduino
	const int NMAX = 64;
	short int* ve1, * ve2;
	
	char buffin[MAX_DATA_LENGTH];
	char end = 'e';

	char* pi, * pb, * e;
	int sz;
	sz = 1;

	pb = buffin;
	pi = pb;

	
	ve1 = (short int*)pi;
	*ve1 = v1;
	pi += sizeof(short int);
	
	
	ve2 = (short int*)pi;
	*ve2 = v2;
	pi += sizeof(short int);
	
	Sleep(10);
	arduino.writeSerialPort(buffin, 4);

	cout <<"\nRIGHT_MOTOR " << v1 << "\tLEFT_MOTOR " << v2 <<buffin[0]<<' '<< buffin[1] << ' ' << buffin[2]<<' ' << buffin[3]<<endl;
	
}

void Robot::get_info() {
	//receive data from arduino
	Sleep(10);
	arduino.readSerialPort(output, MAX_DATA_LENGTH);

	char* p_buffer, * p;
	short unsigned int* l, * di;
	short int* vi1, * vi2;

	short unsigned int light, dist;
	short int v1, v2;
	int lig, dis;

	p_buffer = output;
	p = p_buffer;

	l = (short unsigned int*)p;
	p += sizeof(short unsigned int);
	di = (short unsigned int*)p;
	
	p += sizeof(short unsigned int);
	vi1 = (short int*)p;
	p += sizeof(short int);
	vi2 = (short int*)p;


	light = l[0];
	dist = di[0];

	v1 = vi1[0];
	v2 = vi2[0];

	lig = (int)light;
	dis = (int)dist;

	d = dist;
	li = lig;

	cout << "\nlight " << light << "\tDist " << dist << "\nVI1 " << v1 << "\tVI2 " << v2 << endl;

}

void Robot::manual() {
	//Manually control robot
	if (KEY(VK_DOWN)) //FORWARD
	{
		v1 = 0;
		v2 = 180;
		send_info();
	}
	if (KEY(VK_UP))
	{
		v1 = 180;
		v2 = 0;
		send_info();
	}
	if (KEY(VK_LEFT))
	{
		v1 = 0; //0
		v2 = 0; //85
		send_info();


	}
	if (KEY(VK_RIGHT)) //V1 right 
	{
		v1 = 180; //0
		v2 = 180; //85
		send_info();
	
	}
	if (KEY(VK_SPACE)) //STOP
	{
		v1 = 85;
		v2 = 85;
		send_info();
	}

};


void Robot::em_stop() {
	//Emergency stop
	if (d <= 7) {
		v1 = v2 = 85;
	}
	//send_info();
};

void Robot::robot_go() {
	//Go forward
	v1 = 180;
	v2 = 0;
	send_info();
};
void Robot::robot_stop() {

	v1 = 85;
	v2 = 85;
	send_info();
};
void Robot::robot_right() {
	//turn right
	v1 = 180;
	v2 = 180;

	send_info();
};

void Robot::robot_left() {
	//trun left
	v1 = 0;
	v2 = 0;
	send_info();


};

void Robot::robot_ste_right(){
	
	v1 = 85;
	v2 = 0;
	send_info();
}

void Robot::robot_ste_left() {
	v1 = 180;
	v2 = 85;
	send_info();
}

void Robot::robot_st_right(double t) {
	//the steer robot function takes the duration to 
	//keep one servo stationary and spin the other

	robot_ste_right();
	clock_t st, end;
	double time_gone;
	time_gone = 0;
	st = clock(); //start timer

	while (time_gone <= t) {

		end = clock();
		double time_gone = abs((double(st) - double(end)) / double(CLOCKS_PER_SEC));

		if (time_gone >=t) { //if the time reached by the timer the robot moves forward
			robot_go();
			break;

		}

	}

}

void Robot::robot_st_left(double t) {

	robot_ste_left();
	
	clock_t st, end;
	double time_gone;
	time_gone = 0;
	st = clock();

	while (time_gone <= t) {

		end = clock();
		double time_gone = abs((double(st) - double(end)) / double(CLOCKS_PER_SEC));

		if (time_gone >= t) {
			robot_go();
			break;

		}
		
	}

}


void Robot::turn_right(double t) {
	//Similar to steer function but the turn function spins
	//the servos in opposite directions to turn by 90 degrees for a 
	//specified time.
	clock_t st, end;
	int ti;
	double elapsed_time;
	elapsed_time = 0;
	robot_right();
	
	st = clock();

	while (elapsed_time <= t)

	{
		end = clock();
		
		elapsed_time = abs((double(st) - double(end)) / double(CLOCKS_PER_SEC));
		
		if (elapsed_time >= t) {
			robot_go();
			cout << "\nDone";
			//done = true;
			break;
		}
		cout << "\nTIME " << elapsed_time;
	}
}

void Robot::turn_left(double t) {


	clock_t st, end;

	string time1;
	int ti;
	double elapsed_time;
	elapsed_time = 0;
	robot_left();
	//start = system_clock::now();
	st = clock();

	while (elapsed_time <= t)

	{
		end = clock();
		
		elapsed_time = abs((double(st) - double(end)) / double(CLOCKS_PER_SEC));
	

		if (elapsed_time >= t) {
			robot_go();
			cout << "\nDone";
			//break;
		}
		cout << "\nTIME " << elapsed_time;
	}
}

void Robot::robot_turn_right(short int& v1, short int& v2) {
	
	bool turning = true;
	while (turning == true) {
		v1 = 85;
		v2 = 0;
		double ti;
		ti = high_resolution_time();
		if (ti >= 3.0) {
			v1 = 180;
			v2 = 0;
			turning = false;
			break;
		
		}
	}


}

short int Robot::g_v1() {

	return v1;
}

short int Robot::g_v2() {

	return v2;
}

short int Robot::get_li() {

	return li;
}
short int Robot::get_dis() {

	return d;
}
