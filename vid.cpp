#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/objdetect.hpp>

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

#include"Improc.h"
#include"Lane.h"

#include "timer.h"
#include "Robot.h"
#include"Vision.h"
#include"SerialPort.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )


#pragma comment(lib, "winmm.lib")

using namespace cv;
using namespace std;
using namespace chrono;


void main() {
	
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		cerr << "ERROR: Could not open camera" << std::endl;
		
	}

	int hmin = 0, smin = 0, vmin = 0;
	int hmax = 179, smax = 255, vmax = 255;

	Mat right = imread("Right.jpg");

	Mat left = imread("Left.jpg");

	Mat stop = imread("Red_Sign(jpg).jpg");
	GaussianBlur(stop, stop, Size(7, 7), 0, 0, BORDER_DEFAULT);
	
	Mat go = imread("Green_Sign_hist(jpg).jpg");
	GaussianBlur(go, go, Size(7, 7), 0, 0, BORDER_DEFAULT);


	Vision V(right, left, stop, go, cap);

	while (1) {
		
			V.get_info();
			V.capt();
			V.manual();
			V.info();

			V.stop_go();  
			V.Road(hmin, hmax, smin, smax, vmin, vmax);  
			V.left_right(); 

			V.disp();
			waitKey(1);
		
	}

	
}

