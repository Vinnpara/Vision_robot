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
//Where the Vision class is used and the templates are loaded.
void main() {
	
	VideoCapture cap(1);  //capture video from a webcam
	if (!cap.isOpened()) {
		cerr << "ERROR: Could not open camera" << std::endl;
		
	}

	int hmin = 0, smin = 0, vmin = 0;  //HSV limits for finding lanes
	int hmax = 179, smax = 255, vmax = 255;

	//Read in the pictures used as templates.
	Mat right = imread("Right.jpg");

	Mat left = imread("Left.jpg");

	Mat stop = imread("Red_Sign(jpg).jpg");
	GaussianBlur(stop, stop, Size(7, 7), 0, 0, BORDER_DEFAULT);
	
	Mat go = imread("Green_Sign_hist(jpg).jpg");
	GaussianBlur(go, go, Size(7, 7), 0, 0, BORDER_DEFAULT);

	//initalize the vision class
	Vision V(right, left, stop, go, cap);

	while (1) {
		
			V.get_info(); //receive from arduino
			V.capt();     //capture video
			//V.manual();   //manually control (if needed)
			V.info();    //display info from arduino

			V.stop_go();   //Detect srop/go signs
			V.Road(hmin, hmax, smin, smax, vmin, vmax);   //detec lanes
			V.left_right();  //detect left/right arrows

			V.disp();    //display the processed feed on the screen
			waitKey(1);
		
	}

	
}

