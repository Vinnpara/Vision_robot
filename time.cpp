#include<cmath>
#include<chrono>
#include <iostream>
#include "time.h"
using namespace std;
using namespace chrono;

void Timer1::start()
{
    StartTime = system_clock::now();
    bRunning = true;
}

void Timer1::stop()
{
    EndTime = system_clock::now();
    bRunning = false;
}

double Timer1::elapsedMilliseconds()
{
    time_point<system_clock> endTime;

    if (bRunning)
    {
        endTime = system_clock::now();
    }
    else
    {
        endTime = EndTime;
    }

    return duration_cast<milliseconds>(endTime - StartTime).count();
}

double Timer1::elapsedSeconds()
{
    return elapsedMilliseconds() / 1000.0;
}