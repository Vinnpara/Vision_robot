#include<chrono>



class Timer1 {

    std::chrono::time_point<std::chrono::system_clock>StartTime;
    std::chrono::time_point<std::chrono::system_clock>EndTime;
    bool bRunning = false;

public:
    void start();

    void stop();

    double elapsedMilliseconds();

    double elapsedSeconds();


};
