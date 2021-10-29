# Vision_robot
Autonomous robot using OpenCV

All the functions for following tracks, indetifiying arrows sign, as well as differentiating between colored signs indicating stop/go and the Arduino code are included. The robot receives information via a serial connection to a computer and is controlled by an Arduino Uno controller. The main classes in the project are the Lane, Improc, Vision, Line and Robot classes.

The Lane class indentifies the lanes and determines steering angle, the Line class provides the geometric properties of the found lanes, such as mid point, gradient, y-intercept ect. Improc identifies the properties of the signs and the Vision class identifies the signs and lanes using Improc and the lanes class respectively. It then uses the Robot class to control the robot. The Robot class handles communication with the Arduino controller. For more details, please see the Vision-car pdf

Videos of robot:https://drive.google.com/drive/folders/1iR4UBmFqQ5Qie6uruZDFBLK5tIlisf12?usp=sharing
