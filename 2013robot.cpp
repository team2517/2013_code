#include "WPIlib.h"
#include "Math.h"
#define PI 3.14159265

class DefaultRobot : public SimpleRobot {
	Joystick joystick;
	CANJaguar jagA;
	CANJaguar jagB;
	CANJaguar jagC;
	CANJaguar jagD;
	double theta;
	double radius;
	double phi;
	double leftJoyX;
	double leftJoyY;
	double outA;
	double outB;
	double outC;
	double outD;
	
public:
	DefaultRobot(void):
		/*
		 * 3 = green = d
		 * 6 = red = c
		 * 11 = white = a
		 * 10 = yellow = b
		 */
		joystick(1),
		jagA(11), //invert
		jagB(10), //invert
		jagC(6),
		jagD(3)
	{
		Watchdog().SetExpiration(1);
	}
	
	void Autonomous(void){
		
	}
	
	void OperatorControl(void){
		Watchdog().SetEnabled(true);
		
		while(IsOperatorControl()){
			phi = joystick.GetRawAxis(3);
			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);
			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));
			theta = atan((leftJoyY)/(leftJoyX));
			
			//if in Quadrant 2 or 3 add 180 degrees, if in Quadrant 4 add 360
			if(((leftJoyX<0)&&(leftJoyY>0))||((leftJoyX<0)&&(leftJoyY<0))){ 
				theta += PI;
			} else if ((leftJoyX>0)&&(leftJoyY<0)){
				theta += (2*PI);
			}
			
			//theta NaN handling
			if(theta != theta){
				if((leftJoyY == 0)&&(leftJoyX!=0)){
					if(leftJoyX>0){
						theta = 0;
					} else if (leftJoyX<0){
						theta = PI;
					}
				}
				else if((leftJoyX == 0)&&(leftJoyY!=0)){
					if(leftJoyY>0){
						theta = PI/2;
					} else if (leftJoyY<0){
						theta = (3*PI)/2;
					}
				} else {
					theta = 0;
				}
			}
			
			outA = ((radius)*(sin(theta+(PI/4)))+phi);
			outB = ((radius)*(cos(theta+(PI/4)))+phi);
			outC = ((radius)*(cos(theta+(PI/4)))-phi);
			outD = ((radius)*(sin(theta+(PI/4)))-phi);
			
			
			if(outA > 1){
				jagA.Set(-1);
			} else if(outA < -1){
				jagA.Set(1);
			} else {
				jagA.Set(-outA);
			}
			if(outB > 1){
				jagB.Set(-1);
			} else if(outB < -1){
				jagB.Set(1);
			} else {
				jagB.Set(-outB);
			}
			if(outC > 1){
				jagC.Set(1);
			} else if(outC < -1){
				jagC.Set(-1);
			} else {
				jagC.Set(outC);
			}
			if(outD > 1){
				jagD.Set(1);
			} else if(outD < -1){
				jagD.Set(-1);
			} else {
				jagD.Set(outD);
			}
			
			printf("x: %f y: %f  phi: %f\n", leftJoyX, leftJoyY,phi);
			printf("a: %f b: %f c: %f d: %f\n", jagA.Get(), jagB.Get(), jagC.Get(), jagD.Get());
			printf("theta: %f radius: %f\n", theta,radius);
			
			/*
			jagA.Set((radius)*(sin(theta))+phi);
			jagB.Set((radius)*(cos(theta))-phi);
			jagC.Set((radius)*(cos(theta))+phi);
			jagD.Set((radius)*(sin(theta))-phi);
			*/
		}
	}
};

START_ROBOT_CLASS(DefaultRobot);
