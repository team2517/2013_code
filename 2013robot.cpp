#include "WPIlib.h"
#include "Math.h"
#define PI 3.14159265

class DefaultRobot : public SimpleRobot {
	Joystick driveJoy;
	Joystick shootJoy;
	CANJaguar jagA;
	CANJaguar jagB;
	CANJaguar jagC;
	CANJaguar jagD;
	CANJaguar shootRear;
	CANJaguar shootFront;
	Solenoid lifterA;
	Solenoid lifterB;
	Solenoid suctionA;
	Solenoid suctionB;
	Solenoid hopperGateA;
	Solenoid hopperGateB;
	Compressor compressor;
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
		 * 
		 * Missing shooter motors and hopper gate solenoid
		 */
		driveJoy(1),
		shootJoy(2),
		jagA(11), //invert
		jagB(10), //invert
		jagC(6),
		jagD(3),
		lifterA(1),
		lifterB(2),
		suctionA(3),
		suctionB(4),
		compressor(1, 1)
		
	{
		Watchdog().SetExpiration(1);
		compressor.Start();
	}
	
	void Autonomous(void){
		
	}
	
	void OperatorControl(void){
		Watchdog().SetEnabled(true);
		
		while(IsOperatorControl()){
			phi = driveJoy.GetRawAxis(3);
			leftJoyX = driveJoy.GetRawAxis(1);
			leftJoyY = -driveJoy.GetRawAxis(2);
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
				} else if((leftJoyX == 0)&&(leftJoyY!=0)){
					if(leftJoyY>0){
						theta = PI/2;
					} else if (leftJoyY<0){
						theta = (3*PI)/2;
					}
				} else {
					theta = 0;
				}
			}
			
			//Power equations
			outA = ((radius)*(sin(theta+(PI/4)))+phi);
			outB = ((radius)*(cos(theta+(PI/4)))+phi);
			outC = ((radius)*(cos(theta+(PI/4)))-phi);
			outD = ((radius)*(sin(theta+(PI/4)))-phi);
			
			//Motor output
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
			
			/*
			 * Diagnostics output
			printf("x: %f y: %f  phi: %f\n", leftJoyX, leftJoyY,phi);
			printf("a: %f b: %f c: %f d: %f\n", jagA.Get(), jagB.Get(), jagC.Get(), jagD.Get());
			printf("theta: %f radius: %f\n", theta,radius);
			*/
			
			//Pneumatics
			if(driveJoy.GetRawButton(1)){
				lifterA.Set(false);
				lifterB.Set(true);
			}else{
				lifterA.Set(true);
				lifterB.Set(false);
			}
			if(driveJoy.GetRawButton(2)){
				suctionA.Set(true);
			} else {
				suctionA.Set(false);
			}
			if(driveJoy.GetRawButton(3)){
				suctionB.Set(true);
			}else{
				suctionB.Set(false);
			}
			
			//Shooter
			if(shootJoy.GetRawAxis(2)>.2){
				shootFront.Set(shootJoy.GetRawAxis(2));
				shootRear.Set(shootJoy.GetRawAxis(2));
			}
			if(shootJoy.GetRawButton(1)){
				hopperGateA.Set(true);
				hopperGateB.Set(false);
			}else{
				hopperGateA.Set(false);
				hopperGateB.Set(true);
			}
			
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
