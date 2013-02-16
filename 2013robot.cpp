#include "WPIlib.h"
#include "Math.h"
#include "CANJaguar.h"
#define PI 3.14159265

class DefaultRobot : public SimpleRobot {
	Joystick driveJoy;
	Joystick shootJoy;
	CANJaguar jagA;
	CANJaguar jagB;
	CANJaguar jagC;
	CANJaguar jagD;
	CANJaguar shootFront;
	CANJaguar shootRear;
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
		 * hopper gate solenoid have placeholder values
		 */
		driveJoy(1),
		shootJoy(2),
		jagA(11), //invert
		jagB(10), //invert
		jagC(6),
		jagD(3),
		shootFront(7),
		shootRear(2),
		lifterA(1),
		lifterB(2),
		suctionA(3),
		suctionB(4),
		hopperGateA(7),
		hopperGateB(8),
		compressor(1, 1)
		
	{
		Watchdog().SetExpiration(1);
		compressor.Start();
	}
	
	void Autonomous(void){
		
	}
	
	void OperatorControl(void){
		Watchdog().SetEnabled(true);
		
		/*
		jagA.ChangeControlMode(jagA.kSpeed);
		jagA.ConfigEncoderCodesPerRev(360);
		jagA.EnableControl();
		jagA.ChangeControlMode(jagA.kPercentVbus);
		jagA.EnableControl();
		Watchdog().Feed();
		jagA.ChangeControlMode(jagB.kSpeed);
		jagA.ConfigEncoderCodesPerRev(360);
		jagA.EnableControl();
		jagA.ChangeControlMode(jagB.kPercentVbus);
		jagA.EnableControl();
		Watchdog().Feed();
		jagA.ChangeControlMode(jagC.kSpeed);
		jagA.ConfigEncoderCodesPerRev(360);
		jagA.EnableControl();
		jagA.ChangeControlMode(jagC.kPercentVbus);
		jagA.EnableControl();
		Watchdog().Feed();
		jagA.ChangeControlMode(jagD.kSpeed);
		jagA.ConfigEncoderCodesPerRev(360);
		jagA.EnableControl();
		jagA.ChangeControlMode(jagD.kPercentVbus);
		jagA.EnableControl();
		Watchdog().Feed();
		*/
		
		while(IsOperatorControl()){
<<<<<<< HEAD
			phi = driveJoy.GetRawAxis(3);
			leftJoyX = driveJoy.GetRawAxis(1);
			leftJoyY = -driveJoy.GetRawAxis(2);
=======
			phi = joystick.GetRawAxis(3);
			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);
			
			//Left joystick strafe tolerance
			if((-.1<leftJoyX)&&(leftJoyX<.1)){
				leftJoyX = 0;
			}
			if((-.1<leftJoyY)&&(leftJoyY<.1)){
				leftJoyY = 0;
			}
			
>>>>>>> mec2
			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));
			
			//theta NaN handling
			if((leftJoyY == 0)&&(leftJoyX != 0)){
				if(leftJoyX > 0){
					theta = 0;
				} else if (leftJoyX < 0){
					theta = PI;
				}
			}
			else if((leftJoyX == 0)&&(leftJoyY != 0)){
				if(leftJoyY > 0){
					theta = PI/2;
				} else if (leftJoyY < 0){
					theta = (3*PI)/2;
				}
			} else {
				theta = atan((leftJoyY)/(leftJoyX));
			}
			
			
			//if in Quadrant 2 or 3 add 180 degrees, if in Quadrant 4 add 360
			if(((leftJoyX<0)&&(leftJoyY>0))||((leftJoyX<0)&&(leftJoyY<0))){ 
				theta += PI;
			} else if ((leftJoyX>0)&&(leftJoyY<0)){
				theta += (2*PI);
			}
			
<<<<<<< HEAD
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
=======
			
			
			if (joystick.GetRawButton(3)) {
				theta = 0;
				radius = .5;
			} else if (joystick.GetRawButton(4)) {
				theta = PI / 2;
				radius = .35;
			} else if (joystick.GetRawButton(1)) {
				theta = PI;
				radius = .5;
			} else if (joystick.GetRawButton(2)) {
				theta = (3 * PI) / 2;
				radius = .35;
			}
			
			
			
>>>>>>> mec2
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
			
			//Diagnostics output
			printf("x: %f y: %f  phi: %f\n", leftJoyX, leftJoyY,phi);
<<<<<<< HEAD
			printf("a: %f b: %f c: %f d: %f\n", outA, outB, outC, outD);
=======
			//("a: %f b: %f c: %f d: %f\n", jagA.Get(), jagB.Get(), jagC.Get(), jagD.Get());
>>>>>>> mec2
			printf("theta: %f radius: %f\n", theta,radius);
			//printf("Axis 2: %f", shootJoy.GetRawAxis(2));
			
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
			if(shootJoy.GetRawAxis(2)<-.2){
				shootFront.Set((shootJoy.GetRawAxis(2)));
				shootRear.Set((shootJoy.GetRawAxis(2)));
			}else{
				shootFront.Set(0);
				shootRear.Set(0);
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
