#include "WPIlib.h"
#include "Math.h"
#define PI 3.14159265

class DefaultRobot: public SimpleRobot {
	Joystick joystick;
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
	double shooterRPM;

public:
	DefaultRobot(void) :
		/*
		 * 3 = green   = d = rear right 
		 * 6 = red     = c = rear left
		 * 11 = white  = a = front left
		 * 10 = yellow = b = front right
		 * 
		 * hopper gate solenoid have placeholder values
		 */
		joystick(1),
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

	void Autonomous(void) {

	}

	void OperatorControl(void) {
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
		*/
		Watchdog().Feed();
		shooterRPM = 0.0;
		while (IsOperatorControl()) {
			phi = joystick.GetRawAxis(3);
			leftJoyX = joystick.GetRawAxis(1);
			leftJoyY = -joystick.GetRawAxis(2);

			//Left joystick strafe tolerance
			if ((-.1 < leftJoyX) && (leftJoyX < .1)) {
				leftJoyX = 0;
			}
			if ((-.1 < leftJoyY) && (leftJoyY < .1)) {
				leftJoyY = 0;
			}

			radius = sqrt(pow(leftJoyX, 2) + pow(leftJoyY, 2));

			//theta NaN handling
			if ((leftJoyY == 0) && (leftJoyX != 0)) {
				if (leftJoyX > 0) {
					theta = 0;
				} else if (leftJoyX < 0) {
					theta = PI;
				}
			} else if ((leftJoyX == 0) && (leftJoyY != 0)) {
				if (leftJoyY > 0) {
					theta = PI / 2;
				} else if (leftJoyY < 0) {
					theta = (3 * PI) / 2;
				}
			} else if ((leftJoyY == 0) && (leftJoyX == 0)) {
				theta = 0;
			} else {
				theta = atan((leftJoyY) / (leftJoyX));
			}

			//if in Quadrant 2 or 3 add 180 degrees, if in Quadrant 4 add 360
			if (((leftJoyX < 0) && (leftJoyY > 0)) || ((leftJoyX < 0)
					&& (leftJoyY < 0))) {
				theta += PI;
			} else if ((leftJoyX > 0) && (leftJoyY < 0)) {
				theta += (2 * PI);
			}
			
			/*
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
			*/

			outA = ((radius) * (sin(theta + (PI / 4))) + phi);
			outB = ((radius) * (cos(theta + (PI / 4))) + phi);
			outC = ((radius) * (cos(theta + (PI / 4))) - phi);
			outD = ((radius) * (sin(theta + (PI / 4))) - phi);

			if (outA > 1) {
				jagA.Set(-1);
			} else if (outA < -1) {
				jagA.Set(1);
			} else {
				jagA.Set(-outA);
			}
			if (outB > 1) {
				jagB.Set(-1);
			} else if (outB < -1) {
				jagB.Set(1);
			} else {
				jagB.Set(-outB);
			}
			if (outC > 1) {
				jagC.Set(1);
			} else if (outC < -1) {
				jagC.Set(-1);
			} else {
				jagC.Set(outC);
			}
			if (outD > 1) {
				jagD.Set(1);
			} else if (outD < -1) {
				jagD.Set(-1);
			} else {
				jagD.Set(outD);
			}

			printf("x: %f y: %f phi: %f\n", leftJoyX, leftJoyY, phi);
			//("a: %f b: %f c: %f d: %f\n", jagA.Get(), jagB.Get(), jagC.Get(), jagD.Get());
			//printf("theta: %f radius: %f\n", theta, radius);
			
			//Shooter
			if(joystick.GetRawButton(1)){
				shooterRPM = 1.0;
			}
			if(joystick.GetRawButton(4)){
				shooterRPM = 0;
			}
			if(joystick.GetRawButton(2)){
				shooterRPM += 0.05;
			}
			if(joystick.GetRawButton(3)){
				shooterRPM -= 0.05;
			}
			if(shooterRPM>1.0){shooterRPM=1.0;}
			else if(shooterRPM<0){shooterRPM=0.0}
			shootFront.Set(shooterRPM);
			shootRear.Set(shooterRPM);
			Watchdog().Feed();
		}
	}
};

START_ROBOT_CLASS(DefaultRobot);
