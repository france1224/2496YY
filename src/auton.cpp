#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
#include "odometry.h"

using namespace std;
using namespace pros;

void autonomous() {
	driveStraightC(1000);
	driveTurn2(60);
	driveStraight2(500);
	doinker.set_value(true);
	delay(250);
	driveTurn2(0);
	driveStraight2(-1500);
	doinker.set_value(false);
	driveStraight2(-300);
	driveTurn2(180);
	driveStraight2(-860, 50);
	mogo.set_value(true);
	CONVEYOR.move(127);

	


	


}