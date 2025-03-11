#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
#include "odometry.h"

using namespace std;
using namespace pros;

void autonomous() {
	driveStraight2(-590);
	delay(9);
	driveTurn(-24);
	delay(5);
	driveStraight2(-570, 25);
	mogo.set_value(true);
	delay(48);
	driveTurn2(270); // goes to ring#2
	delay(48);
	CONVEYOR.move(112);
	delay(10);
	CONVEYOR.move(-127);
	delay(9);
	CONVEYOR.move(127);
	delay(100);
	driveStraight2(800);
	delay(700);
	driveTurn2(187); //turns to right mid ring
	delay(5);
	CONVEYOR.move(124);
	driveStraight2(350); //scores the ring
	delay(48);
	driveStraight2(-100);
	driveTurn2(210); // turns to other ring
	delay(9);
	driveStraight2(240, 75);
	delay(10);




	//example of lb macro thingy
	//lbmove==1; //picks position


}