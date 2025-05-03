#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"
#include "odometry.h"

using namespace std;
using namespace pros;

void autonomous() {


	// color = 1; //will move intake
	// color = 0; //get manual control of intake Wooo

	// for(int i = 0; i<5000; i+=10){ //this is FOR delays, the 5000 is 5 secs of delay (wowowwowow) hehehhee
	// 	ColorSort();
	// 	delay(10);
	// }

if(atn==0){

	// mogo.set_value(true);
	// color=1;
	// for(int i = 0; i<110000; i+=10){ //this is FOR delays, the 5000 is 5 secs of delay (wowowwowow) hehehhee
	// 	ColorSort();
	// 	delay(10);
	// }

	// driveStraight2(-663, 100);
	// delay(24);
	// driveTurn(-24);
	// delay(4);
	// driveStraight2(-560, 35);
	// mogo.set_value(true);
	// color=1;
	// delay(200);
	// driveTurn2(270); // goes to ring#2
	// delay(24);
	// driveStraight2(930);
	// delay(200);
	// driveTurn2(342);
	// delay(12);
	// doinker.set_value(true);
	// driveStraight2(1360);
	// driveTurn2(24); 
	// driveStraight2(300);
	// delay(200);
	// doinker.set_value(false);
	// driveStraight2(-240);
	// delay(24);
	// driveTurn2(135);
	// driveStraight2(-600);



	//driveStraight2(-600);

	driveStraight2(-672, 100);
	//delay(1);
	driveTurn2(24);
	//delay(2);
	driveStraight2(-560, 35);
	mogo.set_value(true);
	color=1;
	delay(150);
	driveTurn2(90); // goes to ring#2
	//delay(24);
	driveStraight2(930);
	delay(150);
	driveTurn2(172); //turns to right mid ring
	//delay(2);
	driveStraight2(394); //scores the ring
	delay(150);
	driveStraight2(-100);
	driveTurn2(200); // turns to other ring
	//delay(4);
	driveStraight2(240, 75);
	delay(150);
	driveTurn2(12);
	color=1;
	//delay(5);
	driveStraight2(1600);
	delay(2);
	doinker.set_value(true);
	driveTurn2(36);
	driveStraight2(400);
	//delay(1);
	driveTurn2(90);
	//delay(1);
	driveStraight2(-100);
	driveTurn2(84);
	driveStraight2(-300);
	driveTurn2(120);
	driveStraight2(500);




}else if(atn==1){
	// driveStraightC(1000);
	// driveTurn2(60);
	// driveStraight2(500);
	// doinker.set_value(true);
	// delay(250);
	// driveTurn2(0);
	// driveStraight2(-1500);
	// doinker.set_value(false);
	// driveStraight2(-300);
	// driveTurn2(180);
	// driveStraight2(-860, 50);
	// mogo.set_value(true);
	// CONVEYOR.move(127);

}

	


	


}