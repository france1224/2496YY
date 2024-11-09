#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"

using namespace std;
using namespace pros;
void autonomous() {

    if(atn==0){

    }else if(atn == 1){

    }else if(atn == 2){//redl
    		mogo.set_value(false);
			doinker.set_value(true);
			delay(1000);
			driveStraight2(-580);
			delay(700);
			driveTurn2(44);
			delay(800);
			driveSlow(-800, 80);
			delay(700);
			mogo.set_value(true);
			delay(700);
			CONVEYOR.move(127);
		 	delay(700);			
			driveTurn2(-96);
			delay(700);
			driveStraight2(2000);

    }else if(atn == 3){//redr
    		mogo.set_value(false);
			doinker.set_value(true);
			delay(1000);
			driveStraight2(-580);
			delay(700);
			driveTurn2(-44);
			delay(800);
			driveSlow(-800, 80);
			delay(700);
			mogo.set_value(true);
			delay(700);
			CONVEYOR.move(127);
		 	delay(700);			
			driveTurn2(96);
			delay(700);
			driveStraight2(2000);

    }else if(atn == 4){//bluel
    		mogo.set_value(false);
			doinker.set_value(true);
			delay(1000);
			driveStraight2(-580);
			delay(700);
			driveTurn2(-44);
			delay(800);
			driveSlow(-780 , 80);
			delay(700);
			mogo.set_value(true);
			delay(700);
			CONVEYOR.move(127);
		 	delay(700);			
			driveTurn2(-96);
			delay(700);
			driveStraight2(2000);

    }else if(atn == 5){
			mogo.set_value(false);
			doinker.set_value(true);
			delay(1000);
			driveStraight2(-580);
			delay(700);
			driveTurn2(-44);
			delay(800);
			driveSlow(-800, 80);
			delay(700);
			mogo.set_value(true);
			delay(700);
			CONVEYOR.move(127);
		 	delay(700);			
			driveTurn2(96);
			delay(700);
			driveStraight2(2000);
    }


    // driveTurn2(90);

    // driveStraight(1000);

    // driveTurn(45);

    // driveStraight2(6000  );


    // driveStraight2(2000);
    // driveTurn2(-90);
    // driveStraight2(1500);
    // driveTurn2(90);
    // driveStraight2(1500);
    // driveTurn2(180);
    // driveStraight2(1900);
}