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
    RingColor = 2;

    bool mogo_toggle2;

    driveSlow(-900, 100);

    mogo_toggle2 = !mogo_toggle2;

    mogo.set_value(mogo_toggle2);

    driveStraight(200);

    mogo_toggle2 = !mogo_toggle2;

    delay(200);

    mogo.set_value(mogo_toggle2);

    }else if(atn == 2){

    }else if(atn == 3){

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