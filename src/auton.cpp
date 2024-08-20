#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"

void autonomous() {
    driveStraight2(2000);
    driveTurn2(-90);
    driveStraight2(1500);
    driveTurn2(90);
    driveStraight2(1500);
    driveTurn2(180);
    driveStraight2(1900);
}