#include "api.h"
#include "main.h"

#ifndef PIDH
#define PIDH

extern void driveStraight(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveTurnT(int target);
extern void driveStraight2(int target, int speed=100, int clampDistance=-1);
extern void driveStraightC(int target);

extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcR(double theta, double radius, int timeout);

extern void driveArcLF(double theta, double radius, int timeout);
extern void driveArcRF(double theta, double radius, int timeout);

extern double calcPID(double target, double input, int integralKi, int maxIntegral);
extern double calcPID2(double target, double input, int integralKi, int maxIntegral);

extern void setConstants(double kp, double ki, double kd);

extern void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRM, int voltageRB);

extern void LadyBrownMove();

extern void driveClamp (int target, int clampDistance);
extern void driveSlow (int target, int speed);

extern void ColorSort();
// extern void ColorSort2(int color);

extern void IntakeConveyor(int voltage);

extern void MOGO(bool mogo_toggle);

extern int time2;

extern int lbmove;





#define STRAIGHT_KP 1.9
#define STRAIGHT_KI 0
#define STRAIGHT_KD 9.8

#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5

#define TURN_KP 5
#define TURN_KI 0
#define TURN_KD 45

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define ARC_CORRECTION_KP 18

#define HEADING_KP 8
#define HEADING_KI 0
#define HEADING_KD 0

#define HEADING_INTEGRAL_KI 0
#define HEADING_MAX_INTEGRAL 0

#define ARC_HEADING_KP 4
#define ARC_HEADING_KI 0.1
#define ARC_HEADING_KD 12

#define ARC_HEADING_INTEGRAL_KI 0
#define ARC_HEADING_MAX_INTEGRAL 0

//tune for lady brown

//more vertical (maco 0/1 etc)
#define LADYBROWN_KP 0.01
#define LADYBROWN_KD 12000
#define LADYBROWN_KI 0.003

//more horizontal (macro 2/3)
#define LADYBROWN_KP2 0.012
#define LADYBROWN_KD2 12000
#define LADYBROWN_KI2 0.003

#define LADYBROWN_INTEGRAL_KI 0
#define LADYBROWN_MAX_INTEGRAL 0

//more vertical (maco 0/1 etc)
#define LADYBROWNHOLD_KP 1
#define LADYBROWNHOLD_KD 0
#define LADYBROWNHOLD_KI 0

//more horizontal (macro 2/3)
#define LADYBROWNHOLD_KP2 1
#define LADYBROWNHOLD_KD2 0
#define LADYBROWNHOLD_KI2 0

#define LADYBROWNHOLD_INTEGRAL_KI 0
#define LADYBROWNHOLD_MAX_INTEGRAL 0

#endif