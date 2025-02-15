#include "api.h"
#include "robot.h"
#include "main.h"
#include "auton.h"
#include "odometry.h"
#include "pid.h"     

//Defines the port so this is the only part that  needs to be changed if the port changes
#define INTAKE_PORT 15
#define CONVEYOR_PORT 15
#define LF_PORT 12 //X
#define LM_PORT 4 //X
#define LB_PORT 11 //X
#define RF_PORT 14 //X
#define RM_PORT 19 //X
#define RB_PORT 1 //X
#define IMU_PORT 8 //X
#define OPTICAL_PORT 12
#define LADYBROWN_PORT 13 //X
//#define LADYBROWN_PORT2 20
#define ROTO_PORT 7

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor CONVEYOR (CONVEYOR_PORT, pros::E_MOTOR_GEARSET_18, true);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor LadyBrown1 (LADYBROWN_PORT, pros::E_MOTOR_GEARSET_06, false);
//pros::Motor LadyBrown2 (LADYBROWN_PORT, pros::E_MOTOR_GEARSET_06, true);
//pros::Motor_Group LADYBROWN ({LadyBrown1, LadyBrown2});
pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::Rotation roto (ROTO_PORT);

pros::Imu imu (IMU_PORT);

pros::ADIDigitalIn selec ('C');

pros::ADIDigitalOut mogo ('A');

pros::ADIDigitalOut colorsort ('B');

pros::ADIDigitalOut doinker ('H');

pros::Optical OpticalC (OPTICAL_PORT, 10);