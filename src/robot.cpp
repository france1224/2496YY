#include "api.h"
#include "robot.h"
#include "main.h"
#include "auton.h"
#include "odometry.h"
#include "pid.h"     

//Defines the port so this is the only part that  needs to be changed if the port changes
#define INTAKE_PORT 20 //x
#define CONVEYOR_PORT 20 //x
#define LF_PORT 12 //x
#define LM_PORT 16 //x
#define LB_PORT 1 //x
#define RF_PORT 11 //x
#define RM_PORT 14//x
#define RB_PORT 2 //x
#define IMU_PORT 3 //x
#define OPTICAL_PORT 12
#define LADYBROWN_PORT 10 //x
#define LADYBROWN_PORT2 6//x
#define ROTO_PORT 5 //x

//defining the motors :D
pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor CONVEYOR (CONVEYOR_PORT, pros::E_MOTOR_GEARSET_18, false);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Motor LadyBrown1 (LADYBROWN_PORT, pros::E_MOTOR_GEARSET_36, true);
pros::Motor LadyBrown2 (LADYBROWN_PORT2, pros::E_MOTOR_GEARSET_36, false);
pros::Motor_Group LADYBROWN ({LadyBrown1, LadyBrown2});
pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::Rotation roto (ROTO_PORT);

pros::Imu imu (IMU_PORT);

//DigitalOut = piston
pros::ADIDigitalIn selec ('D');

pros::ADIDigitalOut mogo ('A');

pros::ADIDigitalOut colorsort ('B');

pros::ADIDigitalOut doinker ('C');

pros::Optical OpticalC (OPTICAL_PORT, 10);