#include "api.h"
#include "robot.h"
#include "main.h"
#include "auton.h"
#include "odometry.h"
#include "pid.h"     

//Defines the port so this is the only part that  needs to be changed if the port changes
#define INTAKE_PORT 10
#define CONVEYOR_PORT 10 
#define LF_PORT 20
#define LM_PORT 2
#define LB_PORT 3
#define RF_PORT 5
#define RM_PORT 12
#define RB_PORT 1
#define IMU_PORT 4
#define OPTICAL_PORT 12

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor CONVEYOR (CONVEYOR_PORT, pros::E_MOTOR_GEARSET_18, true);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::Imu imu (IMU_PORT);

//pros::ADIDigitalIn selec ('H');

pros::ADIDigitalOut mogo ('H');

pros::ADIDigitalOut doinker ('C');

pros::Optical OpticalC (OPTICAL_PORT, 10);