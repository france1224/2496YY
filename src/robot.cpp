#include "api.h"
#include "robot.h"
#include "main.h"

//Defines the port so this is the only part thT needs to be changed if the port changes
#define INTAKE_PORT 15
#define LF_PORT 4
#define LM_PORT 8
#define LB_PORT 2
#define RF_PORT 1
#define RM_PORT 6
#define RB_PORT 13
#define IMU_PORT 20

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, true);

pros::Controller con (pros::E_CONTROLLER_MASTER);

pros::Imu imu (IMU_PORT);

