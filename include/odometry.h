#include "api.h"
#include "main.h"

#ifndef ODOH
#define ODOH

#define FORWARD_OFFSET 0
#define IMU_THRESHOLD 0.0001

extern void odometry();
extern void setPosition(float xcoord, float ycoord, float heading);

extern void boomerang(double xTarget, double yTarget);


#endif