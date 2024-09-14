#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odometry.h"

using namespace pros;
using namespace std;

double absoluteAngleToTarget = 0;
double hypot2 = 0;

double const HEADING_CUTOFF = 100;

float deltaX;
float deltaY;

float startingX;
float startingY;
float startingHeading;

float r0;
float r1;

float delta_left_encoder_pos;
float delta_right_encoder_pos;
float delta_center_encoder_pos;

float prev_left_encoder_pos;
float prev_right_encoder_pos;
float prev_center_encoder_pos;

float left_encoder_pos;
float right_encoder_pos;
float center_encoder_pos;

float localX;
float localY;

float phi;

float prev_imu_pos;
float imu_pos;

float x_pos;
float y_pos;

float pi = 3.14159265359;

int odo_time = 0;

void setPosition(float xcoord, float ycoord, float heading){
    startingX = xcoord;
    startingY = ycoord;
    startingHeading = heading;

    x_pos = startingX;
    y_pos = startingY;
}


void odometry (){
    prev_imu_pos = imu_pos;
    imu_pos = imu.get_rotation() + startingHeading;

    prev_left_encoder_pos = left_encoder_pos;
    prev_right_encoder_pos = right_encoder_pos;
    prev_center_encoder_pos = center_encoder_pos;

    left_encoder_pos = LF.get_position();
    right_encoder_pos = RB.get_position();
    center_encoder_pos = 0;

    delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
    delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
    delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

    phi = imu_pos - prev_imu_pos;

    r0 = ((delta_left_encoder_pos + delta_right_encoder_pos)/2)/phi;
    r1 = delta_center_encoder_pos/phi;

    if(abs(phi)<IMU_THRESHOLD){
        localX = (delta_left_encoder_pos + delta_right_encoder_pos)/2;
        localY = delta_center_encoder_pos - FORWARD_OFFSET * ((pi*phi)/180);
    }else{
        localX = r0*sin((pi*phi)/180)-r1*(1-cos((pi*phi)/180));
        localY = r1*sin((pi*phi)/180)+r0*(1-cos((pi*phi)/180));
    }

    deltaY = localX * cos((pi*imu_pos)/180) - localY*sin((pi*imu_pos)/180);
    deltaX = localX * sin((pi*imu_pos)/180) +localY * cos((pi*imu_pos)/180);

    x_pos += deltaX;
    y_pos += deltaY;

    if(odo_time % 50 == 0 && odo_time % 100 != 0 &&  odo_time % 150 != 0){
        con.print(0,0, "x_pos: %f            ", float(x_pos));

    }else if (odo_time % 100 == 0 && odo_time % 150 != 0){
        con.print(1,0, "y_pos: %f            ", float(y_pos));

    }else if (odo_time%150 == 0){
        con.print(2,0, "angle: %f            ", float(absoluteAngleToTarget));
    }

    odo_time += 10;

}

void boomerang(double xTarget, double yTarget){
    double hypot = 0;
    double voltage = 0;
    double heading_correction = 0;
    int turnv = 0;
    int btime = 0;
    int timeout = 30000;
    int count = 0;

    while(true){
        odometry ();
        hypot = sqrt(pow((x_pos - xTarget),2) + pow((y_pos - yTarget), 2));
        absoluteAngleToTarget = (180/pi)*(atan2((xTarget - x_pos), (yTarget - y_pos)));
        
    //    if(absoluteAngleToTarget>0){
    //        absoluteAngleToTarget -= 360;
    //    }

        if (absoluteAngleToTarget > 180){
            absoluteAngleToTarget = absoluteAngleToTarget - 360;
        }

        double position = imu.get_heading()+startingHeading;

        while (position>180){
            position = position - 360;
        }

        if((absoluteAngleToTarget > 0) && (position <0)){
            turnv = absoluteAngleToTarget - position;
        }else if ((absoluteAngleToTarget < 0) && (position > 0)){
            turnv = position - absoluteAngleToTarget;
        }else {
            turnv = abs(abs(absoluteAngleToTarget) - abs(position));
        }   

        // if((absoluteAngleToTarget<0) && (position>0)){
        //     if((position - absoluteAngleToTarget)>=180){
        //         absoluteAngleToTarget = absoluteAngleToTarget + 360;
        //         position = imu.get_heading()+startingHeading;
        //         turnv = (absoluteAngleToTarget - position);
        //     }else {
        //         turnv = (abs(position) + abs(absoluteAngleToTarget));
        //     }
        // }else if ((absoluteAngleToTarget > 0) && (position < 0)){

        //     if((absoluteAngleToTarget - position)>= 180){
        //         position = imu.get_heading() + startingHeading;
        //         turnv = abs(abs(position) - abs(absoluteAngleToTarget));
        //     }else {
        //         turnv = (abs(position) + absoluteAngleToTarget);
        //     }
        // }else{
        //     turnv = abs(abs(position) - abs(absoluteAngleToTarget));
        // }

        if(abs(turnv)>90){
            absoluteAngleToTarget = absoluteAngleToTarget + 180;
            hypot = -hypot;
        }

        if(absoluteAngleToTarget >= 359){
            absoluteAngleToTarget = absoluteAngleToTarget - 360;
        }
        // while(position>180){
        //     position = position - 360;
        // }

        if((absoluteAngleToTarget<0) && (position > 0)){
            if((position - absoluteAngleToTarget) >= 180){
                absoluteAngleToTarget = absoluteAngleToTarget + 360;
                position = imu.get_heading();
            }
        }else if ((absoluteAngleToTarget > 0) && (position<0)){
            if((absoluteAngleToTarget - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(TURN_KP, TURN_KI, TURN_KD);
        heading_correction = calcPID(absoluteAngleToTarget, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
            voltage = -calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);


        if (voltage>17){
            voltage = 17;
        }else if(voltage<-17){
            voltage = -17;
        }


        if(heading_correction>20){
            heading_correction = 20;
        }else if(heading_correction<-20){
            heading_correction = -20;
        }

        if(abs(hypot)< HEADING_CUTOFF){
            heading_correction = 0;
        }
        chasMove((voltage + heading_correction),(voltage + heading_correction),(voltage + heading_correction), (voltage - heading_correction),(voltage - heading_correction),(voltage - heading_correction));
        if(abs(hypot)<15) count++;
        if((count> 20)||(btime>timeout)){
            //break;
        }


        hypot2 = hypot;
        btime += 10;
        delay(10);
    }
}