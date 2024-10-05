#include "api.h"
#include "main.h"
#include "robot.h"
#include "pid.h"

using namespace pros;
using namespace c; 
using namespace std;

double vKp;
double vKi;
double vKd;
float error; //the distance from the target
double prevPower;
double prevError;
int integral;
int derivative;
int time2;
double power;

double vKp2;
double vKi2;
double vKd2;
float error2; //the distance from the target
double prevPower2;
double prevError2;
int integral2;
int derivative2;
int time22;
double power2;

double vKp3;
double vKi3;
double vKd3;
float error3; //the distance from the target
double prevPower3;
double prevError3;
int integral3;
int derivative3;
int time23;
double power3;



void setConstants (double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
}

void resetEncoders () {
    LF.tare_position();
    LM.tare_position();
    LB.tare_position();
    RF.tare_position();
    RM.tare_position();
    RB.tare_position();
}

void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRM, int voltageRB) {
    LF.move(voltageLF);
    LM.move(voltageLM);
    LB.move(voltageLB);
    RF.move(voltageRF);
    RM.move(voltageRM);
    RB.move(voltageRB);
}

double calcPID(double target, double input, int integralKi, int maxIntegral){
    int integral;
    prevError = error;
    error = target-input;

    if(abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral>=0) {
        integral = min(integral, maxIntegral);
    } else {
        integral = max(integral, -maxIntegral);
    }

    derivative = error - prevError;

    power = (vKp * error) + (vKi*integral) + (vKd * derivative);
    return power;

}

double calcPID2(double target, double input, int integralKi, int maxIntegral){
    int integral2;
    prevError2 = error2;
    error2 = target-input;

    if(abs(error2) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2>=0) {
        integral2 = min(integral2, maxIntegral);
    } else {
        integral2 = max(integral2, -maxIntegral);
    }

    derivative2 = error2 - prevError2;

    power2 = (vKp * error2) + (vKi*integral2) + (vKd * derivative2);
    return power2;

}

double calcPID3(double target, double input, int integralKi, int maxIntegral){
    int integral3;
    prevError3 = error3;
    error3 = target-input;

    if(abs(error3) < integralKi) {
        integral3 += error3;
    } else {
        integral3 = 0;
    }

    if(integral3>=0) {
        integral3 = min(integral3, maxIntegral);
    } else {
        integral3 = max(integral3, -maxIntegral);
    }

    derivative3 = error3 - prevError3;

    power3 = (vKp * error3) + (vKi*integral3) + (vKd * derivative3);
    return power3;

}


void driveStraight(int target){
    int timeout = 10000;

    double x = 0;
    x = double(abs(target));
    timeout = (0.000000000000097017*pow(x,5)) + (-0.00000000055038 * pow(x,4))+ (0.00000106378 * pow(x,3)) + (-0.000841031 * pow(x,2)) + (0.591258 *x) + 311.616;
    imu.tare();

    double voltage;
    double encoderAvg;
    int count=0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    bool over = false;

    if(init_heading>180){
        init_heading = init_heading - 360;
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    while(true){

        ColorSort(RingColor);

        encoderAvg = (LF.get_position()+RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();

        if(position>180){
            position = position - 360;
        }

        if((init_heading<0) && (position>0)){
            if((position - init_heading)>=180){
                init_heading = init_heading + 360;
                position = imu.get_heading();
            }
        }else if ((init_heading>0)&&(position<0)){
            if((init_heading - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        

        if(voltage > 127){
            voltage = 127;
        } else if(voltage < -127){
            voltage = -127;
        }

        chasMove((voltage+heading_error), (voltage+heading_error), (voltage+heading_error), (voltage-heading_error), (voltage-heading_error), (voltage-heading_error));

        if(target>0){
            if((encoderAvg - (target - 500))>0){
                over = true;
            }
        }else{
            if(((target+500)-encoderAvg)>0){
                over = true;
            }
        }

        if(over||time2>timeout){
            break;
        }

        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }
        delay(10);
        time2+=10;
 
    }

    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveStraight2(int target){


    int timeout = 10000;

    double x = 0;
    x = double(abs(target));
    timeout = (0.000000000000097017*pow(x,5)) + (-0.00000000055038 * pow(x,4))+ (0.00000106378 * pow(x,3)) + (-0.000841031 * pow(x,2)) + (0.591258 *x) + 311.616;
    

    double voltage;
    double encoderAvg;
    int count=0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    bool over = false;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    if(init_heading > 180){
        init_heading = init_heading - 360;
    }

    while(true){

        ColorSort(RingColor);
        encoderAvg = (LF.get_position()+RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();

        if(position>180){
            position = position - 360;
        }

        if((init_heading < 0) && (position > 0)){
            if((position - init_heading) >=180){
                init_heading = init_heading+360;

            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading-position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants (HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if(voltage > 127){
            voltage = 127;
        } else if(voltage < -127){
            voltage = -127;
        }

        chasMove((voltage+heading_error), (voltage+heading_error), (voltage+heading_error), (voltage-heading_error), (voltage-heading_error), (voltage-heading_error));

        if(target>0){
            if((encoderAvg - (target-500))>0){
                over=true;
            }
        }else {
            if(((target+500)-encoderAvg)>0){
                over = true;
            }
        }

        if(over || time2>timeout){
            break;
        }

        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }
        delay(10);
        time2+=10;
 
    }

    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveClamp(int target, int clampDistance){
    int timeout = 10000;

    double x = 0;
    x = double(abs(target));
    timeout = (0.000000000000097017*pow(x,5)) + (-0.00000000055038 * pow(x,4))+ (0.00000106378 * pow(x,3)) + (-0.000841031 * pow(x,2)) + (0.591258 *x) + 311.616;
    

    double voltage;
    double encoderAvg;
    int count=0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    bool over = false;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    if(init_heading > 180){
        init_heading = init_heading - 360;
    }

    while(true){

        ColorSort(RingColor);

        encoderAvg = (LF.get_position()+RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();

        if(position>180){
            position = position - 360;
        }

        if((init_heading < 0) && (position > 0)){
            if((position - init_heading) >=180){
                init_heading = init_heading+360;

            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading-position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants (HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if(voltage > 127){
            voltage = 127;
        } else if(voltage < -127){
            voltage = -127;
        }

        if(abs(error) < clampDistance){
            mogo.set_value(true);
        }

        chasMove((voltage+heading_error), (voltage+heading_error), (voltage+heading_error), (voltage-heading_error), (voltage-heading_error), (voltage-heading_error));

        if(target>0){
            if((encoderAvg - (target-500))>0){
                over=true;
            }
        }else {
            if(((target+500)-encoderAvg)>0){
                over = true;
            }
        }

        if(over || time2>timeout){
            break;
        }

        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }
        delay(10);
        time2+=10;
 
    }



    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveSlow(int target, int speed){
    int timeout = 10000;

    double x = 0;
    x = double(abs(target));
    timeout = (0.000000000000097017*pow(x,5)) + (-0.00000000055038 * pow(x,4))+ (0.00000106378 * pow(x,3)) + (-0.000841031 * pow(x,2)) + (0.591258 *x) + 311.616;
    

    double voltage;
    double encoderAvg;
    int count=0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;
    bool over = false;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();

    if(init_heading > 180){
        init_heading = init_heading - 360;
    }

    while(true){

         ColorSort(RingColor);
        encoderAvg = (LF.get_position()+RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL); 

        double position = imu.get_heading();

        if(position>180){
            position = position - 360;
        }

        if((init_heading < 0) && (position > 0)){
            if((position - init_heading) >=180){
                init_heading = init_heading+360;

            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading-position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants (HEADING_KP, HEADING_KI, HEADING_KD);
        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if(voltage > 127 * double(speed)/100.0){
            voltage = 127 * double(speed)/100.0;
        } else if(voltage < -127 * double(speed)/100.0){
            voltage = -127 * double(speed)/100.0;
        }

        chasMove((voltage+heading_error), (voltage+heading_error), (voltage+heading_error), (voltage-heading_error), (voltage-heading_error), (voltage-heading_error));

        if(target>0){
            if((encoderAvg - (target-500))>0){
                over=true;
            }
        }else {
            if(((target+500)-encoderAvg)>0){
                over = true;
            }
        }

        if(over || time2>timeout){
            break;
        }

        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }
        delay(10);
        time2+=10;
 
    }

    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


void driveStraightC(int target){
    int timeout = 10000;

    double x = 0;
    x = double(abs(target));
    timeout = (0.000000000000097017*pow(x,5)) + (-0.00000000055038 * pow(x,4))+ (0.00000106378 * pow(x,3)) + (-0.000841031 * pow(x,2)) + (0.591258 *x) + 311.616;
    bool over = false;

    if(target> 0 ){
        target = target + 500;
    }else{
        target = target - 500;
    }

    double voltage;
    double encoderAvg;
    int count=0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0;
    time2 = 0;

    if(init_heading>180){
        init_heading = init_heading - 360;
    }

  
    resetEncoders();

    while(true){
         ColorSort(RingColor);
        encoderAvg = (LF.get_position()+RF.get_position()) / 2;
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        double position = imu.get_heading();

        if(position>180){
            position = position - 360;
        }

        if((init_heading<0) && (position>0)){
            if((position - init_heading)>= 180){
                init_heading = init_heading + 360;
                position = imu.get_heading();

            }
        }else if ((init_heading>0) && (position < 0)){
            if((init_heading - position)>=180){
                position = imu.get_heading();
            }
        }

        setConstants(HEADING_KP, HEADING_KI, HEADING_KD);

        heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

        if(voltage > 127){
            voltage = 127;
        } else if(voltage < -127){
            voltage = -127;
        }

        chasMove((voltage+heading_error), (voltage+heading_error), (voltage+heading_error), (voltage-heading_error), (voltage-heading_error), (voltage-heading_error));
        if(target>0){
            if ((encoderAvg - (target-500)) > 0){
                over = true;
            }
        }else {
            if(((target+500)-encoderAvg)>0){
                over = true;
            }
        }

        if (over || time2>timeout){
            break;
        }

        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }
        delay(10);
        time2+=10;
 
    }

    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveTurn(int target) {
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;

    int timeout = 10000;

    

    double variKD = 0;
    double x = 0;
    x = double(abs(target));
    variKD = (0.0000000090126*pow(x, 5)) + (-0.00000416039*pow(x,4)) + (0.000710426*pow(x,3)) + (-0.0547395*pow(x,2)) + (1.78519*x) + 29.4532;
    setConstants(TURN_KP, TURN_KI, variKD);

    timeout = (0.00000013999*pow(x,5)) + (-0.0000662053*pow(x,4)) + (0.0116874*pow(x,3)) + (-0.934041*pow(x,2)) + (35.0169*x) + 157.303;
    imu.tare_heading();

    while(true){
        position = imu.get_heading();

     ColorSort(RingColor);

        if(position>180){
            position = (position-360);
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target-position) <= 1.5) count++;
        if (count >= 20 || time2>timeout){
           break;
        }
        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }

        time2 += 10;
        delay(10);

    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
    
}

void driveTurn2(int target) {
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;

    int timeout = 10000;

    int turnv = 0;

    position = imu.get_heading();

    if(position > 180){
        position = (position - 360);
    }
    
    if((target < 0) && (position > 0)) {
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position);
        } else {
            turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0)&& (position <0)){
        if((target - position) >= 180){
            position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv=(abs(position)+target);
        }
    }else{
        turnv=abs(abs(position)-abs(target));
    }

    double variKD = 0;
    double x = 0;
    x = double(abs(turnv));
    variKD = (0.0000000090126*pow(x, 5)) + (-0.00000416039*pow(x,4)) + (0.000710426*pow(x,3)) + (-0.0547395*pow(x,2)) + (1.78519*x) + 29.4532;
    setConstants(TURN_KP, TURN_KI, variKD);

    timeout = (0.00000013999*pow(x,5)) + (-0.0000662053*pow(x,4)) + (0.0116874*pow(x,3)) + (-0.934041*pow(x,2)) + (35.0169*x) + 157.303;

    while(true){

    ColorSort(RingColor);
     position = imu.get_heading();

    if(position > 180){
        position = (position - 360);
    }
    
    if((target < 0) && (position > 0)) {
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position);
        } else {
            turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0)&& (position <0)){
        if((target - position) >= 180){
            position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv=(abs(position)+target);
        }
    }else{
        turnv=abs(abs(position)-abs(target));
    }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if (abs(target-position) <= 1.5) count++;
        if (count >= 20 || time2>timeout){
           break;
        }
        if (time2%50==0){
            con.print(0, 0, "Error: %f         ", float(error));
        }

        time2 += 10;
        delay(10);

    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
    
}

void driveArcL (double theta, double radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;
    time2 = 0;
    resetEncoders();

    ltarget = double((theta/360) * 2 * pi * radius);
    rtarget = double((theta/360) * 2 * pi * (radius+750));

    while (true){

         ColorSort(RingColor);

        double position = imu.get_heading();

        if(position > 180){
            position = position - 360;
        }

        if((init_heading<0)&&(position>0)){
            if((position - init_heading) >= 180){
                init_heading = init_heading +360;
                position = imu.get_heading();
            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading - position) >= 180){
                position = imu.get_heading();
            }
        }
        double encoderAvgL = (LF.get_position() + LB.get_position())/2;
        double encoderAvgR = (RF.get_position() + RB.get_position())/2;
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        }else if(voltageL < -127){
            voltageL = -127;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        }else if (voltageR < -127){
            voltageR = -127;
        }

        double leftcorrect = (encoderAvgL *360)/(2*pi*radius);
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((init_heading - leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);


        chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
       
        if ((abs(ltarget-encoderAvgL) <=4 ) && (abs(rtarget - encoderAvgR) <=4)) count++;
        if (count>=20 || time2 > timeout){
            break;
        }

        if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0,0, "ERROR: %f                    ", float(error));
        }else if (time2 % 50 == 0 & time2 % 100 != 0){
            con.print(1,0, "EncoderAvg: %f        ", float(encoderAvgL));

        }else if (time2 % 50 == 0){
            con.print(2,0, "Voltage: %f         ", float(voltageL));
        }
        
        time2 += 10;
        delay(10);
    }

}

void driveArcR (double theta, double radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    double ltarget = 0;
    double rtarget = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    int count = 0;
    time2 = 0;
    resetEncoders();

    ltarget = double((theta/360) * 2 * pi * (radius+750));
    rtarget = double((theta/360) * 2 * pi * radius);

    while (true){
         ColorSort(RingColor);

        double position = imu.get_heading();

        if(position > 180){
            position = position - 360;
        }

        if((init_heading<0)&&(position>0)){
            if((position - init_heading) >= 180){
                init_heading = init_heading +360;
                position = imu.get_heading();
            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading - position) >= 180){
                position = imu.get_heading();
            }
        }
        double encoderAvgL = (LF.get_position() + LB.get_position())/2;
        double encoderAvgR = (RF.get_position() + RB.get_position())/2;
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        }else if(voltageL < -127){
            voltageL = -127;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        }else if (voltageR < -127){
            voltageR = -127;
        }

        double rightcorrect = (encoderAvgR *360)/(2*pi*radius);
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((init_heading+rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        if ((abs(ltarget-encoderAvgL) <=4 ) && (abs(rtarget - encoderAvgR) <=4)) count++;
        if (count>=20 || time2 > timeout){
            break;
        }

        if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0,0, "ERROR: %f                    ", float(error));
        }else if (time2 % 50 == 0 & time2 % 100 != 0){
            con.print(1,0, "EncoderAvg: %f        ", float(encoderAvgL));

        }else if (time2 % 50 == 0){
            con.print(2,0, "Voltage: %f         ", float(voltageL));
        }
        
        time2 += 10;
        delay(10);
    }

}

void driveArcLF (double theta, double radius, int timeout){
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);


    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    bool over = false;
    int count = 0;
    time2 = 0;
    resetEncoders();

    if(init_heading>180){
        init_heading = init_heading - 360;
    }

    ltargetFinal = double((theta/360) * 2 * pi * radius);
    rtargetFinal = double((theta/360) * 2 * pi * (radius+750));
    theta = theta+45;
    ltarget = double((theta/360) * 2 * pi * radius);
    rtarget = double((theta/360) * 2 * pi * (radius+750));

    while (true){

         ColorSort(RingColor);

        double position = imu.get_heading();

        if(position > 180){
            position = position - 360;
        }

        if((init_heading<0)&&(position>0)){
            if((position - init_heading) >= 180){
                init_heading = init_heading +360;
                position = imu.get_heading();
            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading - position) >= 180){
                position = imu.get_heading();
            }
        }

        double encoderAvgL = (LF.get_position() + LB.get_position())/2;
        double encoderAvgR = (RF.get_position() + RB.get_position())/2; 
        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        }else if(voltageL < -127){
            voltageL = -127;
        }
 
        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        }else if (voltageR < -127){
            voltageR = -127;
        }

        double leftcorrect = (encoderAvgL *360)/(2*pi*radius);
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((init_heading-leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);
        

        chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
        
        if(theta > 0){
            if((encoderAvgL - ltargetFinal)>0){
                over = true;
            }
        }else {
            if((ltargetFinal - encoderAvgL)> 0){
                over = true;
            }
        }
        if (over || time2 > timeout){
            break;
        }

        if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0,0, "ERROR: %f                    ", float(error));
        }else if (time2 % 50 == 0 & time2 % 100 != 0){
            con.print(1,0, "EncoderAvg: %f        ", float(encoderAvgL));

        }else if (time2 % 50 == 0){
            con.print(2,0, "Voltage: %f         ", float(voltageL));
        }
        
        time2 += 10;
        delay(10);
    }

}

void driveArcRF (double theta, double radius, int timeout){
 

    double ltarget = 0;
    double rtarget = 0;
    double ltargetFinal = 0;
    double rtargetFinal = 0;
    double pi = 3.14159265359;
    double init_heading = imu.get_heading();
    bool over = false;
    int count = 0;
    time2 = 0;
    resetEncoders();

    ltargetFinal = double((theta/360) * 2 * pi * (radius+750));
    rtargetFinal = double((theta/360) * 2 * pi * radius);
    theta = theta+45;
    ltarget = double((theta/360) * 2 * pi * (radius+750));
    rtarget = double((theta/360) * 2 * pi * radius);

    while (true){

         ColorSort(RingColor);

        double position = imu.get_heading();

        if(position > 180){
            position = position - 360;
        }

        if((init_heading<0)&&(position>0)){
            if((position - init_heading) >= 180){
                init_heading = init_heading +360;
                position = imu.get_heading();
            }
        }else if ((init_heading > 0) && (position < 0)){
            if((init_heading - position) >= 180){
                position = imu.get_heading();
            }
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        double encoderAvgL = (LF.get_position() + LB.get_position())/2;
        double encoderAvgR = (RF.get_position() + RB.get_position())/2;
        int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageL > 127){
            voltageL = 127;
        }else if(voltageL < -127){
            voltageL = -127;
        }

        int voltageR = calcPID2(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if(voltageR > 127){
            voltageR = 127;
        }else if (voltageR < -127){
            voltageR = -127;
        }

        double rightcorrect = (encoderAvgR *360)/(2*pi*radius);
        setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
        int fix = calcPID3((init_heading+rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

        chasMove((voltageL - fix), (voltageL - fix), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
        
        if(theta>0){
            if((encoderAvgR - (rtargetFinal)) > 0){
                over = true;
            }
        }else{
            if(((rtargetFinal) - encoderAvgR) > 0){
                over = true;
            }
        }

        if (over || time2>timeout){
            break;
        }

        if(time2 % 50 == 0 && time2 % 100 != 0 && time2 % 150 != 0){
            con.print(0,0, "ERROR: %f                    ", float(error));
        }else if (time2 % 50 == 0 & time2 % 100 != 0){
            con.print(1,0, "EncoderAvg: %f        ", float(encoderAvgL));

        }else if (time2 % 50 == 0){
            con.print(2,0, "Voltage: %f         ", float(voltageL));
        }
        
        time2 += 10;
        delay(10);
    }

}

bool InitColor = false;
int ColorCount;
bool Backwards = false;
  
void ColorSort (int color){
    if(color==0){// blue rejection
        if(OpticalC.get_hue()<240 && OpticalC.get_hue()>180){
            InitColor = true; 
        }
  
        if(InitColor){

            if( Backwards == false){
                INTAKE.move(127);
            
                 if(INTAKE.get_position() > 500){
                Backwards = true;
                 }
            }else {
                INTAKE.move(-127);
                if(INTAKE.get_position() < 200){
                    Backwards = false;
                    InitColor = false; 
                }
            }
        }else {
            INTAKE.move(127);
            INTAKE.tare_position();
        }





    }else if(color==1){ // red rejection
        if(OpticalC.get_hue()>0 && OpticalC.get_hue()<40){
            InitColor = true;
        }
        if(InitColor){

            if( Backwards == false){
                INTAKE.move(127);
            
                 if(INTAKE.get_position() > 500){
                Backwards = true;
                 }
            }else {
                INTAKE.move(-127);
                if(INTAKE.get_position() < 200){
                    Backwards = false;
                    InitColor = false; 
                }
            }
        }else {
            INTAKE.move(127);
            INTAKE.tare_position();
        }
        }
}