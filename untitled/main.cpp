#include <math.h>
#include <string.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHBuzzer.h>

#define PI_180 0.0174532925
#define SPEEDCONSTANT 12.41 //inches per second TODO: Calibrate
#define ROTATIONCONSTANT 170.74 //degrees per second TODO: Calibrate
#define FRMT "%.2f "

using namespace std;

// GLOBAL VARIABLES ###########################################################
struct POS{
    float x;
    float y;
    float heading;
    float timestamp;
} Robot;

float cdsControl;
float startX, startY;

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

AnalogInputPin cds(FEHIO::P0_0);//TODO: DECIDE PINS!!!
FEHServo crankyBoi(FEHServo::Servo0);
FEHServo forkLift(FEHServo::Servo1);

// FUNCTION PROTOTYPES ########################################################
void doc(const char *text);
void doc(const char *text, float a);
void doc(const char *text, float a, float b);
void doc(const char *text, float a, float b, float c);
void doc(const char *text, float a, float b, float c, float d);
bool startWithCds();
void setupRun();
void calibrateCds();
void calibrateRPS();
void waitForTouch();
bool updatePosition();

void setWheels(float fl, float fr, float bl, float br);
void halt();

float setVelocityComponents(float right, float forward, float speedPercent);
float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);

float principal(float x);
float arg(float x1,float y1,float x2,float y2);
float pythag(float x1,float y1,float x2,float y2);

void moveBlind(float angle, float distance, float speedPercent);
void moveBlindTo(float x, float y, float speedPercent);
void moveTo1(float x, float y);
void moveTo2(float x, float y);
void rotateBy(float angle, float speedPercent);
void rotateTo(float heading);


// OVERALL PROCEDURE ##########################################################

int main(){
    //RPS.InitializeTouchMenu();
    setupRun();
    LCD.WriteRC("Touch to domniate.",6,4);
    waitForTouch();
    startWithCds();

    //

    moveTo1(-12,-12);

    //

    doc("Program halted");
    SD.CloseLog();
}

// STARTUP AND BOOKKEEPING ####################################################

void setupRun(){
    SD.OpenLog();
    calibrateRPS();
    calibrateCds();
}

void calibrateCds(){
    //takes 1.2 seconds
    LCD.WriteRC("Touch to calubrate CdS.",6,4);
    waitForTouch();

    float sum=0;
    int numCalibrations=20;
    for(int i=0; i<numCalibrations; i++){
        sum+=cds.Value();
        Sleep(10);
    }
    cdsControl = sum / numCalibrations;
    doc("cds control:", cdsControl);
}
void cdsMeterMode(){
    while(true){
        float sum=0;
        int numCalibrations=20;
        for(int i=0; i<numCalibrations; i++){
            sum+=cds.Value();
            Sleep(10);
        }
        cdsControl = sum / numCalibrations;
        LCD.Clear();
        LCD.WriteLine(cdsControl);
    }
}
void calibrateRPS(){
    startX=RPS.X();
    startY=RPS.Y();
    updatePosition();
}
void waitForTouch(){
    float x,y;
    while(LCD.Touch(&x,&y)) Sleep(1); //until untouched
    while(!LCD.Touch(&x,&y)) Sleep(1); //until pressed
    while(LCD.Touch(&x,&y)) Sleep(1); //until released
    Buzzer.Beep();
}
bool startWithCds(){
    float readings[20]={4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
    float sum=4*20;
    float m=cdsControl*0.8;//threshhold in volts TODO:Calibrate
    int panicTime = TimeNow() + 60; //after 60 seconds, go anyway.
    while(sum/20>m){
        if(TimeNow()>panicTime){
            doc("Going without CdS.");
            return false;
        }
        //move only after average of 20 under threshold.
        sum-=readings[0];
        for(int i=0;i<19;i++)
            readings[i]=readings[i+1];
        readings[19]=cds.Value();
        sum+=readings[19];
        Sleep(5);
    }
    doc("Going with CdS.", sum/20, m);
    return true;
}

void doc(const char *text){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text);
    SD.Printf("\n"); LCD.Write('\n');
}
void doc(const char *text, float a){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text);
    SD.Printf(FRMT, a); LCD.Write(a);
    SD.Printf("\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text);
    SD.Printf(FRMT, a); LCD.Write(a);
    SD.Printf(FRMT, b); LCD.Write(b);
    SD.Printf("\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b, float c){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text);
    SD.Printf(FRMT, a); LCD.Write(a);
    SD.Printf(FRMT, b); LCD.Write(b);
    SD.Printf(FRMT, c); LCD.Write(c);
    SD.Printf("\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b, float c, float d){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text);
    SD.Printf(FRMT, a); LCD.Write(a);
    SD.Printf(FRMT, b); LCD.Write(b);
    SD.Printf(FRMT, c); LCD.Write(c);
    SD.Printf(FRMT, d); LCD.Write(d);
    SD.Printf("\n"); LCD.Write('\n');
}

// MOVEMENT AND NAVIGATION ###################################################

bool updatePosition(){
    //THis is the only place we need to worry about how RPS works.
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(fabs(heading)>0){
        if(Robot.x == x && Robot.y == y && Robot.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
        } else {
            Robot.x = x;
            Robot.y = y;
            Robot.heading = heading;
            Robot.timestamp=TimeNow();
            doc("Position: ", x,y,heading, Robot.timestamp);
            return true;
        }
    }
    doc("Position Update Failed: ",x,y,heading);
    return false;
}

void moveTo1(float x, float y){
    //ALGORITHM 1
    updatePosition();
    float angle=arg(Robot.x, Robot.y, x, y);
    float distance = pythag(Robot.x,Robot.y, x, y);
    float speed = moveAtAngleRelCourse(angle, 1.0);

    //Go halfway
    float halfTime = TimeNow()+(distance*0.5/*ADJUST*/)/speed;
    doc("movingTo1", x, y);
    while(TimeNow()<halfTime);
    if(!updatePosition){//in case RPS fails
        Robot.x += speed*distance*0.5*cos(PI_180*angle);
        Robot.y += speed*distance*0.5*sin(PI_180*angle);
        doc("CalcHalfPos", Robot.x, Robot.y);
    }

    //Go the rest of the way
    distance = pythag(Robot.x, Robot.y, x, y);
    if(distance < 3/*ADJUST*/){
        moveBlindTo(x,y,.3);/*ADJUST*/
        if(!updatePosition()){//in case RPS fails
            Robot.x = x;
            Robot.y = y;
            doc("CalcPos", Robot.x, Robot.y);
        }
    } else {
        moveTo1(x,y);
    }
}

void moveTo2(float x, float y){
    //ALGORITHM 2
    updatePosition();
    float speed = moveAtAngleRelCourse(arg(Robot.x, Robot.y, x, y), 1.0);
    float distance = pythag(Robot.x, Robot.y, x, y);
    float endTime = TimeNow()+(distance)/speed;
    while(TimeNow()<endTime);
    if(updatePosition()){
        moveBlindTo(x,y,.25);
    } else {
        Robot.x = x;
        Robot.y = y;
    }
}

void rotateTo(float heading){
    updatePosition();
    float to = principal(heading);
    float from = principal(Robot.heading);
    float rotationAngle=0;
    if(fabs(to-from)<180){
        rotationAngle=to-from;
    }else if(to>from){
        rotationAngle=-from+(to-360);
    }else if(to<from){
        rotationAngle=(360-from)+to;
    }
    doc("Rot from/to/by:", from, to, rotationAngle);
    float angleSpeed=1;
    if(fabs(rotationAngle)<90) angleSpeed=fabs(rotationAngle/90);
    rotateBy(rotationAngle, angleSpeed);

    //Check and adjust until within 5 degrees
    if(updatePosition()){
        float newAngle = principal(heading-Robot.heading);
        if(newAngle>5 && newAngle<355){
            rotateTo(heading);
        }
    } else {
        Robot.heading=heading;
    }

}

void moveBlind(float angle, float distance, float speedPercent){
    doc("BlindMove", angle, distance);
    float speed = moveAtAngleRelCourse(angle, speedPercent);
    float endTime = TimeNow()+distance/speed;
    while(TimeNow()<endTime);
    if(!updatePosition){//in case RPS fails
        Robot.x += speed*distance*cos(PI_180*angle);
        Robot.y += speed*distance*sin(PI_180*angle);
        doc("CalcPosition", Robot.x, Robot.y);
    }
    doc("BlindMove finished");
    halt();
}

void moveBlindTo(float x, float y, float speedPercent){
    float angle = (1/PI_180)*atan2(y-Robot.y, x-Robot.x);
    float distance = sqrt(pow(y-Robot.y,2)+pow(x-Robot.x,2));
    moveBlind(angle, distance, speedPercent);
}

float setVelocityComponents(float right, float forward, float speedPercent){
    //arguments between -1 and 1
    //Ex: setVelocityComponents(-1, 1, 0.75);
    //will set Robot going forward and to the left at 75% Speed
    //returns speed
    float fl=-forward-right;
    float fr=+forward-right;
    float bl=-forward+right;
    float br=+forward+right;
    //Scale back so speed is maximum and no overflow:
    float m=fmax( fmax( fabs(fl),fabs(fr) ), fmax( fabs(bl),fabs(br) ) );
    if(fabs(m)>.001){
        fl/=m; fr/=m; br/=m; bl/=m;
    }
    if(fabs(speedPercent)>.001){
        fl*=speedPercent; fr*=speedPercent; br*=speedPercent; bl*=speedPercent;
    }
    setWheels(fl,fr,bl,br);
    //pythagorean between two components of motion to return speed
    float speed = SPEEDCONSTANT * sqrt(fl*fl+fr*fr);
    doc("Wheels set: ",fl,fr);
    doc("Velocity:", right, forward, speed);
    return speed;
}

void halt(){
    doc("Halting");
    setWheels(0,0,0,0);
}

void setWheels(float fl, float fr, float bl, float br){
    //Sets wheels to given speeds
    //TODO: include calibrations for wheels
    motorFL.SetPercent(100.0*fl);
    motorFR.SetPercent(100.0*fr);
    motorBL.SetPercent(100.0*bl);
    motorBR.SetPercent(100.0*br);
}

float moveAtAngleRelRobot(float heading, float speedPercent){
    doc("RelRobot: ", heading, speedPercent);
    return setVelocityComponents(cos(PI_180*heading), sin(PI_180*heading), 1.0);
}

float moveAtAngleRelCourse(float heading, float speedPercent){
    doc("RelCourse: ", heading, speedPercent);
    return moveAtAngleRelRobot(heading-Robot.heading, speedPercent);
}

void setRotation(float direction){
    if(fabs(direction)>1) direction=(direction>0? 1:-1);
    doc("Setting Rotation:", direction);
    setWheels(direction, direction, direction, direction);
}

void rotateBy(float angle, float speedPercent){
    doc("Rotating by:", angle);
    setRotation(speedPercent*(angle>0? 1:-1));
    float timeEnd=TimeNow()+angle/(speedPercent*ROTATIONCONSTANT);
    while(TimeNow()<timeEnd);
    halt();
    if(!updatePosition) Robot.heading=principal(Robot.heading+angle);
    doc("Rotation Finished.");
}


// MATH FUNCTIONS WITHOUT SIDE EFFECTS ##########################################

float principal(float x){
    while(x>=360)x-=360;
    while(x<0)x+=360;
    return x;
}

float arg(float x1, float y1, float x2, float y2){
    //returns the angle from (x1,y1) to (x2,y2)
    return atan2(y2-y1,x2-x1)/(PI_180);
}

float pythag(float x1, float y1, float x2, float y2){
    //returns the distance from (x1,y1) to (x2,y2)
    return sqrt(pow(y2-y1,2)+pow(x2-x1,2));
}
