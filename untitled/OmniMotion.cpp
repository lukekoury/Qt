#include <math.h>
#include <string.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <FEHMotor.h>
#include <FEHServo.h>

#define PI_180 0.0174532925
#define SPEEDCONSTANT 12.41 //inches per second TODO: Calibrate
#define ROTATIONCONSTANT 170.74 //degrees per second TODO: Calibrate
#define FRMT "%.1f "

using namespace std;

// GLOBAL VARIABLES ###########################################################
struct POS{
    float x;
    float y;
    float heading;
    float timestamp;
} Robot;

float cdsControl;

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
void waitForInitiation();
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
void rotateBy(float angle);
void rotateTo(float heading);


// OVERALL PROCEDURE ##########################################################

int main(){
    //RPS.InitializeTouchMenu();
    setupRun();
    startWithCds();

    for(int i=0; i<360; i+=10){
        moveBlind(i, 2, 1.0);
    }

    doc("Program halted");
    SD.CloseLog();
}

// STARTUP AND BOOKKEEPING ####################################################

void setupRun(){
    SD.OpenLog();
    calibrateCds();
    Robot = {0,0,0,0};
    updatePosition();
    waitForInitiation();
}

void calibrateCds(){
    //takes 1.2 seconds
    doc("Setting cds control");
    Sleep(1.0);
    float sum=0;
    int numCalibrations=20;
    for(int i=0; i<numCalibrations; i++){
        sum+=cds.Value();
        Sleep(10);
    }
    cdsControl = sum / numCalibrations;
    doc("cds control:", cdsControl);
}

void waitForInitiation(){
    float x,y;
    LCD.WriteRC("Touch to domniate.",6,2);
    while(LCD.Touch(&x,&y)) Sleep(1); //until untouched
    while(!LCD.Touch(&x,&y)) Sleep(1); //until pressed
    while(LCD.Touch(&x,&y)) Sleep(1); //until released
    LCD.Clear(SCARLET);
}

bool startWithCds(){
    float reading=0, oldReading=0, oldOldReading=0;
    float m=cdsControl-1.0;//threshhold in volts TODO:Calibrate
    int panicTime = TimeNow() + 5; //after 90 seconds, go anyway.
    while(reading>m || oldReading>m || oldOldReading>m){
        //move only after three in a row under threshold.
        oldOldReading=oldReading;
        oldReading=reading;
        reading=cds.Value();
        if(TimeNow()>panicTime){
            doc("Going without CdS.");
            LCD.Clear(BLACK);
            return false;
        }
        Sleep(5);
    }
    doc("Going with CdS.");
    LCD.Clear(BLACK);
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
    float x = RPS.X();
    float y = RPS.Y();
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
    float speed = moveAtAngleRelCourse(angle, 1.0);
    float distance = pythag(Robot.x,Robot.y, x,y);
    float halfTime = TimeNow()+(distance*0.5/*ADJUST*/)/speed;
    doc("movingTo1", x, y, distance, halfTime);
    while(TimeNow()<halfTime);
    if(!updatePosition){//in case RPS fails
        Robot.x += speed*distance*0.5*cos(PI_180*angle);
        Robot.y += speed*distance*0.5*sin(PI_180*angle);
        doc("CalcPosition", Robot.x, Robot.y);
    }
    distance = pythag(Robot.x, Robot.y, x, y);
    if(distance < 3/*ADJUST*/){
        moveBlindTo(x,y,.5);
        if(!updatePosition){//in case RPS fails
            Robot.x = x;
            Robot.y = y;
            doc("CalcPosition", Robot.x, Robot.y);
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
        rotationAngle= -from+(to-360);
    }else if(to<from){
        rotationAngle= (360-from)+to;
    }
    doc("Rot from/to/by:", from, to, rotationAngle);
    rotateBy(rotationAngle);
    //Maybe check and adjust?
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
    //TODO: MAKE SURE THIS IS HOW HEADINGS WORK
    return setVelocityComponents(cos(PI_180*heading), sin(PI_180*heading), 1.0);
}

float moveAtAngleRelCourse(float heading, float speedPercent){
    doc("RelCourse: ", heading, speedPercent);
    return moveAtAngleRelRobot(heading-Robot.heading, speedPercent);
}

void setRotation(float direction){
    if(fabs(direction)>1) direction/=fabs(direction);
    doc("Setting Rotation:", direction);
    setWheels(direction, direction, direction, direction);
}

void rotateBy(float angle){
    doc("Rotating by:", angle);
    //TODO: MAKE SURE THIS IS HOW HEADINGS WORK
    setRotation(angle>0? 1:-1);
    float timeEnd=TimeNow()+angle/ROTATIONCONSTANT;
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
