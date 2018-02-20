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
#define BLANKDOC -123.456

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

void moveBlind(float angle, float distance);
void moveTo1(float x, float y);
void moveTo2(float x, float y);
void rotateBy(float angle);
void rotateTo(float heading);


// OVERALL PROCEDURE ##########################################################
int main(){
    setupRun();
    waitForInitiation();
    startWithCds();

    moveBlind(0,36);
    moveBlind(90,36);
    moveBlind(180,36);
    moveBlind(360,36);

    SD.CloseLog();
}

// STARTUP AND BOOKKEEPING ####################################################

void setupRun(){
    RPS.InitializeTouchMenu();
    SD.OpenLog();
    calibrateCds();
    updatePosition();
}

void calibrateCds(){
    doc("Setting cds control");
    Sleep(2.0);
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
    LCD.Clear(BLACK);
    while(LCD.Touch(&x,&y)) Sleep(1); //until untouched
    while(!LCD.Touch(&x,&y)) Sleep(1); //until pressed
    while(LCD.Touch(&x,&y)) Sleep(1); //until released
    LCD.Clear(LIME);
}

bool startWithCds(){
    float reading=0, oldReading=0, oldOldReading=0;
    float m=cdsControl-1.0;//threshhold in volts TODO:Calibrate
    int panicTime = TimeNow() + 90; //after 90 seconds, go anyway.
    while(reading>m || oldReading>m || oldOldReading>m){
        //move only after three in a row under threshold.
        oldOldReading=oldReading;
        oldReading=reading;
        reading=cds.Value();
        if(TimeNow()>panicTime){
            doc("Going without CdS.");
            return false;
        }
        Sleep(5);
    }
    doc("Going with CdS.");
    return true;
}
void doc(const char *text){
    char formatString[80]="%3.3f %s\n";
    LCD.Write(TimeNow());
    LCD.Write(text);
    LCD.Write('\n');
    SD.Printf(formatString, TimeNow(), text);
}
void doc(const char *text, float a){
    char formatString[]="%3.3f %s %3.1f\n";
    LCD.Write(TimeNow());
    LCD.Write(text);
    LCD.Write(a);
    LCD.Write('\n');
    SD.Printf(formatString, TimeNow(), text, a);
}
void doc(const char *text, float a, float b){
    char formatString[]="%3.3f %s %3.1f %3.1f\n";
    LCD.Write(TimeNow());
    LCD.Write(text);
    LCD.Write(a);
    LCD.Write(b);
    LCD.Write('\n');
    SD.Printf(formatString, TimeNow(), text, a, b);
}
void doc(const char *text, float a, float b, float c){
    char formatString[]="%3.3f %s %3.1f %3.1f %3.1f\n";
    LCD.Write(TimeNow());
    LCD.Write(text);
    LCD.Write(a);
    LCD.Write(b);
    LCD.Write(c);
    LCD.Write('\n');
    SD.Printf(formatString, TimeNow(), text, a, b, c);
}
void doc(const char *text, float a, float b, float c, float d){
    char formatString[]= "%3.3f %s %3.1f %3.1f %3.1f %3.1f\n";
    LCD.Write(TimeNow());
    LCD.Write(text);
    LCD.Write(a);
    LCD.Write(b);
    LCD.Write(c);
    LCD.Write(d);
    LCD.Write('\n');
    SD.Printf(formatString, TimeNow(), text, a,b,c,d);
}

// MOVEMENT AND NAVIGATION ###################################################
bool updatePosition(){
    float x = RPS.X();
    float y = RPS.Y();
    float heading = RPS.Heading();
    if(fabs(heading-(-1))>.001){
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
    float angle=(1/PI_180)*atan2(y-Robot.x, x-Robot.y);
    float speed = moveAtAngleRelCourse(angle, 1.0);
    float distance = sqrt(pow(y-Robot.y,2)+pow(x-Robot.x,2));
    float halfTime = TimeNow()+(distance*0.5/*ADJUST*/)/speed;
    doc("movingTo1", x, y, distance, halfTime);
    while(TimeNow()<halfTime);
    if(!updatePosition){//in case RPS fails
        Robot.x += speed*distance*0.5*cos(PI_180*angle);
        Robot.y += speed*distance*0.5*sin(PI_180*angle);
        doc("CalcPosition", Robot.x, Robot.y);
    }
    distance = sqrt(pow(y-Robot.y,2)+pow(x-Robot.x,2));
    if(distance < 3/*ADJUST*/){
        moveBlind( (1/PI_180)*atan2(y-Robot.y, x-Robot.x), distance);
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
    float speed = moveAtAngleRelCourse(atan2(y-Robot.x, x-Robot.y), 1.0);
    float distance = sqrt(pow(y-Robot.y,2)+pow(x-Robot.x,2));
    float endTime = TimeNow()+(distance)/speed;
    while(TimeNow()<endTime);
    if(!updatePosition()){
        Robot.x = x;
        Robot.y = y;
    }
    distance = sqrt(pow(y-Robot.y,2)+pow(x-Robot.x,2));
    moveBlind(atan2(y-Robot.x, x-Robot.y), distance);
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

void moveBlind(float angle, float distance){
    doc("BlindMove", angle, distance);
    float speed = moveAtAngleRelCourse(angle, 1.0);
    float endTime = TimeNow()+distance/speed;
    while(TimeNow()<endTime);
    if(!updatePosition){//in case RPS fails
        Robot.x += speed*distance*cos(PI_180*angle);
        Robot.y += speed*distance*sin(PI_180*angle);
        doc("CalcPosition", Robot.x, Robot.y);
    }
    doc("BlindMove finished");
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

float principal(float x){
    while(x>=360)x-=360;
    while(x<0)x+=360;
    return x;
}
