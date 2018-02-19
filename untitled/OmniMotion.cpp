#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <string>
#include <sstream>
#include <FEHMotor.h>
using namespace std;

#define PI_180 0.0174532925
#define SPEEDCONSTANT 12.41 //TODO: Calibrate
#define ROTATIONCONSTANT 12.41 //TODO: Calibrate
#define BLANKDOC -123.456

float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);
void moveBlind(float angle, float distance);
void rotateBy(float angle);
void rotateTo(float heading);
void halt();
void moveTo(float x, float y);
void doc(string text, float a=BLANKDOC, float b=BLANKDOC, float c=BLANKDOC, float d=BLANKDOC);
float principal(float x);

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

bool updatePosition();
void doc(string text, float a, float b, float c, float d);
void setWheels(float fl, float fr, float bl, float br);
void setWheels(float fl, float fr, float bl, float br);

struct POS{
    float x;
    float y;
    float heading;
    int timestamp;
} RobotPosition;

int main(){
    RPS.InitializeTouchMenu();
    SD.OpenLog();
    updatePosition();
    moveBlind(0,36);
    moveBlind(90,36);
    moveBlind(180,36);
    moveBlind(360,36);
    SD.CloseLog();
}

void doc(string text, float a, float b, float c, float d){
    stringstream s;
    s<<text;
    if(a!=BLANKDOC) s<<a<<" ";
    if(b!=BLANKDOC) s<<b<<" ";
    if(c!=BLANKDOC) s<<c<<" ";
    if(d!=BLANKDOC) s<<d<<" ";
    int n = s.str().length();
    char char_array[n+1];
    strcpy(char_array,s.str().c_str());
    LCD.WriteLine(char_array);
    SD.Printf(char_array);
}

bool updatePosition(){
    float x = RPS.X();
    float y = RPS.Y();
    float heading = RPS.Heading();
    if(fabs(heading-(-1))>.001){
        if(RobotPosition.x == x && RobotPosition.y == y && RobotPosition.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
        } else {
            RobotPosition.x = x;
            RobotPosition.y = y;
            RobotPosition.heading = heading;
            RobotPosition.timestamp=TimeNow();
            doc("Position: ", x,y,heading, RobotPosition.timestamp);
            return true;
        }
    }
    doc("Position Update Failed: ",x,y,heading);
    return false;
}

void setWheels(float fl, float fr, float bl, float br){
    //Sets wheels to given speeds
    //TODO: include calibrations for wheels
    motorFL.SetPercent(100.0*fl);
    motorFR.SetPercent(100.0*fr);
    motorBL.SetPercent(100.0*bl);
    motorBR.SetPercent(100.0*br);
}

float setVelocityComponents(float right, float forward, float speedPercent){
    //arguments between -1 and 1
    //Ex: setVelocityComponents(-1, 1, 0.75);
    //will set robot going forward and to the left at 75% Speed
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

float moveAtAngleRelRobot(float heading, float speedPercent){
    doc("RelRobot: ", heading, speedPercent);
    //TODO: MAKE SURE THIS IS HOW HEADINGS WORK
    return setVelocityComponents(cos(PI_180*heading), sin(PI_180*heading), 1.0);
}

float moveAtAngleRelCourse(float heading, float speedPercent){
    doc("RelCourse: ", heading, speedPercent);
    return moveAtAngleRelRobot(heading-RobotPosition.heading, speedPercent);
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
    if(!updatePosition) RobotPosition.heading=principal(RobotPosition.heading+angle);
    doc("Rotation Finished.");
}

float principal(float x){
    while(x>=360)x-=360;
    while(x<0)x+=360;
    return x;
}

void rotateTo(float heading){
    updatePosition();
    float to = principal(heading);
    float from = principal(RobotPosition.heading);
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
        RobotPosition.x += speed*distance*cos(PI_180*angle);
        RobotPosition.y += speed*distance*sin(PI_180*angle);
        doc("CalcPosition", RobotPosition.x, RobotPosition.y);
    }
    doc("BlindMove finished");
}

void moveTo1(float x, float y){
    //ALGORITHM 1
    updatePosition();
    float angle=(1/PI_180)*atan2(y-RobotPosition.x, x-RobotPosition.y);
    float speed = moveAtAngleRelCourse(angle, 1.0);
    float distance = sqrt(pow(y-RobotPosition.y,2)+pow(x-RobotPosition.x,2));
    float halfTime = TimeNow()+(distance*0.5/*ADJUST*/)/speed;
    while(TimeNow()<halfTime);
    if(!updatePosition){//in case RPS fails
        RobotPosition.x += speed*distance*0.5*cos(PI_180*angle);
        RobotPosition.y += speed*distance*0.5*sin(PI_180*angle);
        doc("CalcPosition", RobotPosition.x, RobotPosition.y);
    }
    distance = sqrt(pow(y-RobotPosition.y,2)+pow(x-RobotPosition.x,2));
    if(distance < 3/*ADJUST*/){
        moveBlind( (1/PI_180)*atan2(y-RobotPosition.y, x-RobotPosition.x), distance);
        if(!updatePosition){//in case RPS fails
            RobotPosition.x = x;
            RobotPosition.y = y;
            doc("CalcPosition", RobotPosition.x, RobotPosition.y);
        }
    } else {
        moveTo1(x,y);
    }
}

void moveTo2(float x, float y){
    //ALGORITHM 2
    updatePosition();
    float speed = moveAtAngleRelCourse(atan2(y-RobotPosition.x, x-RobotPosition.y), 1.0);
    float distance = sqrt(pow(y-RobotPosition.y,2)+pow(x-RobotPosition.x,2));
    float endTime = TimeNow()+(distance-1)/speed;
    while(TimeNow()<endTime);
    updatePosition();
    distance = sqrt(pow(y-RobotPosition.y,2)+pow(x-RobotPosition.x,2));
    moveBlind(atan2(y-RobotPosition.x, x-RobotPosition.y), distance);
}

void halt(){
    doc("Halting");
    setWheels(0,0,0,0);
}
