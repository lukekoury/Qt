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
#include <FEHBattery.h>

#define PI_180 0.0174532925
#define SPEEDCONSTANT 13.9 //inches per second
#define ROTATIONCONSTANT 186.7 //degrees per second
#define FRMT "%.2f "
#define OFF 0
#define REDLIGHT 1
#define BLUELIGHT 2

using namespace std;

// GLOBAL VARIABLES ###########################################################
struct POS{
    float x;
    float y;
    float heading;
    float timestamp; //when this was last updated
} Robot;

float cdsControl, redControl;
float startX, startY;

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

AnalogInputPin cds(FEHIO::P0_1);//TODO: DECIDE PINS
AnalogInputPin cdsRed(FEHIO::P0_0);
FEHServo crankyBoi(FEHServo::Servo0);
FEHServo forkLift(FEHServo::Servo1);

// FUNCTION PROTOTYPES ########################################################
void doc(const char *text);
void doc(const char *text, float a);
void doc(const char *text, float a, float b);
void doc(const char *text, float a, float b, float c);
void doc(const char *text, float a, float b, float c, float d);
void setupRun();
bool startWithCds();
void calibrateCds();
void calibrateRPS();
int getColor();
void meterMode();
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
float deltaAngle(float from, float to);

void moveBlind(float angle, float distance, float speedPercent);
void moveBlindTo(float x, float y, float speedPercent);
void moveComponents(float x, float y, float speedPercent);
void moveTo1(float x, float y);
void moveTo2(float x, float y);
void rotateBy(float angle, float speedPercent);
void rotateTo(float heading);

// OVERALL PROCEDURE ##########################################################

int main(){
    setupRun();

    //To debug color detector:
    //while(true)getColor();

    /////////////////////

    //Performance Test 2

    moveComponents(0,-5,1); //exit box
    moveBlindTo(8.75,-8.5,1); //diagonal to light
    int color=getColor();

    //idk just in case go in a square spiral
        float d=1; //length of ssegments in spiral
        while(color==OFF){
            for(int i=0;i<360;i+=90){ //cardinal directions
                for(int j=0; j<d; j++){ //length
                    moveBlind(i,1,.3);
                    color=getColor();
                    if(color!=OFF) break;
                }
                if(color!=OFF) break;
                d+=0.5;
            }
        }
    //We must've found the light, so we are here:
    Robot.x=8.75; Robot.y=-8.5;
    switch(color){
        case REDLIGHT:
            LCD.Clear(RED);
            moveComponents(1.8,0,.3); //left to button
        break;
        case BLUELIGHT:
            LCD.Clear(BLUE);
            moveComponents(-1.45,0,.3); //right to button
        break;
    }

    moveComponents(0,-5.25,1);//move up to button
    //Push button without changing position
    setVelocityComponents(0,-1,.3); Sleep(.5); halt();
    moveComponents(0,1,1); //back away

    moveBlindTo(-11,-12.02,1); //move over to wrench
    //Hit wrench without changing position
    setVelocityComponents(-1,0,.3); Sleep(.5); halt();
    moveComponents(1,0,1); //move away from wrench

    moveBlindTo(0,0,1); //move back to start
    moveComponents(0,10,0); //hit that nut button

    /////////////////////

    doc("Program finished.");
    SD.CloseLog();
}

// STARTUP AND BOOKKEEPING ####################################################

void setupRun(){
    /*
     *  Subroutine for starting up a run.
     */
    //RPS.InitializeTouchMenu();
    SD.OpenLog();
    doc("Voltage: ", Battery.Voltage());
    calibrateRPS();
    calibrateCds();
    doc("Touch to dominate.");
    waitForTouch();
    doc("Waiting for CdS.");
    startWithCds();
}
void calibrateRPS(){
    /*
     *  Make the current position the origin.
     */
    startX=RPS.X();
    startY=RPS.Y();
    updatePosition();
}
void meterMode(){
    /*
     *  Turn the robot into a "meter" for debugging sensors
     */
    while(true){
        //CdS
        float sum=0, redsum=0;
        int numCalibrations=50;
        for(int i=0; i<numCalibrations; i++){
            sum+=cds.Value();
            redControl+=cdsRed.Value();
            Sleep(4);
        }
        cdsControl = sum / numCalibrations;
        redControl = redsum / numCalibrations;
        LCD.Clear();
        LCD.Write("CdS: ");
        LCD.WriteLine(cdsControl);
        LCD.Write("CdSRed: ");
        LCD.WriteLine(redControl);

        //RPS
        LCD.Write("RPS ( ");
        LCD.Write(RPS.X());
        LCD.Write(" , ");
        LCD.Write(RPS.Y());
        LCD.Write(" )\nH ");
        LCD.WriteLine(RPS.Heading());
    }
}
void waitForTouch(){
    /*
     *  Wait until the screen is touched.
     *  Beep when touched.
     */
    float x,y;
    while(LCD.Touch(&x,&y)) Sleep(1); //until untouched
    while(!LCD.Touch(&x,&y)) Sleep(1); //until pressed
    while(LCD.Touch(&x,&y)) Sleep(1); //until released
    Buzzer.Beep();
}
bool startWithCds(){
    /*
     *  Wait until the CdS sensor consistently reads bright (low) values.
     *  Return true if successful.
     */
    int numreadings = 50;
    float readings[numreadings];
    for(int i=0;i<numreadings;i++)readings[i]=4;
    float sum=4*numreadings;
    float m=cdsControl*0.8; //threshhold in volts (20% brightness)
    int panicTime = TimeNow() + 40; //after 40 seconds, go anyway.
    while(sum/numreadings>m){
        if(TimeNow()>panicTime){
            doc("Going without CdS.");
            return false;
        }
        //move only after average of 20 under threshold.
        sum-=readings[0];
        for(int i=0;i<numreadings-1;i++)
            readings[i]=readings[i+1];
        readings[numreadings-1]=cds.Value();
        sum+=readings[numreadings-1];
        Sleep(2);
    }
    doc("Going with CdS.", sum/numreadings, m);
    return true;
}

//polymorphic function for displaying stuff on screen and logging to the SD card
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

// SENSORS ###################################################################

void calibrateCds(){
    /*
     * Reads the CdS cell average, and sets control to it.
     * Takes 1.4 seconds in total.
     */
    LCD.WriteLine("Touch to calibrate CdS.");
    waitForTouch();
    Sleep(1.0); //Wait a bit to avoid interfenece of user
    float sum=0, redsum=0;
    int numCalibrations=100;
    for(int i=0; i<numCalibrations; i++){
        sum+=cds.Value();
        redsum+=cdsRed.Value();
        Sleep(4);
    }
    cdsControl = sum / numCalibrations;
    redControl = redsum / numCalibrations;
    doc("CdS baseline:", cdsControl);
    doc("Red CdS baseline:", redControl);
}
int getColor(){
    /*
     * Reads the CdS cells and determines the light color #define code.
     */
    float avg=0, redavg=0;
    int numCalibrations=50;
    for(int i=0; i<numCalibrations; i++){
        avg+=cds.Value();
        redavg+=cdsRed.Value();
        Sleep(2);
    }
    avg/=numCalibrations;
    redavg/=numCalibrations;

    float brightness = (cdsControl-avg)/cdsControl;
    float redness = (redControl-redavg)/redControl;

    int color;

    if(brightness<.3) color=OFF;
    else if(redness>.5*brightness) color=REDLIGHT;
    else color=BLUELIGHT;

    doc("Bright/red, color", brightness, redness, color);
    return color;
}

// MOVEMENT AND NAVIGATION ###################################################

bool updatePosition(){
    /*
     *  Reads the RPS data into the 'Robot' global variable
     *  This is the only place we need to worry about how RPS works.
     */
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(heading>-1){ //TODO: fix this when we figure it out.
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
    /*
     * MOTION ALGOTITHM 1
     * Moves robot to definite position (x,y):
     * 1. go halfway
     * 2. adjust course and repeat 1
     * 3. When close enough, just go all the way.
     */
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
        halt();
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
    /*
     * MOTION ALGOTITHM 2
     * Moves robot to definite position (x,y):
     * 1. go all the way
     * 2. check and adjust position
     */
    updatePosition();
    float speed = moveAtAngleRelCourse(arg(Robot.x, Robot.y, x, y), 1.0);
    float distance = pythag(Robot.x, Robot.y, x, y);
    float endTime = TimeNow()+(distance)/speed;
    while(TimeNow()<endTime);
    if(updatePosition()){
        moveBlindTo(x,y,.3);
    } else {
        Robot.x = x;
        Robot.y = y;
    }
}

void rotateTo(float heading){
    /*
     *  Rotates the robot to face the specified heading.
     */
    updatePosition();
    float rotationAngle=deltaAngle(Robot.heading, heading);
    doc("Rot from/to/by:", Robot.heading, heading, rotationAngle);

    float angleSpeed=1;
    //slow if small angle
    if(fabs(rotationAngle)<90) angleSpeed=fabs(rotationAngle/90);
    rotateBy(rotationAngle, angleSpeed); //do it

    //Check and adjust until within 5 degrees
    if(updatePosition()){ //if RPS available, check and re-rotate
        float newAngle = principal(heading-Robot.heading);
        if( fabs(deltaAngle(Robot.heading, heading))>5 /*ADJUST*/){
            rotateTo(heading); //recursion is always good and cool. /s
        }
    } else {
        //assume we're correct
        Robot.heading=heading;
    }

}

void moveBlind(float angle, float distance, float speedPercent){
    doc("BlindMove", angle, distance);
    float speed = moveAtAngleRelCourse(angle, speedPercent);
    float endTime = TimeNow()+distance/speed;
    while(TimeNow()<endTime);
    //if(!updatePosition){ //in case RPS fails
        Robot.x += distance*cos(PI_180*angle);
        Robot.y += distance*sin(PI_180*angle);
        doc("CalcPosition", Robot.x, Robot.y);
    //}
    doc("BlindMove finished");
    halt();
}

void moveComponents(float x, float y, float speedPercent){
    /*
     * Moves robot by <x,y> without the help of RPS.
     *
     * Note: we don't do this directl with setVelocityComponents
     *       because the robot could be tilted
     */
    moveBlindTo(Robot.x+x, Robot.y+y, speedPercent);
}
void moveBlindTo(float x, float y, float speedPercent){
    /*
     * Moves the robot to (x,y) with out help of RPS.
     */
    float angle = arg(Robot.x, Robot.y, x, y);
    float distance = pythag(Robot.x, Robot.y, x, y);
    moveBlind(angle, distance, speedPercent);
}

float setVelocityComponents(float right, float forward, float speedPercent){
    /*
     *  Sets the x- and y-velocities at speedPercent of the maximum.
     *  arguments between -1 and 1.
     *
     *  Ex: setVelocityComponents(-1, 1, 0.75); will set Robot going forward
     *       and to the left at 75% Speed
     *  returns speed in inches/second
     */

    //Wheel values based on orientation of wheels:
    float fl=-forward-right;
    float fr=+forward-right;
    float bl=-forward+right;
    float br=+forward+right;
    //Scale back values so speed is maximum and no values greater than 1:
    float m=fmax( fmax( fabs(fl),fabs(fr) ), fmax( fabs(bl),fabs(br) ) );
    if(fabs(m)>.001){
        fl/=m; fr/=m; br/=m; bl/=m;
    }
    fl*=speedPercent; fr*=speedPercent; br*=speedPercent; bl*=speedPercent;
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
    /*
     *  Sets wheels to given speeds
     */
    motorFL.SetPercent(100.0*fl);
    motorFR.SetPercent(100.0*fr);
    motorBL.SetPercent(100.0*bl);
    motorBR.SetPercent(100.0*br);
}

float moveAtAngleRelRobot(float heading, float speedPercent){
    /*
     *  Sets Robot moving at an angle (0=right)
     */
    doc("RelRobot: ", heading, speedPercent);
    return setVelocityComponents(
                cos(PI_180*heading),
                sin(PI_180*heading),
                speedPercent);
}
float moveAtAngleRelCourse(float heading, float speedPercent){
    /*
     *  Sets Robot moving at an angle (0=East)
     */
    doc("RelCourse: ", heading, speedPercent);
    return moveAtAngleRelRobot(heading-Robot.heading, speedPercent);
}
void setRotation(float direction){
    /*
     *  Sets Robot Rotating at given speed/direction
     *  Ex. setRotation(.5); makes robot start spinning counterclockwise
     *      at 50% speed.
     */
    if(fabs(direction)>1) direction=(direction>0? 1:-1);
    doc("Setting Rotation:", direction);
    setWheels(direction, direction, direction, direction);
}
void rotateBy(float angle, float speedPercent){
    /*
     *  Rotates robot by given angle and %speed.
     */
    doc("Rotating by:", angle);
    setRotation(speedPercent*(angle>0? 1:-1));
    float timeEnd=TimeNow()+angle/(speedPercent*ROTATIONCONSTANT);
    while(TimeNow()<timeEnd);
    halt();
    if(!updatePosition) Robot.heading=principal(Robot.heading+angle);
    doc("Rotation Finished.");
}

// MATH FUNCTIONS WITHOUT SIDE EFFECTS #######################################
//      note: angles measured counteclockwise from +x axis

float principal(float x){
    //returns x's coterminal angle in [0,360).
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
float deltaAngle(float from, float to){
    //returns the angle change from 'from' to 'to'. Between -180 and 180.
    to = principal(to);
    from = principal(from);
    float rotationAngle=0;
    if(fabs(to-from)<180){ //if no problem with 0=360
        rotationAngle=to-from;
    //But when there is a problem:
    }else if(to>from){ //from Quadrant 3or4 to Quadrant 1or2
        rotationAngle=to-from-360;
    }else if(to<from){ //from Quadrant 1or2 to Quadrant 3or4
        rotationAngle=to-from+360;
    }
    return rotationAngle;
}
