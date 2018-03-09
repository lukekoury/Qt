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
#define ROTATIONCONSTANT 180 //degrees per second
#define FRMT "%.2f "
#define OFF 0
#define REDLIGHT 1
#define BLUELIGHT 2
#define NEWRPS 1
#define NORPS 0
#define OLDRPS 2
#define OCTANE 1
#define NITRO 2

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
int fuelType;
float crankPosition;

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

AnalogInputPin cds(FEHIO::P0_1);
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
void motorTest();

void raiseForkLift();
void lowerForkLift();
void setCrank(float pos);

void setWheels(float fl, float fr, float bl, float br);
void halt();

float setVelocityComponents(float right, float forward, float speedPercent);
float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);
void pushAgainst(float heading, float speedPercent, float duration);

float principal(float x);
float arg(float x1,float y1,float x2,float y2);
float pythag(float x1,float y1,float x2,float y2);
float deltaAngle(float from, float to);

void moveBlind(float angle, float distance, float speedPercent);
void moveBlindTo(float x, float y, float speedPercent);
void moveComponents(float x, float y, float speedPercent);
void rotateBy(float angle, float speedPercent);
void rotateTo(float heading, float precision);
void moveTo(float x, float y, float precision);
int updatePosition();
void moveToFast(float x, float y, float precision);
void pushToward(float x, float y, float speedPercent, float duration);

// OVERALL PROCEDURE ##########################################################

int main(){
    setupRun();


    /////////////////////////////////

    moveBlindTo(0,-8,1);

    int color = OFF;
    while(color==OFF){
        moveTo(8.75,-8.5,1);
        color=getColor();
        if(color!=OFF) break;
        Sleep(0.8);
        updatePosition();
    }

    switch(color){
        //Pusher centered 0.5 inches to the left of robot center
        case REDLIGHT:
            LCD.Clear(RED);
            pushToward(6.5,-13,.8,1);
        break;
        case BLUELIGHT:
            LCD.Clear(BLUE);
            pushToward(12,-13,.8,1);
        break;
    }

    moveComponents(0,3,1);

    while(RPS.IsDeadzoneActive()!=2){
        pushToward(8.75,-13,.8,6);
        moveComponents(0,3,1);
    }

    moveComponents(0,2,1);
    moveComponents(-8,0,1);
    moveTo(-5.5,-12,2); //over
    moveTo(-14,-22.1,1); //corner
    pushToward(-1.79,-22.1,1,1); //lever
    moveComponents(-4,0,1);


    moveTo(-5.5,-11.7,0.5); //Move over to wrench
    lowerForkLift();
    rotateTo(0,2); //line up with wrench
    Sleep(0.8);
    moveTo(-9,-11.7,0.5); // insert into wrench
    raiseForkLift();

    moveComponents(0,3,1);
    moveBlindTo(-12,-4.5,1);
    moveComponents(0,12,1);
    updatePosition();
    moveTo(-13,15,4); // make sure up ramp

    moveTo(0,27.5,1); //to center of top
    rotateTo(-45,2); //line up to deposit wrench
    moveTo(-9.5-.75, 36, 1); //up to garage
    lowerForkLift();
    Sleep(0.5);
    moveComponents(3,-3,1); // out of garage
    forkLift.SetDegree(90);

    moveTo(0,27.5,2); //to center of top
    rotateTo(-45,3);

    switch(fuelType){
        case OCTANE:
            crankyBoi.SetDegree(0);
            crankPosition=0;
            moveTo(6.40,33.29,0.5); //move to crank
            moveTo(7.90,34.79,0.5); //move to crank
            setCrank(180);
            Sleep(.8);
        break;
        case NITRO:
            crankyBoi.SetDegree(180);
            crankPosition=180;
            moveTo(6.40,33.29,0.5); //move to crank
            moveTo(7.90,34.79,0.5); //move to crank
            setCrank(0);
            Sleep(.8);
        break;
    }
    moveComponents(-3,-3,.5);
    Sleep(0.8);

    moveTo(0,27.5,2); //to center of top

    moveTo(12,14,2);    //approach ramp
    rotateTo(90,5);      //get level
    moveTo(12,-5,1);    //descend ramp
    moveTo(0,-5,1);     //get to center
    moveTo(0,9,1);      //smack the button



    /////////////////////////////////



    doc("Program finished.");
    SD.CloseLog();
    return 0;
}

// STARTUP AND BOOKKEEPING ####################################################

void setupRun(){
    //forkLift.TouchCalibrate();
    //vfcrankyBoi.TouchCalibrate();
    forkLift.SetMin(690); forkLift.SetMax(2250);
    forkLift.SetDegree(90);
    crankyBoi.SetMin(515); crankyBoi.SetMax(2300);
    crankyBoi.SetDegree(90);
    crankPosition=90;
    RPS.InitializeTouchMenu();
    SD.OpenLog();
    doc("Voltage: ", Battery.Voltage());
    calibrateCds();
    calibrateRPS();
    doc("Touch to dominate.");
    waitForTouch();
    doc("Waiting for CdS.");
    startWithCds();
    fuelType=RPS.FuelType();
}
void motorTest(){
    forkLift.SetDegree(180);
    setWheels(1,0,0,0); Sleep(250);
    setWheels(0,1,0,0); Sleep(250);
    forkLift.SetDegree(90);
    setWheels(0,0,1,0); Sleep(250);
    setWheels(0,0,0,1); Sleep(250);
    halt();
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
void meterMode(){
    /*
     *  Turn the robot into a "meter" for debugging sensors
     */
    RPS.InitializeTouchMenu();
    calibrateRPS();
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
        LCD.Write(RPS.X()-startX);
        LCD.Write(" , ");
        LCD.Write(RPS.Y()-startY);
        LCD.Write(" )\nH ");
        LCD.WriteLine(RPS.Heading());
    }
}

// CdS SENSORS ###############################################################

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
bool startWithCds(){
    /*
     *  Wait until the CdS sensor consistently reads
     *  bright (low) values. Return true if successful.
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
int getColor(){
    /*
     * Reads the CdS cells and determines the light color code.
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
    else if(redness>.8*brightness) color=REDLIGHT;
    else color=BLUELIGHT;

    doc("Bright/red/color:", brightness, redness, color);
    return color;
}

// ACTIONS #################################################################

void raiseForkLift(){
    for(int i=180; i>95; i--){
        forkLift.SetDegree(i);
        Sleep(12);
    }
}
void lowerForkLift(){
    for(int i=95; i<180; i++){
        forkLift.SetDegree(i);
        Sleep(12);
    }
}

void setCrank(float pos){
    for(float i=crankPosition; i<=pos; i+=(pos-crankPosition)/60){
        crankyBoi.SetDegree(i);
        Sleep(0.5/60);
    }
    crankyBoi.SetDegree(pos);
    crankPosition=pos;
}

// RPS #####################################################################

int updatePosition(){
    /*
     *  Reads the RPS data into the 'Robot' global variable if valid and new
     */
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(heading>-1){
        if(Robot.x == x && Robot.y == y && Robot.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
            doc("RPS Not Updated");
            return OLDRPS;
        } else {
            Robot.x = x;
            Robot.y = y;
            Robot.heading = heading;
            Robot.timestamp=TimeNow();
            doc("RPS: ", x, y, heading);
            return NEWRPS;
        }
    }
    doc("Position Update Failed: ",x,y,heading);
    return NORPS;
}

void calibrateRPS(){
    /*
     *  Make the current position the origin.
     *  Keep checking until detected
     */
//    doc("Step away to calibrate RPS.");
//    float h=-1;
//    while(h<=0){
//        h=RPS.Heading();
//        startX=RPS.X();
//        startY=RPS.Y();
//    }
//    Buzzer.Beep();
//    doc("RPS tare:", startX, startY);
    startX=17.4;
    startY=29.3;
    updatePosition();
}

// MOVEMENT ##############################################################

void moveTo(float x, float y, float precision){
    /*
     *  Moves the robot to within precision of (x,y).
     */
    doc("moveTo", x, y, precision);
    float distance = pythag(Robot.x,Robot.y, x, y);
    if(distance>precision){
        float speedPercent=1;
        if(distance<6) speedPercent=.3;
        moveBlindTo(x,y,speedPercent);
        Sleep(.8);
        updatePosition();
        moveTo(x, y, precision);
    }

}

void moveToFast(float x, float y, float precision){
    /*
     * Updates position while moving. Doesn't work.
     */
    updatePosition();
    while(pythag(Robot.x,Robot.y, x, y)>precision){
        float distance = pythag(Robot.x,Robot.y, x, y);
        float angle = arg(Robot.x,Robot.y, x, y);
        float speed = moveAtAngleRelCourse(angle,(distance>6? 1.0:0.4));
        float endTime=TimeNow()+0.5*distance/speed; /*ADJUST*/
        bool updated=false;
        while(TimeNow()<endTime){
            if(updatePosition()==NEWRPS) updated=true;
        }
        if(updated){
            //change in position since last new update
            //consistently overshooting => assume more delay.
            float d=speed*(TimeNow()-Robot.timestamp+.5/*ADJUST*/);
            Robot.x+=d*cos(PI_180*angle);
            Robot.y+=d*sin(PI_180*angle);
        } else {
            Robot.x=x;
            Robot.y=y;
        }
        if( pythag(Robot.x,Robot.y, x, y)<precision){
            halt();
            if(TimeNow()-Robot.timestamp>1.0/*Adjust*/){
                Sleep(0.8);
                updatePosition();
            }
        }

    }
}

void pushAgainst(float heading, float speedPercent, float duration){
    /*
     * Blindly turn on the motors to push in a particular direction.
     * Used to push against buttons.
     * Benefit:     Unlike moveBlind or moveComponents, this will
     *              not blindly increment position values.
     */
    doc("push against", heading, duration);
    float speed = moveAtAngleRelCourse(heading,speedPercent);
    float stopTime=TimeNow()+duration;
    while(TimeNow()<stopTime);

    updatePosition(); //if this fails, assume we didn't move.

}

void pushToward(float x, float y, float speedPercent, float duration){
    float endTime=TimeNow()+duration;
    while(TimeNow()<endTime){
        float angle = arg(Robot.x, Robot.y, x, y);
        moveAtAngleRelCourse(angle, speedPercent);
        Sleep(100);
        updatePosition();
    }
    halt();
}

void rotateTo(float heading, float precision){
    /*
     *  Rotates the robot to face the specified heading.
     */
    updatePosition();
    float rotationAngle=deltaAngle(Robot.heading, heading);
    if(fabs(rotationAngle)>precision){
        doc("Rot from/to/by:", Robot.heading, heading, rotationAngle);
        float angleSpeed=.6;
        if(fabs(rotationAngle)<90) angleSpeed=.4;
        rotateBy(rotationAngle, angleSpeed);
        Sleep(.8);
        rotateTo(heading, precision);
    }

}
void moveBlindTo(float x, float y, float speedPercent){
    /*
     * Moves the robot to (x,y) without help of RPS.
     */
    float angle = arg(Robot.x, Robot.y, x, y);
    float distance = pythag(Robot.x, Robot.y, x, y);
    moveBlind(angle, distance, speedPercent);
}
void moveBlind(float angle, float distance, float speedPercent){
    /*
     * Moves robot at angle (RelCourse), by distance, at speedPercent,
     * without RPS.
     */
    float speed = moveAtAngleRelCourse(angle, speedPercent);
    float endTime = TimeNow()+distance/speed;
    while(TimeNow()<endTime);
    halt();

    Robot.x += distance*cos(PI_180*angle); //the one place where we assume
    Robot.y += distance*sin(PI_180*angle);

    doc("blindPosition", Robot.x, Robot.y);

}
void moveComponents(float x, float y, float speedPercent){
    /*
     * Moves robot by <x,y> without the help of RPS.
     * Note: we don't do this directly with setVelocityComponents
     *       because the robot could be tilted
     */
    moveBlindTo(Robot.x+x, Robot.y+y, speedPercent);
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
    //doc("Wheels set: ",fl,fr);
    //doc("Velocity:", right, forward, speed);
    return speed;
}

void halt(){
    doc("Halting");
    setWheels(0,0,0,0);
}

void setWheels(float fl, float fr, float bl, float br){
    /*
     *  Sets wheels to given speeds.
     *  Positive is counterclockwise from perspective of robot;
     *  Ex: setWheels(1,1,1,1);
     *      will make the robot spin counterclockwise real fast.
     */
    motorFL.SetPercent(100.0*fl);
    motorFR.SetPercent(100.0*fr);
    motorBL.SetPercent(100.0*bl);
    motorBR.SetPercent(100.0*br);
}

float moveAtAngleRelRobot(float heading, float speedPercent){
    /*
     *  Sets Robot moving at an angle (0=right, 90=forward)
     */
    //doc("RelRobot: ", heading, speedPercent);
    return setVelocityComponents(
                cos(PI_180*heading),
                sin(PI_180*heading),
                speedPercent);
}
float moveAtAngleRelCourse(float heading, float speedPercent){
    /*
     *  Sets Robot moving at an angle (0=East, 90=North)
     */
    //doc("RelCourse: ", heading, speedPercent);
    return moveAtAngleRelRobot(principal(heading-Robot.heading), speedPercent);
}
void setRotation(float direction){
    /*
     *  Sets Robot Rotating at given speed/direction
     *  Ex. setRotation(.5); makes robot start spinning counterclockwise
     *      at 50% speed.
     */
    if(fabs(direction)>1) direction=(direction>0? 1:-1);
    //doc("Setting Rotation:", direction);
    setWheels(direction, direction, direction, direction);
}
void rotateBy(float angle, float speedPercent){
    /*
     *  Rotates robot by given angle and %speed.
     */
    //doc("Rotating by:", angle, speedPercent);
    setRotation(speedPercent*(angle>0? 1:-1));

    float timeEnd=TimeNow()+fabs(angle)/(speedPercent*ROTATIONCONSTANT);
    while(TimeNow()<timeEnd);
    halt();
    Robot.heading=principal(Robot.heading+angle);
    doc("Rotation Finished.");
}

// MATH FUNCTIONS WITHOUT SIDE EFFECTS #######################################
//      note: angles measured counteclockwise from +x axis

float principal(float x){
    //returns x's coterminal angle in [0,360).
    while(x>=360) x-=360;
    while(x<0) x+=360;
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
    //returns the angle from from to to, between -180 and +180
    float angle = to-from;
    while(angle>180)angle-=360;
    while(angle<=-180)angle+=360;
    return angle;
}

// DATA LOGGING ###############################################################
// functions for displaying stuff on screen and logging to the SD card:
void doc(const char *text){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
    SD.Printf("\r\n"); LCD.Write('\n');
}
void doc(const char *text, float a){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
    SD.Printf("\r\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
    SD.Printf("\r\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b, float c){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
    SD.Printf(FRMT, c); LCD.Write(c); LCD.Write(" ");
    SD.Printf("\r\n"); LCD.Write('\n'); LCD.Write(" ");
}
void doc(const char *text, float a, float b, float c, float d){
    SD.Printf(FRMT, TimeNow());
    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
    SD.Printf(FRMT, c); LCD.Write(c); LCD.Write(" ");
    SD.Printf(FRMT, d); LCD.Write(d); LCD.Write(" ");
    SD.Printf("\r\n"); LCD.Write('\n');
}
