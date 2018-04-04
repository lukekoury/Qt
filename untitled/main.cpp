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

#define SLEEPTIME 0.8 //Time delay for robot to stably halt and RPS to catch up

using namespace std;

// GLOBAL VARIABLES ###########################################################

struct POS{
    //store usable values for the robot's current position
    float x;
    float y;
    float heading;
    float timestamp; //when this was last updated
} Robot;

float cdsControl, redControl; //baseline voltages for comparison
float startX, startY, startHeading; //the "tare" that will make (0,0) be the starting position.
float crankX, crankY, crankHeading; //Results of Calibration waypoints
float wrenchX, wrenchY, wrenchHeading;
int fuelType;
float crankPosition; //current crank orientation in degrees
float runBeginTime=0; //when the run begins, so that the log file has times beginning then.
float movementEndTime=9999999999; //if it gets to this time and a movement hasn't worked, stop trying.
float addToFLWheel=0;

//Drivetrain
FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

//Sensors
AnalogInputPin cds(FEHIO::P0_1);
AnalogInputPin cdsRed(FEHIO::P0_0);

//Servos
FEHServo crankyBoi(FEHServo::Servo0);
FEHServo forkLift(FEHServo::Servo1);

// FUNCTION PROTOTYPES ########################################################

//documentation for log file
void doc(const char *text);
void doc(const char *text, float a);
void doc(const char *text, float a, float b);
void doc(const char *text, float a, float b, float c);
void doc(const char *text, float a, float b, float c, float d);

//setup & sensors
void setupRun();
void calibrateRPS();
bool startWithCds();
void calibrateCds();
int updatePosition();
int updateOnlyXY();
int updateOnlyHeading();
int getColor();
void waitForTouch();
void motorTest();
void reverseMotorTest();

//actions
void raiseForkLift();
void lowerForkLift();
void setCrank(float pos);

//Instantaneous motion commands - they set wheels and take no time.
void setWheels(float fl, float fr, float bl, float br);
void halt();
float setVelocityComponents(float right, float forward, float speedPercent);
float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);

//Motion commands that take time
void moveBlind(float angle, float distance, float speedPercent);
void moveBlindTo(float x, float y, float speedPercent);
void moveComponents(float x, float y, float speedPercent);
void rotateBy(float angle, float speedPercent);
void rotateTo(float heading, float precision);
void moveTo(float x, float y, float precision);
void moveTo(float x, float y, float precision, float speed);
void pushToward(float x, float y, float speedPercent, float duration);

//Math functions without side effects
float principal(float x);
float arg(float x1,float y1,float x2,float y2);
float pythag(float x1,float y1,float x2,float y2);
float deltaAngle(float from, float to);

// ############################################################################
// OVERALL PROCEDURE ##########################################################
// ############################################################################

int main(){
    setupRun();

    //############################################################
    //# THE MAIN PROGRAM #############################################
    //####################################################################


    doc("MOVING DOWN AND GETTING COLOR");
        moveBlindTo(0,-8,1);
        int color = OFF;
        while(color==OFF){
            moveTo(8.75,-8.5,1);
            color=getColor();
            if(color!=OFF) break;
            Sleep(SLEEPTIME);
            updatePosition();
        }

    doc("GOT COLOR, PUSHING BUTTONS", 0);
        float startTime=TimeNow();
        //Pusher centered 0.5 inches to the left of robot center
        //Buttons at 8.75 +/- 3.25,
        //so buttons are centered on pusher at x=(6 | 9.25 | 11.5)
        float awayFromCenter=3.25*0.5;
        switch(color){
            //slide over to the button, then push toward it for a bit.
            case REDLIGHT:
                LCD.Clear(RED);
                moveBlindTo(9.25-awayFromCenter,-8.5,.5);
                pushToward(9.25-awayFromCenter,-15,.8,0.6);
            break;
            case BLUELIGHT:
                LCD.Clear(BLUE);
                moveBlindTo(9.25+awayFromCenter,-8.5,.5);
                pushToward(9.25+awayFromCenter,-15,.8,0.6);\
            break;
        }

    doc("AIMING FOR WHITE ", startTime-TimeNow());
        while(RPS.IsDeadzoneActive()==0){ //until pushed
            switch(color){
            //hold the white button.
            //Case distinction in case we want to aim toward the safe side
                case REDLIGHT:
                    pushToward(9.25-awayFromCenter/4.0,-14,0.5,0.2);
                break;
                case BLUELIGHT:
                    addToFLWheel = .5;
                    pushToward(9.25,-14,0.5,0.2);
                    addToFLWheel = 0;
                break;
            }
            if(RPS.IsDeadzoneActive()!=0) break;
            moveComponents(0,2,1);
        }
        doc("TOUCHED WHITE", startTime-TimeNow());
        while(RPS.IsDeadzoneActive()!=2){ //until activated
            switch(color){
            //hold the white button.
            //Case distinction in case we want to aim toward the safe side
                case REDLIGHT:
                    pushToward(9.25-awayFromCenter/4.0,-14,0.5,0.2);
                break;
                case BLUELIGHT:
                    addToFLWheel = 0.5;
                    pushToward(9.25,-14,0.5,0.2);
                    addToFLWheel = 0;
                break;
            }
            if(TimeNow()-startTime>30) break;
            updatePosition();
        }
        doc("PUSHED FOR", TimeNow()-startTime);

    doc("AWAY FROM BOARD");
        moveComponents(0,4,1);      //away from button board
        moveBlindTo(9.25-16,Robot.y,1);
    doc("LINE UP FOR LEVER");
        movementEndTime=TimeNow()+8;
            moveTo(wrenchX+1,wrenchY,3);  //line up East-West for lever
    doc("LINE UP N/S");
        moveTo(wrenchX-3.0,wrenchY-10,2); //up to lever TODO: consider blind
    doc("PUSHING LEVER");
        pushToward(-1.79,-22.1,1,0.8); //push lever
        halt();
    doc("AWAY FROM LEVER");
        moveComponents(-3,0,1); //back up
        moveComponents(0,5,1);  //move up to not run into lever again

    doc("LINING UP W/WRENCH");
        moveTo(wrenchX+2.5,wrenchY,0.5); //Move over to wrench
        float forkliftEndTime=TimeNow()+0.8;
        forkLift.SetDegree(180);
        rotateTo(wrenchHeading, 2);
        while(TimeNow()<forkliftEndTime){
            Sleep(1);
        }
    doc("INSERTING FORKLIFT");
        moveTo(wrenchX,wrenchY,0.5); // insert into wrench
        raiseForkLift();

    doc("MOVING TO BASE OF RAMP");
        moveComponents(0,3,1); //avoid running into wrench place
        moveBlindTo(-12,-4.5,1); //base of ramp
    doc("MOVING UP RAMP");
        moveComponents(0,12,1);     //up ramp
        updatePosition();
        movementEndTime = TimeNow()+8;
        moveTo(-13,15,4); // make sure up ramp

    doc("MOVING TO CENTER");
        moveTo(0,27.5,0.5); //to center of top TODO: bigger tolerance?
        while(updatePosition()==NORPS){
            //in case we overshoot and land in dead zone
            while(updatePosition()==NORPS){
                //inch back out of dead zone
                moveComponents(1,-1,0.5);
            }
            moveTo(0,27.5,0.5); //to center of top
        }

    doc("LINING UP TO DEPOSIT");
        rotateTo(-45,2); //line up to deposit wrench
        moveBlindTo(-9+0.2, crankY+0.2, 1.0); //up to garage
    doc("DEPOSITING");
        lowerForkLift();
    doc("LEAVING GARAGE");
        moveComponents(3,-3,1); // out of garage
        forkLift.SetDegree(90); // return to vertical
        moveTo(crankX-8,crankY-8,2); //to center(ish) of top

    doc("MOVING TO CRANK");
        float startAngle, endAngle;
        switch(fuelType){
            case OCTANE:
                doc("OCTANE");
                startAngle=0;
                endAngle=180;
            break;
            case NITRO:
                doc("NITRO");
                startAngle=180;
                endAngle=0;
            break;
        }
        crankyBoi.SetDegree(startAngle);
        crankPosition=startAngle;
        moveTo(crankX-2.5,crankY-2.5,0.5); //move to crank TODO:delete?
        rotateTo(crankHeading, 3);
        movementEndTime=TimeNow()+8;
            moveTo(crankX,crankY,0.5); //move to crank
        setCrank(endAngle);
        Sleep(SLEEPTIME);

    doc("LEAVING CRANK");
        moveComponents(-1,-1,.5);
        moveComponents(-7,-7,1.0);
        crankyBoi.SetDegree(90);

    doc("GOING TO RAMP");
        rotateBy(deltaAngle(Robot.heading,0),0.4);    //get level TODO: consider faster
        movementEndTime=TimeNow()+8;
            moveBlindTo(12,14,1.0);    //approach ramp TODO: CONSIDER BLIND
        //rotateBy(deltaAngle(Robot.heading,0),0.4);    //get level TODO: consider faster
        moveTo(12,-5,3);    //descend ramp
        for(int i=0; i<10; i++){
            //please let RPS respond so we don't go the wrong direction
            if(updatePosition()!=NORPS) break;
            Sleep(20);
        }

    doc("GOING TO END BUTTON");
        while(true){
            moveBlindTo(0,-5.5,1);     //get to center
            movementEndTime=TimeNow()+4;
            moveTo(0,9,1);      //smack the button
            //PROGRAM SHOULD END HERE
            LCD.Clear(YELLOW);
            moveComponents(0,-3,1);
        }


    //####################################################################
    //################################################################
    //############################################################

    //inaccessible
    doc("Program finished.");
    SD.CloseLog();
    return 0;
}

// ############################################################################
// STARTUP ####################################################################
// ############################################################################

void setupRun(){
    /*
     * All pre-run tasks
     */
    //forkLift.TouchCalibrate();
    //crankyBoi.TouchCalibrate();
    forkLift.SetMin(690); forkLift.SetMax(2215);
    crankyBoi.SetMin(515); crankyBoi.SetMax(2300);

    RPS.InitializeTouchMenu();
    //SD.OpenLog();
    doc("Voltage: ", Battery.Voltage());

    forkLift.SetDegree(180);
    crankyBoi.SetDegree(180);
    calibrateRPS();
    movementEndTime=9999999999;

    LCD.Clear();
    forkLift.SetDegree(90);
    crankyBoi.SetDegree(90);
    crankPosition=90;
    calibrateCds();
    LCD.Clear(BLACK);
    //waitForTouch();
    LCD.Clear(GREEN);
    doc("Prepare for Domination.");
    doc("Waiting for CdS.");
    if(updatePosition()==NORPS){
        Robot.x=0; Robot.y=0;
        Robot.heading=0;
    }
    startWithCds();
    fuelType=RPS.FuelType();
}
void motorTest(){
    /*
     * Test all six motors; leave them in starting position.
     */
    forkLift.SetDegree(180);
    crankyBoi.SetDegree(180);
    setWheels(1,0,0,0); Sleep(250);
    crankyBoi.SetDegree(0);
    setWheels(0,1,0,0); Sleep(250);
    forkLift.SetDegree(90);
    setWheels(0,0,1,0); Sleep(250);
    crankyBoi.SetDegree(90);
    setWheels(0,0,0,1); Sleep(250);
    halt();
}
void reverseMotorTest(){
    /*
     * Move the servos to calibration positions.
     */
    forkLift.SetDegree(180);
    crankyBoi.SetDegree(180);
    Sleep(250);
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

// ############################################################################
// CdS CELL SENSORS ###########################################################
// ############################################################################

void calibrateCds(){
    /*
     * Reads the CdS cell average, and sets control to it.
     * Takes 1.4 seconds in total.
     */
    LCD.WriteLine("Touch to calibrate CdS.");
    //waitForTouch();
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
    runBeginTime=TimeNow();
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

    if(brightness<.3){
        color=OFF;
    }else if(redness>.8*brightness){
        color=REDLIGHT;
    }else{
        color=BLUELIGHT;
    }
    doc("Bright/red/color:", brightness, redness, color);
    return color;
}

// ############################################################################
// ACTIONS ####################################################################
// ############################################################################

void raiseForkLift(){
    for(int i=180; i>95; i--){
        forkLift.SetDegree(i);
        Sleep(8);
    }
}
void lowerForkLift(){
    float endTime = TimeNow()+0.4;
    forkLift.SetDegree(180);
    while(TimeNow()<endTime);
}

void setCrank(float pos){
    for(float i=crankPosition; i<=pos; i+=(pos-crankPosition)/60){
        crankyBoi.SetDegree(i);
        Sleep(0.5/60);
    }
    crankyBoi.SetDegree(pos);
    crankPosition=pos;
}

// ############################################################################
// RPS ########################################################################
// ############################################################################

int updatePosition(){
    /*
     *  Reads the RPS data into the 'Robot' global variable if valid and new
     *  (The "Timestamp" functionality is not currently used.)
     */
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(heading>-1){
        heading=principal(heading-startHeading);
        if(Robot.x == x && Robot.y == y && Robot.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
            //doc("RPS Stayed the same");
            return OLDRPS;
        } else {
            Robot.x = x;
            Robot.y = y;
            Robot.heading = heading;
            Robot.timestamp=TimeNow();
            //doc("RPS: ", x, y, heading);
            return NEWRPS;
        }
    }
    //doc("RPS Update Failed: ",x,y,heading);
    return NORPS;
}
int updateOnlyXY(){
    /*
     *  Reads the RPS data into the 'Robot' global variable if valid and new
     *  (The "Timestamp" functionality is not currently used.)
     */
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(heading>-1){
        heading=principal(heading-startHeading);
        if(Robot.x == x && Robot.y == y && Robot.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
            //doc("XY Stayed the same");
            return OLDRPS;
        } else {
            Robot.x = x;
            Robot.y = y;
            //Robot.heading = heading; DON'T UPDATE
            Robot.timestamp=TimeNow();
            doc("XY:  ", x, y, heading);
            return NEWRPS;
        }
    }
    //doc("XY Update Failed: ",x,y,heading);
    return NORPS;
}
int updateOnlyHeading(){
    /*
     *  Reads the RPS data into the 'Robot' global variable if valid and new
     *  (The "Timestamp" functionality is not currently used.)
     */
    float x = RPS.X()-startX;
    float y = RPS.Y()-startY;
    float heading = RPS.Heading();
    if(heading>-1){
        heading=principal(heading-startHeading);
        if(Robot.x == x && Robot.y == y && Robot.heading == heading){
            //Do not update timestamp if RPS hasn't been updated
            //doc("Heading Stayed the same");
            return OLDRPS;
        } else {
            //Robot.x = x; DON'T UPDATE
            //Robot.y = y;
            Robot.heading = heading;
            Robot.timestamp=TimeNow();
            //doc("Heading: ", x, y, heading);
            return NEWRPS;
        }
    }
    //doc("Heading Update Failed: ",x,y,heading);
    return NORPS;
}


class waypoint{
    //points to caibrate before run
public:
    //fill constructor
    waypoint(char* name, float x, float y, float head, float screenLeft, float screenTop, float w, float h);
    //defaults in case we don't have time to calibrate
    float myDefaultX;
    float myDefaultY;
    float myDefaultHeading;
    //current values
    float myX;
    float myY;
    float myHeading;
    //GUI buttons
    FEHIcon::Icon calButton;
    FEHIcon::Icon revertButton;
    void calibrateHere();
    void revert(); //to defaults
    bool calibrated;
    unsigned int labelColor();
};
    waypoint::waypoint(char* name, float x, float y, float head, float screenLeft, float screenTop, float w, float h){
        waypoint::myDefaultX=x; waypoint::myDefaultY=y; waypoint::myDefaultHeading=head;
        waypoint::myX=myDefaultX; waypoint::myY=myDefaultY; waypoint::myHeading=myDefaultHeading;
        waypoint::calibrated=false;
        waypoint::calButton.SetProperties(name,screenLeft+1, screenTop+1,w/2-2,h-2,GOLD,GOLD);
        waypoint::revertButton.SetProperties("Revert",screenLeft+w/2+1, screenTop+1,w/2-2,h-2,PINK,PINK);
        waypoint::calButton.Draw();
        waypoint::revertButton.Draw();
    }
    void waypoint::calibrateHere(){
        Buzzer.Beep();
        Robot.x=-1; Robot.y=-1;
        float endTime=TimeNow()+2;
        while((Robot.x<0 || Robot.y<0) && TimeNow()<endTime){
            Robot.x=RPS.X();
            Robot.y=RPS.Y();
            Robot.heading=RPS.Heading();
            if( !(Robot.x<0 || Robot.y<0) ){
                calibrated=true;
            }
        }
        waypoint::myX=Robot.x;
        waypoint::myY=Robot.y;
        waypoint::myHeading=Robot.heading;

    }
    void waypoint::revert(){
        //revert x,y to defaults
        Buzzer.Beep();
        waypoint::myX=waypoint::myDefaultX;
        waypoint::myY=waypoint::myDefaultY;
        waypoint::myHeading=waypoint::myDefaultHeading;
        waypoint::calibrated=false;
    }
    unsigned int waypoint::labelColor(){
        //color depends on how close calibration is to expected value
        if(waypoint::calibrated){
            float d=0.05*fabs(deltaAngle(
                          waypoint::myHeading,
                          waypoint::myDefaultHeading)
                    )
                    +pythag(
                          waypoint::myX,waypoint::myY,
                          waypoint::myDefaultX,waypoint::myDefaultY
                    );
            if(d<1){
                return 0x00FF00;
            }else if(d<3){
                return YELLOW;
            }else{
                return RED;
            }
        }
        return GRAY;
    }

void calibrateRPS(){
    /*
     * Allows us to re-calibrate exact cooordinates before each run.
     *
     * Creates a GUI that will be used as follows:
     *  1. Move Robot to Crank and press "Crank"
     *  2. Move Robot to Wrench and press "Wrench:
     *  3. Press "Motors" To test all 6 Motors and move them to starting positions
     *  4. Move Robot to Origin (starting light) and press "Origin"
     *  5. Press "GO!" to begin.
     *
     *  -If any of these steps is not followed, we should be fine because of the defaults.
     *  -The GUI includes the default values (GRAY) next to the current waypoint values.
     *  -The current waypoint values are colored by their distance from the default
     *      -Red=more than 3 inches away; Green=closer than 1in; Yellow=between
     *      -Gray=hasn't been changed from default
     *  -If one of the values is incorrect, Recalibrate OR press "Revert" to reset to default.
     *  -Pressing "Motors" again will return motors to original state for continuing calibration.
     */

    LCD.Clear();
    waypoint crank("Crank",     25.4,63.9,315   ,160,0,160,60);
    waypoint wrench("Wrench",    8.4,17.9,0     ,160,60,160,60);
    waypoint origin("Origin",   17.1,29.0,0     ,160,120,160,60);
    FEHIcon::Icon motorButton, goButton;
    bool motorsTested=false;
    motorButton.SetProperties("Motors",161,181,78,56,CYAN,CYAN);
    goButton.SetProperties("GO!",241,181,78,56,0x00FF00,0x00FF00); //green
    goButton.Draw();
    motorButton.Draw();

    LCD.SetFontColor(GRAY); //Display Defaults
    LCD.WriteAt(crank.myDefaultX, 0,10);     LCD.WriteAt(crank.myDefaultY, 80,10);
    LCD.WriteAt(wrench.myDefaultX, 0,70);   LCD.WriteAt(wrench.myDefaultY, 80,70);
    LCD.WriteAt(origin.myDefaultX, 0,130);  LCD.WriteAt(origin.myDefaultY, 80,130);

    float x,y; //screen position touched
    while(true){
        LCD.Touch(&x,&y);
        if(goButton.Pressed(x,y,1)) break;
        if(crank.calButton.Pressed(x,y,1))      crank.calibrateHere();
        if(wrench.calButton.Pressed(x,y,1))     wrench.calibrateHere();
        if(origin.calButton.Pressed(x,y,1))     origin.calibrateHere();
        if(crank.revertButton.Pressed(x,y,1))   crank.revert();
        if(wrench.revertButton.Pressed(x,y,1))  wrench.revert();
        if(origin.revertButton.Pressed(x,y,1))  origin.revert();
        if(motorButton.Pressed(x,y,1)){
            if(!motorsTested){
                motorTest();
                motorsTested=true;
            } else {
                reverseMotorTest();
                motorsTested=false;
            }
        }
        //Display calibrations
        LCD.SetFontColor(crank.labelColor());
            LCD.WriteAt(crank.myX, 0,40);   LCD.WriteAt(crank.myY, 80,40);
        LCD.SetFontColor(wrench.labelColor());
            LCD.WriteAt(wrench.myX, 0,100); LCD.WriteAt(wrench.myY, 80,100);
        LCD.SetFontColor(origin.labelColor());
            LCD.WriteAt(origin.myX, 0,160); LCD.WriteAt(origin.myY, 80,160);

        LCD.SetFontColor(WHITE); //Display Current position
        LCD.WriteAt(RPS.X(), 0,197); LCD.WriteAt(RPS.Y(), 80,197);
        LCD.WriteAt(RPS.Heading(), 40,215);
        Sleep(1);
    }

    doc("Crank RPS", crank.myX, crank.myY, crank.myHeading);
    doc("Wrench RPS", wrench.myX, wrench.myY, wrench.myHeading);
    startX=origin.myX;
    startY=origin.myY;
    startHeading=origin.myHeading;
    doc("RPS Tare:",startX,startY,startHeading);
    crankX=crank.myX-startX;
    crankY=crank.myY-startY;
    crankHeading=principal(crank.myHeading-startHeading);
    doc("Crank Position:",crankX, crankY, crankHeading);
    wrenchX=wrench.myX-startX;
    wrenchY=wrench.myY-startY;
    wrenchHeading=principal(wrench.myHeading-startHeading);
    doc("Wrench Position:",wrenchX, wrenchY, wrenchHeading);

    updatePosition();
}

// ############################################################################
// MOVEMENT ###################################################################
// ############################################################################
//      roughly in order of dependence, most primitive first.

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
void halt(){
    //doc("Halting");
    setWheels(0,0,0,0);
}
float setVelocityComponents(float right, float forward, float speedPercent){
    /*
     *  Sets the x- and y-velocities at speedPercent of the maximum.
     *  arguments between -1 and 1.
     *
     *  Ex: setVelocityComponents(-1, 1, 0.75); will set Robot going diagonally
     *       forward and to the left at 75% Speed
     *  returns speed in inches/second
     */

    //Wheel values based on orientation of wheels:
    float fl=-forward-right + addToFLWheel;
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
void moveBlind(float angle, float distance, float speedPercent){
    /*
     * Moves robot at angle (RelCourse), by distance, at speedPercent,
     * without RPS.
     */
    float speed = moveAtAngleRelCourse(angle, speedPercent);
    float endTime = TimeNow()+distance/speed;
    while(TimeNow()<endTime){
        updateOnlyHeading(); /////////NEW
        Sleep(2);
    }
    halt();

    //This is the one place where we assume that the robot's
    //  intended motion has actually happened.
    //  (Along with wherever this function is called)
    Robot.x += distance*cos(PI_180*angle);
    Robot.y += distance*sin(PI_180*angle);

    doc("blindPosition", Robot.x, Robot.y);

}
void moveBlindTo(float x, float y, float speedPercent){
    /*
     * Moves the robot to (x,y) without help of RPS.
     */
    float angle = arg(Robot.x, Robot.y, x, y);
    float distance = pythag(Robot.x, Robot.y, x, y);
    moveBlind(angle, distance, speedPercent);
}
void moveComponents(float x, float y, float speedPercent){
    /*
     * Moves robot by <x,y> without the help of RPS.
     * Note: we don't do this directly with setVelocityComponents
     *       because the robot could be tilted
     */
    moveBlindTo(Robot.x+x, Robot.y+y, speedPercent);
}
void moveTo(float x, float y, float precision){
    /*
     *  Moves the robot to within precision of (x,y).
     *  Is the most frequently called function in the main program!
     */

    doc("moveTo", x, y, precision);
    float distance = pythag(Robot.x,Robot.y, x, y);
    if(distance>precision && TimeNow()<=movementEndTime){
        float speedPercent=1;
        if(distance<6) speedPercent=.5; //slower if closer TODO: Make faster?
        moveBlindTo(x,y,speedPercent);
        Sleep(SLEEPTIME); //wait for RPS to catch up
        updatePosition();
        moveTo(x, y, precision); //Recursion is always good and cool, right?
    } else {
        //Finish moving
        movementEndTime=99999999999;  //so as not to affect the next movement
    }
}

void moveTo(float x, float y, float precision, float speed){
    /*
     *  Moves the robot to within precision of (x,y), at speed
     *  Is the most frequently called function in the main program!
     */

    doc("moveTo", x, y, precision);
    float distance = pythag(Robot.x,Robot.y, x, y);
    if(distance>precision && TimeNow()<=movementEndTime){
        float speedPercent=speed;
        moveBlindTo(x,y,speedPercent);
        Sleep(SLEEPTIME); //wait for RPS to catch up
        updatePosition();
        moveTo(x, y, precision, speed); //Recursion is always good and cool, right?
    } else {
        //Finish moving
        movementEndTime=99999999999;  //so as not to affect the next movement
    }
}

void pushToward(float x, float y, float speedPercent, float duration){
    /*
     * Pushes toward a particular point for some duration.
     * Updates position and adjusts, so more stable than simple pushAgainst.
     * Used to push buttons.
     */
    float endTime=TimeNow()+duration;
    while(TimeNow()<endTime){
        float angle = arg(Robot.x, Robot.y, x, y);
        moveAtAngleRelCourse(angle, speedPercent);
        Sleep(100);
        updatePosition();
    }
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
    while(TimeNow()<timeEnd){
        updateOnlyXY();
        Sleep(2);
    }
    halt();
    Robot.heading=principal(Robot.heading+angle);
    doc("Rotation Finished.");
}
void rotateTo(float heading, float precision){
    /*
     *  Rotates the robot to face the specified heading.
     *  (Within precision degrees)
     */
    updatePosition();
    float rotationAngle=deltaAngle(Robot.heading, heading);
    if(fabs(rotationAngle)>precision){
        doc("Rot from/to/by:", Robot.heading, heading, rotationAngle);
        float angleSpeed=.6;
        if(fabs(rotationAngle)<90) angleSpeed=.4;
        rotateBy(rotationAngle, angleSpeed);
        Sleep(SLEEPTIME);
        rotateTo(heading, precision);
    }

}

// ############################################################################
// MATH FUNCTIONS WITHOUT SIDE EFFECTS ########################################
// ############################################################################
//      note: angles measured in degrees counteclockwise from +x axis

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
    //returns the angle from "from" to "to", between -180 and +180
    //Much much simpler than weird if statements.
    float angle = to-from;
    while(angle>180)angle-=360;
    while(angle<=-180)angle+=360;
    return angle;
}


// ############################################################################
// DATA LOGGING ###############################################################
// ############################################################################
// Functions for displaying stuff on screen and logging to the SD card.
// Accept a string and 0-4 floating point numbers to document.

void doc(const char *text){
//    SD.Printf(FRMT, TimeNow()-runBeginTime);
//    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
//    SD.Printf("\r\n"); LCD.Write('\n');
    //Fun fact: Windows doesn't recognize "\n" as a line break,
    //  but it does recognize "\r\n",
    //  so you can open these files straight in Notepad.
    //If you're an FEH TA/Instructor reading this, tell future students.
}
void doc(const char *text, float a){
//    SD.Printf(FRMT, TimeNow()-runBeginTime);
//    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
//    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
//    SD.Printf("\r\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b){
//    SD.Printf(FRMT, TimeNow()-runBeginTime);
//    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
//    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
//    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
//    SD.Printf("\r\n"); LCD.Write('\n');
}
void doc(const char *text, float a, float b, float c){
//    SD.Printf(FRMT, TimeNow()-runBeginTime);
//    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
//    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
//    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
//    SD.Printf(FRMT, c); LCD.Write(c); LCD.Write(" ");
//    SD.Printf("\r\n"); LCD.Write('\n'); LCD.Write(" ");
}
void doc(const char *text, float a, float b, float c, float d){
//    SD.Printf(FRMT, TimeNow()-runBeginTime);
//    SD.Printf(text); LCD.Write(text); LCD.Write(" ");
//    SD.Printf(FRMT, a); LCD.Write(a); LCD.Write(" ");
//    SD.Printf(FRMT, b); LCD.Write(b); LCD.Write(" ");
//    SD.Printf(FRMT, c); LCD.Write(c); LCD.Write(" ");
//    SD.Printf(FRMT, d); LCD.Write(d); LCD.Write(" ");
//    SD.Printf("\r\n"); LCD.Write('\n');
}
