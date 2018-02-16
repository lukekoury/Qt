#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <FEHSD.h>
using namespace std;

#define PI_180 0.0174532925
#define SPEEDCONSTANT 1.0 //TODO: Calibrate
#define ROTATIONCONSTANT 1.0 //TODO: Calibrate

float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);
void moveBlind(float angle, float distance);
void rotateBy(float angle);
void rotateTo(float heading);
void halt();
void moveTo(float x, float y);

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

struct {
  float x;
  float y;
  float heading;
} RobotPosition;

int main(){
  RPS.InitializeTouchMenu();
  OpenLog();
  updatePosition();
  CloseLog();
  //example: box starting from top corner
}

bool updatePosition(){
  float x = X();
  float y = Y();
  float heading = Heading();
  if(abs(x-(-1))>.001){
    RobotPosition.x = X();
    RobotPosition.y = Y();
    RobotPosition.heading = Heading();
    doc("Position: ", x,y,heading);
    return true;
  }
  doc("Position Update Failed: "x,y,heading);
  return false;
}

void doc(const char * text, float a=nan, float b=nan, float c=nan){
  LCD.WriteLine(text+to_string(a)+to_string(b)+to_string(c));
  SD.Printf(to_string(TimeNow())+text+to_string(a)+to_string(b)+to_string(c));
}

void setWheels(float fl, float fr, float bl, float br){
  //Sets wheels to given speeds
  //TODO: include calibrations for wheels
  motorFL.SetPercent(100.0*fl);
  motorFR.SetPercent(100.0*fr);
  motorBL.SetPercent(100.0*bl);
  motorBR.setPercent(100.0*br);
}

float setVelocityComponents(float right, float forward, float speedPercent){
  //arguments between -1 and 1
	//Ex: setVelocityComponents(-1, 1, 0.75);
  //  will set robot going forward and to the left at 75% Speed
	//returns speed
  float fl=forward+right;
  float fr=-forward+right;
  float bl=forward-right;
  float br=-forward-right;

	//Scale back so speed is maximum and no overflow:
  float m=max( max( abs(fl),abs(fr) ), max( abs(bl),abs(br) ) );
  if(abs(m)>.001){
    fl/=m; fr/=m; br/=m; bl/=m;
  }
  if(abs(speedPercent)>.001){
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
	return setVelocityComponents(cos(PI_180 * heading), sin(PI_180*heading), 1.0);
}

float moveAtAngleRelCourse(float heading, float speedPercent){
  doc("RelCourse: ", heading, speedPercent);
	return moveAtAngleRelRobot(heading-RobotPosition.heading, speedPercent);
}

void setRotation(float direction){
  if(abs(direction)>1) direction/=abs(direction);
  doc("Setting Rotation:", direction);
  setWheels(direction, direction, direction, direction);
}
void rotateBy(float angle){
  doc("Rotating by:", angle)
	setRotation(angle>0? 1:-1);
	timeEnd=TimeNow()+angle/ROTATIONCONSTANT;
	while(TimeNow()<timeEnd){
		//Do Stuff while rotating
	}
  halt();
  doc("Rotation Finished.");
}
void rotateTo(float heading){
  heading %=360;
	currentHeading=RobotPosition.heading%360;
  float rotationAngle=0;
	if(abs(heading-currentHeading)<180){
		rotationAngle=heading-currentHeading);
	}else if(heading>currentHeading){
		rotationAngle=-currentHeading+(heading-360)
	}else if(heading<currentHeading){
    rotationAngle=(360-currentHeading)+heading
  }
  doc("Rot from/to/by:", currentHeading, heading, rotationAngle);
  rotateBy(rotationAngle);
	//Maybe check and adjust?
}
void moveBlind(float angle, float distance){
  doc("BlindMove", angle, distance);
	float speed = moveAtAngleRelCourse(angle, 1.0);
	float endTime = TimeNow()+distance/speed;
	while(timeNow()<endTime);
  doc("BlindMove finished");
}
void moveTo(float x, float y){
	float speed = moveAtAngleRelCourse(atan2(y-currentY, x-currentx), 1.0);
	//Algorithm? Adjust course halfway through?
  //Within certain distance, go blind?
}

void halt(){
  doc("Halting");
  setWheels(0,0,0,0);
}
