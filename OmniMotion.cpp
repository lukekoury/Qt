#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
using namespace std;

#define PI_180 0.0174532925
#define SPEEDCONSTANT 1.0 
#define ROTATIONCONSTANT 1.0

float moveAtAngleRelRobot(float heading, float speedPercent);
float moveAtAngleRelCourse(float heading, float speedPercent);
float setRotation(float direction);
void moveBlind(float angle, float distance);
void rotateBy(float angle);
void rotateTo(float heading);
void halt();
void moveTo(float x, float y);

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

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
  setWheels(fl*speedPercent,fr*speedPercent,bl*speedPercent,br*speedPercent);
	return speedPercent * SPEEDCONSTANT * sqrt(fl*fl+fr*fr);
}

float moveAtAngleRelRobot(float heading, float speedPercent){
	return setVelocityComponents(cos(PI_180 * heading), sin(PI_180 * heading), 1.0);
}

float moveAtAngleRelCourse(float heading, float speedPercent){
	return moveAtAngleRelRobot(heading-(0/*Robot Orientation*/), speedPercent);
}

void setRotation(float direction){
  if(abs(direction)>1) direction/=abs(direction);
  setWheels(direction, direction, direction, direction);
}
void rotateBy(float angle){
	setRotation(angle>0? 1:-1);
	timeEnd=TimeNow()+angle()/ROTATIONCONSTANT;
	while(TimeNow()<timeEnd){
		//Do Stuff while rotating
	}
}
void rotateTo(float heading){
	currentHeading=0/*ROBOT HEADING*/;
	if(abs(heading-currentHeading)<180){
		rotateBy(heading-currentHeading);
	}else{
		rotateBy((heading-currenHeading)-360);
	}
	//Maybe check and adjust?
}
void moveBlind(float angle, float distance){
	float speed = moveAtAngleRelCourse(angle, 1.0);
	float endTime = TimeNow()+distance/Speed;
	while(timeNow()<endTime){
		//Do stuff while moving maybe
	}
}
void moveTo(float x, float y){
	float speed = moveAtAngleRelCourse(atan2(y-currentY, x-currentx), 1.0);
	//Algorithm? Adjust course halfway through? Within certain distance, go blind?
}

void halt(){
  setWheels(0,0,0,0);
}
