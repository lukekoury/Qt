#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
using namespace std;

#define PI_180 0.0174532925
#define SPEEDCONSTANT 1.0

void moveAtAngleRelRobot(float heading, float speedPercent)
void moveAtAngleRelCourse(float heading, float speedPercent);
void setRotation(float direction);
void halt();

FEHMotor motorFL(FEHMotor::Motor0,7.2);
FEHMotor motorFR(FEHMotor::Motor1,7.2);
FEHMotor motorBL(FEHMotor::Motor2,7.2);
FEHMotor motorBR(FEHMotor::Motor3,7.2);

void setWheels(float fl, float fr, float bl, float br){
  //Sets wheels to given speeds
  //TODO: include calibrations for wheels
  motorFL.setPercent(100.0*fl);
  motorFR.setPercent(100.0*fr);
  motorBL.setPercent(100.0*bl);
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
	return moveAtAngleRelRobot(heading-90/*Robot Orientation*/, speedPercent);
}

void setRotation(float direction){
  if(abs(direction)>1) direction/=abs(direction);
  setWheels(direction, direction, direction, direction);
}

void halt(){
  setWheels(0,0,0,0);
}
