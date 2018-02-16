#include <math.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
using namespace std;

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

void setVelocityComponents(float right, float forward, fload speedPercent){
  //arguments between -1 and 1
	//Ex: setVelocityComponents(-1, 1, 0.75);
  //  will set robot going forward and to the left at 75% Speed
  
  float fl=forward+right;
  float fr=-forward+right;
  float br=-forward-right;
  float bl=forward-right;
  
	//Scale back so max speed and no overflow:
  float m=max( max(abs(fr), abs(fl)), max(abs(bl), abs(br)) );
  if(abs(m)<.001){
    fl\=m; fr\=m; br\=m; bl\=m;
  }
  setWheels(fl*speedPercent,fr*speedPercent,bl*speedPercent,br*speedPercent);
}

void moveRelRobot(float heading, float speedPercent){
	setVelocityComponents(cos heading, sin heading, 1.0);
}

void moveRelCourse(float heading, float speedPercent){
	setRobotHeading(heading-robotHeading, float speedPercent)
}
