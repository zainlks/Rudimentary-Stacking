#pragma config(Sensor, in1,    autoPoti,       sensorPotentiometer)
#pragma config(Sensor, in2,    mobilePoti,     sensorPotentiometer)
#pragma config(Sensor, in3,    liftPoti,       sensorPotentiometer)
#pragma config(Sensor, in4,    armPoti,        sensorPotentiometer)
#pragma config(Sensor, in5,    expander,       sensorAnalog)
#pragma config(Sensor, in6,    lsBarL,         sensorReflection)
#pragma config(Sensor, in7,    lsBarR,         sensorReflection)
#pragma config(Sensor, in8,    gyrorob,        sensorGyro)
#pragma config(Sensor, dgtl1,  trackL,         sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  trackR,         sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  trackB,         sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  sonarL,         sensorSONAR_mm)
#pragma config(Sensor, dgtl9,  limMobile,      sensorTouch)
#pragma config(Sensor, dgtl10, jmpSkills,      sensorDigitalIn)
#pragma config(Sensor, dgtl11, sonarR,         sensorSONAR_mm)
#pragma config(Motor,  port2,           liftL,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           driveL1,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           driveL2,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           arm,           tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           mobile,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           driveR2,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           driveR1,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           liftR,         tmotorVex393HighSpeed_MC29, openLoop)

#include "defines.h"
#include "armliftfunctions.h"
#include "drive.h"
bool gManualDrive = true;
task Backup()
{
	setDrive(-60);
	bool LeftTurn = false;
	bool RightTurn = false;
	while((trackingL<500) & (!gManualDrive))
	{
		if(vexRT(Btn5U)) {LeftTurn=true;}
		if(vexRT(Btn6U)) {RightTurn=true;}
	}
	if(LeftTurn){}
	if(RighTurn)
}
void WallBackup()
{
	bool shiftBackup = false;
	if(vexRT(Btn8L))
	{
		if(!shiftBackup)
		{

		}
	}
}
task main()
{
	WallBackup();
}
