#pragma config(Sensor, in1,    autoPoti,       sensorPotentiometer)
#pragma config(Sensor, in2,    mobilePoti,     sensorPotentiometer)
#pragma config(Sensor, in3,    liftPoti,       sensorPotentiometer)
#pragma config(Sensor, in4,    armPoti,        sensorPotentiometer)
#pragma config(Sensor, in5,    limLift,         sensorAnalog)
#pragma config(Sensor, in6,    lsBarL,         sensorReflection)
#pragma config(Sensor, in7,    lsBarR,         sensorReflection)
#pragma config(Sensor, in8,    lsMobile,       sensorReflection)
#pragma config(Sensor, dgtl1,  trackL,         sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  trackR,         sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  trackB,         sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  sonarL,         sensorSONAR_mm)
#pragma config(Sensor, dgtl9,  limArm,         sensorTouch)
#pragma config(Sensor, dgtl10, jmpSkills,      sensorDigitalIn)
#pragma config(Sensor, dgtl11, sonarR,         sensorSONAR_mm)
#pragma config(Motor,  port2,           liftR,         tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           driveL1,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           driveL2,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           arm,           tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           mobile,        tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port7,           driveR2,       tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           driveR1,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           liftL,         tmotorVex393HighSpeed_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
#include "defines.h"
#include "armliftfunctions.h"
#include "drive.h"
#include "risingfalling.h"
// STACKING ON                     0     1     2     3     4     5     6     7     8     9     10    11
const int gLiftRaiseTarget[12] = { 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2400, LIFT_TOP };
const int gLiftPlaceTarget[12] = { 1150, 1150, 1200, 1310, 1450, 1550, 1640, 1750, 1820, 1950, 2070, 2200 };
const int gLiftRaiseTargetS[7] = { 1850, 1920, 2020, 2120, 2240, 2380, LIFT_TOP };
const int gLiftPlaceTargetS[7] = { 1560, 1660, 1750, 1860, 1960, 2100, 2220 };
bool ManualDrive = true;
int gStackCount = 0;
bool gStacking = false;
task stackStack()
{
	LiftLowerSimple(LIFT_BOTTOM);
	ArmLowerSimple(ARM_BOTTOM, true);
	ArmRaiseSimple(ARM_CARRY, false);
	return;
}
task DriveMotors()
{
	setLeft(vexRT(JOY_THROTTLE)+vexRT(JOY_TURN));
	setRight(vexRT(JOY_THROTTLE)-vexRT(JOY_TURN));
}
task ManualLift()
{
}
task main()
{
	while(true)
	{
		//Rising Edge of Stack Button
		startTask(DriveMotors);
		risingMobileMiddle();
		risingMobile();
        updateStackBtn();
        if(btnStackRising())
        {
            startTask(stackStack);
        }
	}
}
