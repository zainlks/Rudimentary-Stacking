void KillDrive()
{
	motor[driveR1] = 0;
	motor[driveR2] = 0;
	motor[driveL1] = 0;
	motor[driveL2] = 0;
}
void setLeft(int speed)
{
	motor[driveL1] = speed;
	motor[driveL2] = speed;
}
void setRight(int speed)
{
	motor[driveR1] = speed;
	motor[driveR2] = speed;
}
void setDrive(int speed)
{
	setLeft(speed);
	setRight(speed);
}
//void turnLeft(int speed, int targetDegrees)
//{
//	while(trackingL<targetDegrees)
//}
// reset encoder function
void resetTracking()
{
	SensorValue[trackR] = 0;
	SensorValue[trackL] = 0;
}
