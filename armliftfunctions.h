//Move Lift Functions
//Raise Lift Function
bool liftLimit()
{
	return SensorValue[limLift] < 200;
}
void setLift(word power,bool debug=true)
{
	if( debug ) writeDebugStreamLine("%06d Lift %4d", nPgmTime,power );
	motor[liftL] = motor[liftR] = power;
	writeDebugStreamLine("%d", power);
}
void setArm(word power,bool debug=true)
{
	if( debug ) writeDebugStreamLine("%06d Arm %4d", nPgmTime,power );
	motor[arm] = power;
	writeDebugStreamLine("%d", power);
}
void armHolding(int pos)
{
	if(SensorValue[armPoti] > pos +- armTolerance)
	setArm(-15);
	else if(SensorValue[armPoti] < pos +- armTolerance)
	setArm(15);
}
void GoToLimitLift()
{
	while(!liftLimit())
	{
		setLift(-127);
	}
	writeDebugStreamLine("Limit switch hit");
	setLift(-15);
}

void GoToLimitArm()
{
	while(!SensorValue[limArm])
	{
		setArm(-127);
	}
	writeDebugStreamLine("Limit Switch Value = %b", SensorValue[limArm]);
	setArm(-15);
}
void ArmRaiseSimple(int pos, bool hardstop)
{
	if(hardstop)
	{
		while(SensorValue[armPoti] < pos-100)
		{
		setArm(127);
		writeDebugStreamLine("Arm position set as %4d", "pos");
		}
		writeDebugStreamLine("Arm Reached position of %4d", "pos");
	}
	setArm(-15);
	if(!hardstop)
	{
		while(SensorValue[armPoti] < pos)
		{
		if(SensorValue[armPoti] < pos-400)
		setArm(127);
		else if(SensorValue[armPoti] < pos-200)
		setArm(60);
		else if(SensorValue[liftPoti] == pos+-100)
		armHolding(pos);
		}
	}
}
void ArmLowerSimple(int pos, bool hardstop)
{
	if(hardstop)
	{
		while(SensorValue[armPoti] > pos+100)
		{
		setArm(-127);
		writeDebugStreamLine("Arm position set as %4d", "pos");
		}
		writeDebugStreamLine("Arm Reached position of %4d", "pos");
		setArm(-15);
	}

	else if(!hardstop)
	{
		while(SensorValue[armPoti] > pos)
		{
		if(SensorValue[armPoti] > pos+400)
		setArm(-127);
		else if(SensorValue[armPoti] > pos+200)
		setArm(-60);
		else if(SensorValue[liftPoti] == pos+-100)
		armHolding(pos);
		}
	}
}
void LiftRaiseSimple(int pos)
{
	while(SensorValue[liftPoti] < pos)
	{
		if(SensorValue[liftPoti] < pos-400)
		setLift(127);
		else if(SensorValue[liftPoti] < pos-200)
		setLift(60);
		else if(SensorValue[liftPoti] == pos)
		{
			if(SensorValue[liftPoti] > pos)
			setLift(-15);
			else if(SensorValue[liftPoti] < pos)
			setLift(15);
		}
	}
}

void LiftLowerSimple(int pos)
{

	while(SensorValue[liftPoti] > pos)
	{
		if(SensorValue[liftPoti] > pos+400)
		setLift(-127);
		else if(SensorValue[liftPoti] > pos+200)
		setLift(-60);
	}
setLift(-15);
}

void setMobile(word power,bool debug=true)
{
	if( debug ) writeDebugStreamLine("%06d Mobile %4d", nPgmTime,power );
	motor[mobile] = power;
	writeDebugStreamLine("%d", power);
}

void mobileOuttake()
{
	if(SensorValue[mobilePoti]>MOBILE_HALFWAY)
				{
					while(SensorValue[mobilePoti]>MOBILE_BOTTOM)
					{
						setMobile(-127);
					}
					setMobile(-15);
				}
				else
				{
					while(SensorValue[mobilePoti]< MOBILE_TOP)
					{
						setMobile(127);
					}
					setMobile(15);
				}
}
void mobileMiddleDown()
{
	   while(SensorValue[mobilePoti] > MOBILE_MIDDLE_DOWN)
		{
			if(SensorValue[mobilePoti] > MOBILE_MIDDLE_DOWN+200)
			{setMobile(-127);}
			else
			{setMobile(-75);}
		}
		setMobile(15);
}
