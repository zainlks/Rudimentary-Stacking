//Move Lift Functions
//Raise Lift Function

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
	if(pos==ARM_TOP)
	setArm(15);
	else
	setArm(-15);
	if(!hardstop)
	{
		while(SensorValue[armPoti] < pos)
		{
		if(SensorValue[armPoti] < pos-400)
		setArm(127);
		else if(SensorValue[armPoti] < pos-200)
		setArm(60);
		else if(SensorValue[liftPoti] == pos-100)
		armHolding(pos);
		}
	}
}
void ArmLowerSimple(int pos, bool hardstop)
{
	while(SensorValue[armPoti] > pos+100)
		{
		setArm(-127);
		writeDebugStreamLine("Arm position set as %4d", "pos");
		}
		writeDebugStreamLine("Arm Reached position of %4d", "pos");
		if(pos==ARM_BOTTOM)
	setArm(-15);
	else
	setArm(15);

}
void LiftRaiseSimple(int pos, float kp, float ki, float kd)
{
	float error = 0;
	float last_error = 0;
	float integral = 0;
	float derivative = 0;
	unsigned long lastTime = nPgmTime;
	while(SensorValue[liftPoti] < pos)
	{
		error = pos-SensorValue[liftPoti];
		integral += (nPgmTime - lastTime) * error;
		derivative= (error - last_error)/(nPgmTime-lastTime);
		last_error = error;
		setLift(kp*error + ki*integral + kd*derivative);
		lastTime = nPgmTime;
		sleep(1);
	}

}



void LiftLowerSimple(int pos, float kp)
{
    float error = 0;
	while(SensorValue[liftPoti] > pos)
	{
        error = pos - SensorValue[liftPoti];
        setLift(error*kp)
	}
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
			{setMobile(-127);}
		}
		setMobile(15);
}
