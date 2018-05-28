typedef struct _SetupJoy
{
    bool cur;
    bool last;
    int deadzone;
}  SetupJoy;


void risingAll()
{
    Mobile.last = Mobile.cur;
    
}


void risingMobile()
{
	bool shiftMobile = false;
	if(vexRT(Btn6U))
	{
		if(!shiftMobile)
		{
			mobileOuttake();
			shiftMobile = true;
		}
		else {shiftMobile= false;}
	}
}

void risingMobileMiddle()
{
		bool shiftMobileMiddle = false;
		if(vexRT(Btn6D))
		{
			if(!shiftMobileMiddle)
			{
				mobileMiddleDown();
				shiftMobileMiddle=true;
			}
			else{shiftMobileMiddle = false;}
		}
}

bool btnLastStack = false;
bool btnCurStack = false;

void updateStackBtn()
{
    btnLastStack = btnCurStack;
    btnCurStack = vexRT[Btn8D];
}

bool btnStackRising()
{
    return btnCurStack && !btnLastStack;
}

bool btnStackFalling()
{
    return !btnCurStack && btnLastStack
}
