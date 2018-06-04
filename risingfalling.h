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
				//action
				mobileMiddleDown();
				shiftMobileMiddle=true;
			}
			else{shiftMobileMiddle = false;}
		}
}

void risingLiftBottom()
{

}
