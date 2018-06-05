#define JOY_TURN  vexRT(Ch4)
#define JOY_THROTTLE vexRT(Ch3)
#define JOY_STRAFE vexRT(Ch1)
task main()
{
    while(true)
    {
        if(abs(JOY_TURN) < 5)
        {
            
        }
        motor[frontLeft] = JOY_TURN + JOY_STRAFE + JOY_THROTTLE;
        motor[frontRight] = JOY_TURN + JOY_STRAFE - JOY_THROTTLE;
        motor[rearLeft] = JOY_TURN - JOY_STRAFE + JOY_THROTTLE;
        motor[rearRight] = JOY_TURN - JOY_STRAFE - JOY_THROTTLE;
    }
}
