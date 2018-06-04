#define JOY_TURN Ch4
#define DZ_TURN 15
#define JOY_THROTTLE Ch3
#define DZ_THROTTLE 15
//Include files
//#include "controls.h"
#define liftVal SensorValue[liftPoti]
#define armVal SensorValue[armPoti]
#define trackingL SensorValue[trackL]
#define trackingR SensorValue[trackR]
#define JOY_DEAD abs(JOY_THROTTLE)<DZ_THROTTLE+5 & abs(JOY_THROTTLE)<DZ_TURN+5
//Lift positions
#define LIFT_BOTTOM 1100
#define LIFT_TOP (LIFT_BOTTOM + 1650)
#define LIFT_MID (LIFT_BOTTOM + 620)
#define LIFT_HOLD_DOWN_THRESHOLD (LIFT_BOTTOM + 50)
#define LIFT_HOLD_UP_THRESHOLD (LIFT_TOP - 100)
#define LIFT_LOADER (LIFT_BOTTOM + 800)
#define LIFT_LOADER_PICKUP (LIFT_BOTTOM + 480)
#define LIFT_RETURN (LIFT_BOTTOM + 400)
#define LIFT_WALL (LIFT_BOTTOM + 450)

//Arm Positions
#define RL_ARM_TOP 2880
#define ARM_TOP (RL_ARM_TOP - 100)

//Actual ARM_BOTTOM = 1020
#define ARM_BOTTOM (RL_ARM_TOP - 1630)

#define ARM_PRESTACK (RL_ARM_TOP - 800)
#define ARM_RELEASE (RL_ARM_TOP - 700)
#define ARM_CARRY (RL_ARM_TOP - 1040)
#define ARM_STACK (RL_ARM_TOP - 100)
#define ARM_HORIZONTAL (RL_ARM_TOP - 1590)
#define ARM_HOLD_DOWN_THRESHOLD (RL_ARM_TOP - 1550)

//Podi Position Tolerance
#define armTolerance 100
#define liftTolerance 100

//Mobile Positions
#define MOBILE_TOP 2450
#define MOBILE_BOTTOM 800
#define MOBILE_MIDDLE_UP 1000
#define MOBILE_MIDDLE_DOWN 1400
#define MOBILE_MIDDLE_THRESHOLD 2100
#define MOBILE_HALFWAY 1200
