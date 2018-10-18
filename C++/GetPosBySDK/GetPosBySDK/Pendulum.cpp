#include "stdafx.h"
#include "Pendulum.h"
#include "PIDController.h"
#include <math.h>

PENDULUM_STATE PendulumState = {0};
EN_PEND_RUN_STATE enPendulumRunState = P_NEVER;
PEND_PID_CRL PidPendulum_x = {0};
PEND_PID_CRL PidPendulum_y = {0};


void IsPendStart(void)
{
	if(enPendulumRunState == P_NEVER || enPendulumRunState == P_FAIL)
	{
		if(fabs(PendulumState.Attitude.flx * 180 /3.1416) <= 2 && fabs(PendulumState.Attitude.fly * 180 /3.1416) <= 2)
		{
			enPendulumRunState = P_FIRST; 
		}
	}
}

void IsPendFail(void)
{
	if(enPendulumRunState == P_RUN)
	{
		if(fabs(PendulumState.Attitude.flx * 180 /3.1416) >= 8 || fabs(PendulumState.Attitude.fly * 180 /3.1416) >= 8)
		{
			enPendulumRunState = P_FAIL;
			PidPendulum_y.flESum = 0;
			PidPendulum_y.flE = 0;
			PidPendulum_y.flPreE = 0;
			PidPendulum_y.flU = 0;

			PidPendulum_x.flESum = 0;
			PidPendulum_x.flE = 0;
			PidPendulum_x.flPreE = 0;
			PidPendulum_x.flU = 0;
		}
	}
}

void CalAbsPendPID(PEND_PID_CRL *pPendPidCrl)
{
	pPendPidCrl->flESum += pPendPidCrl->flE;
	pPendPidCrl->flU = pPendPidCrl->flP * pPendPidCrl->flE	 + pPendPidCrl->flI * pPendPidCrl->flESum + pPendPidCrl->flD * (pPendPidCrl->flE - pPendPidCrl->flPreE);
	pPendPidCrl->flPreE = pPendPidCrl->flE;	
}