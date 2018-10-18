#include <mav_client/lib_pid_controller.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void CalAbsPID(PID_CRL *pPidCrl)
{
	pPidCrl->flESum += pPidCrl->flE;
	pPidCrl->flU = pPidCrl->flP * pPidCrl->flE	 + pPidCrl->flI * pPidCrl->flESum + pPidCrl->flD * (pPidCrl->flE - pPidCrl->flPreE);
	pPidCrl->flPreE = pPidCrl->flE;
}

void CalIncPID(PID_CRL *pPidCrl)
{

	pPidCrl->flU += pPidCrl->flP * (pPidCrl->flE - pPidCrl->flPreE) + pPidCrl->flI * pPidCrl->flE + pPidCrl->flD * (pPidCrl->flE - 2 * pPidCrl->flPreE + pPidCrl->flPrePreE);
	pPidCrl->flPrePreE = pPidCrl->flPreE;
	pPidCrl->flPreE = pPidCrl->flE;
}


float ClipFloat(float flValue,float flMin,float flMax)
{
	if(flValue < flMin)
	{
		return flMin;
	}
	else if(flValue > flMax)
	{
		return flMax;
	}
	else
	{
		return flValue;
	}
}

short ClipShort(short shValue,short shMin,short shMax)
{
	if(shValue < shMin)
	{
		return shMin;
	}
	else if(shValue > shMax)
	{
		return shMax;
	}
	else
	{
		return shValue;
	}
}

float _ltos(float data)
{
	FLOAT_CONV d1,d2;
	d1.f = data;

	d2.c[0] = d1.c[3];
	d2.c[1] = d1.c[2];
	d2.c[2] = d1.c[1];
	d2.c[3] = d1.c[0];

	return d2.f;
}
