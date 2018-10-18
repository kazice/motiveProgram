#include "stdafx.h"
#include "PIDController.h"

// PID控制量，pPidCrl指针
void CalAbsPID(PID_CRL *pPidCrl)
{
	pPidCrl->flESum += pPidCrl->flE;
	pPidCrl->flU = pPidCrl->flP * pPidCrl->flE	 + pPidCrl->flI * pPidCrl->flESum + pPidCrl->flD * (pPidCrl->flE - pPidCrl->flPreE);
	pPidCrl->flPreE = pPidCrl->flE;	
}

// 取值在[flMin,flMax]区间内
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
