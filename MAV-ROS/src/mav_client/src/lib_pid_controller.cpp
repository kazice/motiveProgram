#include <mav_client/lib_pid_controller.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void CalAbsPID(PID_CRL *pPidCrl)
{
	pPidCrl->flESum += pPidCrl->flE;
	if(pPidCrl->chEflag == 0)
	{
		pPidCrl->flESum = 0;
	}
	pPidCrl->flDE = pPidCrl->flD * (pPidCrl->flE - pPidCrl->flPreE);
	pPidCrl->flU = pPidCrl->flP * pPidCrl->flE	 + pPidCrl->flI * pPidCrl->flESum + pPidCrl->flD * (pPidCrl->flE - pPidCrl->flPreE);
	pPidCrl->flPreE = pPidCrl->flE;
}

void CalAbsPlusPID(PID_CRL *pPidCrl)
{
	float Sum1 = 0;
	float Sum2 = 0;
	pPidCrl->flESum += pPidCrl->flE;
	if(pPidCrl->chEflag == 0)
	{
		pPidCrl->flESum = 0;
	}
	pPidCrl->flNPreE[0] = pPidCrl->flE;
	for(int j_pid=0;j_pid<N_PRE/2;j_pid++)
	{
		Sum1 = Sum1 + pPidCrl->flNPreE[j_pid];
		Sum2 = Sum2 + pPidCrl->flNPreE[j_pid + N_PRE/2];
	}
	pPidCrl->flDE = pPidCrl->flD * ((Sum1 - Sum2) / N_PRE * 2);
	pPidCrl->flU = pPidCrl->flP * pPidCrl->flE	 + pPidCrl->flI * pPidCrl->flESum + pPidCrl->flD * ((Sum1 - Sum2) / N_PRE * 2);
	for(int i_pid=0;i_pid<N_PRE-1;i_pid++)
	{
		pPidCrl->flNPreE[i_pid+1] = pPidCrl->flNPreE[i_pid];
	}
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

float SgnFloat(float data)
{
	if(data > 0)
	{
		return 1;
	}
	else if(data < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}
