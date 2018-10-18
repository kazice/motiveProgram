/*
 * lib_pid_controller.h
 *
 *  Created on: Dec 6, 2016
 *      Author: yw
 */
#ifndef __LIB_PID_CONTROLLER_H__
#define __LIB_PID_CONTROLLER_H__

typedef struct
{
	float flP;
	float flI;
	float flD;
	float flE;
	float flPreE;
	float flPrePreE;
	float flESum;
	float flU;
	float flELimit;
	float flULimit;
	float flEDead;
}PID_CRL;

typedef struct
{
	PID_CRL pidPosx;
	PID_CRL pidPosy;
	PID_CRL pidPosz;
	PID_CRL pidRoll;
	PID_CRL pidYaw;
	PID_CRL pidPitch;
	PID_CRL pidVeltx;
	PID_CRL pidVelty;
	PID_CRL pidVeltz;
}PID_CRL_MAV;

typedef struct
{
	float flx;
	float fly;
	float flz;
}POS_MAV;

typedef struct
{
	float flvx;
	float flvy;
	float flvz;
}VELT_MAV;

typedef struct
{
	float flroll;
	float flyaw;
	float flpitch;
}ATT_MAV;


typedef struct
{
	float flx;
	float fly;
	float flz;
}DES_VELT_MAV;


typedef enum
{
	NEVER,
	FIRST,
	RUN,
	DONE

}EN_RUN_STATE;

typedef enum
{
	OPEN,
	POS,
	VELT
}EN_CONTROL_STATE;

typedef union{
	float f;
	char c[4];
}FLOAT_CONV;

void CalAbsPID(PID_CRL *pPidCrl);
void CalIncPID(PID_CRL *pPidCrl); //zengliang
float _ltos(float data);
float ClipFloat(float flValue,float flMin,float flMax);
short ClipShort(short shValue,short shMin,short shMax);
#endif /* SRC_MAV_CLIENT_SRC_LIB_PID_CONTROLLER_H_ */
