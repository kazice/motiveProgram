/*
 * lib_pid_controller.h
 *
 *  Created on: Dec 6, 2016
 *      Author: yw
 */
#ifndef __LIB_PID_CONTROLLER_H__
#define __LIB_PID_CONTROLLER_H__

#define N_PRE 6


typedef struct
{
	float flP;
	float flI;
	float flD;
	float flE;
	char chEflag;
	float flPreE;
	float flDE; //for test
	float flNPreE[N_PRE]; //N point
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
	PID_CRL pidPendulumx;
	PID_CRL pidPendulumy;
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
}ATT_PENDULUM;

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
	POS_VELT,
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

void CalAbsPlusPID(PID_CRL *pPidCrl);

float SgnFloat(float data);
#endif /* SRC_MAV_CLIENT_SRC_LIB_PID_CONTROLLER_H_ */
