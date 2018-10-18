#pragma once
typedef struct
{
	float flx; 
	float fly; 
	float flz;   
}POS_PEND;

typedef struct
{
	float flx; 
	float fly; 
	float flz;   
}VELT_PEND;

typedef struct
{
	float flx; 
	float fly; 
	float flz;   
}ATTITUDE_PEND;

typedef struct
{
	POS_PEND Pos;
	VELT_PEND Velt;
	ATTITUDE_PEND Attitude;
}PENDULUM_STATE;

typedef enum
{
	P_NEVER,
	P_FIRST,
	P_RUN,
	P_FAIL,
	P_DONE

}EN_PEND_RUN_STATE;

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
}PEND_PID_CRL;

void IsPendStart(void);
void IsPendFail(void);
void CalAbsPendPID(PEND_PID_CRL *pPendPidCrl);

extern PENDULUM_STATE PendulumState;
extern EN_PEND_RUN_STATE enPendulumRunState;
extern PEND_PID_CRL PidPendulum_x;
extern PEND_PID_CRL PidPendulum_y;