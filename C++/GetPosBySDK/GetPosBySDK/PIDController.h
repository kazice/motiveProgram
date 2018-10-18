#pragma once



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
	float flx; 
	float fly; 
	float flz;   
}REAL_POS_UAV;

typedef struct
{
	float flx; 
	float fly; 
	float flz;   
}DES_POS_UAV;

typedef struct
{
	float flx; 
	float fly; 
	float flz;   
}DES_VELT_UAV;


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
float _ltos(float data);
float ClipFloat(float flValue,float flMin,float flMax);