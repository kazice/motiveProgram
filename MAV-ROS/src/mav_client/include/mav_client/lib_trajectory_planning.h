/*
 * lib_trajectory_planning.h
 *
 *  Created on: Feb 18, 2017
 *      Author: Yan Wei
 */

#ifndef LIB_TRAJECTORY_PLANNING_H_
#define LIB_TRAJECTORY_PLANNING_H_

#define END_POS 0.01f

typedef enum
{
	TRA_UP_FIRST,
	TRA_UP_RUN,
	TRA_DOWN_FIRST,
	TRA_DOWN_RUN,
}EN_TRA_RUN_STATE;

typedef enum
{
	TRA_NEVER,
	TRA_FIRST,
	TRA_RUN,
	TRA_DONE

}EN_TRA_STATE;

typedef struct
{
	float flStartPos;  	//Start of trajectory
	float flEndPos;		//DesPos of trajectory
	float flLength;
	float flUpLength;
	float flDownLength;
	float flPosNow;  	//Postion of MAV
	float flDesPosOut;	//The next despos of MAV output to autopliot
	float flAccUp;		//The Acc
	float flAccDown;
	float flVeltMax;	//The max velt of MAV
	float flHz;			//The frequency of control
	float flTime;
	EN_TRA_STATE enTraState;
	EN_TRA_RUN_STATE enTraRunState;
}TRA_PLAN;

extern TRA_PLAN Tra_Plan_X;
extern TRA_PLAN Tra_Plan_Y;
extern TRA_PLAN Tra_Plan_Z;

void CalTraPlanning(TRA_PLAN *pTraPlan);
float FLOAT_ABS(float flInput);

#endif /* LIB_TRAJECTORY_PLANNING_H_ */
