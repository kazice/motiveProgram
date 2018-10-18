/*
 * lib_trajectory_planning.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: YanWei
 */

#include <mav_client/lib_trajectory_planning.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

TRA_PLAN Tra_Plan_X;
TRA_PLAN Tra_Plan_Y;
TRA_PLAN Tra_Plan_Z;

void CalTraPlanning(TRA_PLAN *pTraPlan)
{
	float flVeltMax;
	if(pTraPlan->enTraState == TRA_FIRST)
	{
		pTraPlan->flLength = FLOAT_ABS(pTraPlan->flEndPos - pTraPlan->flStartPos);
		flVeltMax = sqrt(pTraPlan->flLength/(1/(2*pTraPlan->flAccUp)+1/(2*pTraPlan->flAccDown)));
		if(pTraPlan->flVeltMax > flVeltMax) //判断能达到的最大速度是否低于设定速度，修正路径最大速度
		{
			pTraPlan->flVeltMax = flVeltMax;
		}
		pTraPlan->flUpLength = FLOAT_ABS(pTraPlan->flVeltMax*pTraPlan->flVeltMax/(2*pTraPlan->flAccUp));//加速距离
		pTraPlan->flDownLength = FLOAT_ABS(pTraPlan->flVeltMax*pTraPlan->flVeltMax/(2*pTraPlan->flAccDown));
		if(pTraPlan->flEndPos - pTraPlan->flStartPos < 0)
		{
			pTraPlan->flVeltMax = -pTraPlan->flVeltMax;
			pTraPlan->flAccUp = -pTraPlan->flAccUp;
			pTraPlan->flAccDown = -pTraPlan->flAccDown;
		}
		pTraPlan->flTime = 0;
		pTraPlan->flDesPosOut = pTraPlan->flStartPos;
		//printf("%f\n",5*0.5*FLOAT_ABS(pTraPlan->flAccDown)*1/(pTraPlan->flHz*pTraPlan->flHz));
		printf("Misson start! \nThe Velt Max is:%f\nThe Start Pos is:%f\nThe End Pos is:%f\nThe Length is:%f\nThe Uplength is:%f\nThe DownLength is:%f\n"
				,pTraPlan->flVeltMax,pTraPlan->flStartPos,pTraPlan->flEndPos,pTraPlan->flLength,pTraPlan->flUpLength,pTraPlan->flDownLength);
		pTraPlan->enTraState = TRA_RUN;
		pTraPlan->enTraRunState = TRA_DOWN_FIRST;
	}
	else if(pTraPlan->enTraState == TRA_RUN)
	{
		if(FLOAT_ABS(pTraPlan->flDesPosOut - pTraPlan->flStartPos) <= pTraPlan->flUpLength) //处于加速阶段
		{
			pTraPlan->flTime = pTraPlan->flTime + 1/pTraPlan->flHz;
			pTraPlan->flDesPosOut = pTraPlan->flStartPos + 0.5*pTraPlan->flAccUp*pTraPlan->flTime*pTraPlan->flTime;
		}
		else if(FLOAT_ABS(pTraPlan->flDesPosOut-pTraPlan->flStartPos) > pTraPlan->flUpLength
		&& FLOAT_ABS(pTraPlan->flDesPosOut-pTraPlan->flStartPos) < (pTraPlan->flLength-pTraPlan->flDownLength)) //处于匀速阶段
		{
			pTraPlan->flDesPosOut = pTraPlan->flDesPosOut + pTraPlan->flVeltMax*1/pTraPlan->flHz;
		}
		else if(FLOAT_ABS(pTraPlan->flDesPosOut-pTraPlan->flStartPos) >= (pTraPlan->flLength-pTraPlan->flDownLength)) //处于减速阶段
		{
			if(pTraPlan->enTraRunState == TRA_DOWN_FIRST)
			{
				pTraPlan->flTime = 0;
				pTraPlan->enTraRunState = TRA_DOWN_RUN;
			}
			pTraPlan->flTime = pTraPlan->flTime + 1/pTraPlan->flHz;
			//printf("%f\t%f\t%f\n",pTraPlan->flVeltMax,FLOAT_ABS(pTraPlan->flDesPosOut - pTraPlan->flEndPos),pTraPlan->flAccDown*(pTraPlan->flTime-1/pTraPlan->flHz));
			pTraPlan->flDesPosOut = pTraPlan->flDesPosOut + 0.5*(pTraPlan->flVeltMax-pTraPlan->flAccDown*pTraPlan->flTime
			+pTraPlan->flVeltMax-pTraPlan->flAccDown*(pTraPlan->flTime-1/pTraPlan->flHz))*1/pTraPlan->flHz;
		}
		if(FLOAT_ABS(pTraPlan->flDesPosOut - pTraPlan->flEndPos) < END_POS)
		{
			pTraPlan->flDesPosOut = pTraPlan->flEndPos;
			pTraPlan->enTraState = TRA_DONE;
			printf("Misson done!\n");
		}
	}
	else if(pTraPlan->enTraState == TRA_DONE)
	{
		pTraPlan->flDesPosOut = pTraPlan->flEndPos;
		//pTraPlan->enTraState = TRA_NEVER;
	}
}


float FLOAT_ABS(float flInput)
{
	if(flInput < 0)
	{
		return -flInput;
	}
	else
	{
		return flInput;
	}
}
