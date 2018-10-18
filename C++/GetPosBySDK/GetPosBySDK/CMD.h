#pragma once


void CMD_deal(int cmd);
void resetUAV(void);
void Display_CMD_Menu(void);
void Display_CMD_Deal(void);

typedef struct
{
	bool Total;
	bool Pos;
	bool Velt;
	bool PosX;
	bool PosY;
	bool PosZ;
	bool VeltX;
	bool VeltY;
	bool VeltZ;
	bool OutputVeltX;
	bool OutputVeltY;
	bool OutputVeltZ;
	bool OutputAttitudeRoll;
	bool OutputAttitudePitch;
	bool OutputAttitudeYaw;
}DATA_DIS;


extern int c;
extern bool bExit;
extern int cmd_dis;
extern DATA_DIS bDataDisplay;
extern bool bMenuFlag;