/*
 * mav_action.h
 *
 *  Created on: Dec 6, 2016
 *      Author: yw
 */

#ifndef __MAV_ACTION_H__
#define __MAV_ACTION_H__

#include <mav_client/lib_pid_controller.h>
#include <mavros_msgs/OverrideRCIn.h>

#define NET_TIMEOUT 500 //ms

#define PENDULUM_START_X 3.0f
#define PENDULUM_START_Y 3.0f

#define PENDULUM_FAIL_X 6.0f
#define PENDULUM_FAIL_Y 6.0f

#define PENDULUM_LIMIT_X 0.4f
#define PENDULUM_LIMIT_Y 0.4f

#define pi 3.14159265357f

typedef enum
{
	MAV_NEVER,
	MAV_UNLOCK,
	MAV_LOCK,
	MAV_INIT,
	MAV_READY,
	MAV_TAKEOFF,
	MAV_LAND,
	MAV_HOVER,
	MAV_PENDULUM

}EN_MAV_STATE;
extern EN_MAV_STATE enMAVState;

extern POS_MAV realPosMAV;
extern ATT_MAV realAttMAV;
extern VELT_MAV realVeltMAV;
extern ATT_PENDULUM realAttPendulum;

extern POS_MAV desPosMAV;
extern VELT_MAV desVeltMAV;
extern ATT_MAV desAttMAV;
extern ATT_PENDULUM desAttPendulum;

extern bool bpendulumvalid;

extern PID_CRL_MAV pidCrlMAV;


extern EN_CONTROL_STATE enControlStateX;
extern EN_CONTROL_STATE enControlStateY;
extern EN_CONTROL_STATE enControlStateZ;

extern EN_RUN_STATE enTakeoffState;
extern mavros_msgs::OverrideRCIn RC_controller_output;
void mav_takeoff(void);
void mav_land(void);
void mav_hover(void);
void mav_pendulum(void);

void mav_pendulum_x(void); //for test
void mav_pendulum_y(void);

void is_pendulum_start(void);
bool is_pendulum_fail(void);
void pid_data_pos_velt_init(void);
void pid_data_pendulum_init(void);
void mav_takeoff_data_clear(void);
void mav_pendulum_data_clear(void);
void mav_posloopx_data_clear(void);
void mav_posloopy_data_clear(void);
void mav_posloopz_data_clear(void);
#endif /* SRC_MAV_CLIENT_SRC_MAV_ACTION_H_ */
