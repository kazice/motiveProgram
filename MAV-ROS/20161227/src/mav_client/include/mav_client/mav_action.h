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

extern POS_MAV realPosMAV;
extern ATT_MAV realAttMAV;
extern VELT_MAV realVeltMAV;

extern POS_MAV desPosMAV;
extern VELT_MAV desVeltMAV;
extern ATT_MAV desAttMAV;

extern PID_CRL_MAV pidCrlMAV;

extern EN_RUN_STATE enTakeoffState;
extern mavros_msgs::OverrideRCIn RC_controller_output;
void mav_takeoff(void);
void mav_land(void);
void data_init(void);
void mav_takeoff_data_clear(void);


#endif /* SRC_MAV_CLIENT_SRC_MAV_ACTION_H_ */
