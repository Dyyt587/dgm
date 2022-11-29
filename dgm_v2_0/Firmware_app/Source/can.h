/*
	Copyright 2021 codenocold 1107795287@qq.com
	Address : https://github.com/codenocold/dgm
	This file is part of the dgm firmware.
	The dgm firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	The dgm firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __CAN_H__
#define __CAN_H__

#include "main.h"
#include <stdbool.h>

typedef enum eCanCmd{
	CAN_CMD_MOTOR_DISABLE = 0,
	CAN_CMD_MOTOR_ENABLE,
	
	CAN_CMD_ERROR_REPORT,
	CAN_CMD_ERROR_RESET,
	
	CAN_CMD_GET_STAT,
	
	CAN_CMD_CALIBRATION_START,
	CAN_CMD_CALIBRATION_REPORT,
	CAN_CMD_CALIBRATION_ABORT,
	
	CAN_CMD_ANTICOGGING_START,
	CAN_CMD_ANTICOGGING_REPORT,
	CAN_CMD_ANTICOGGING_ABORT,
	
	CAN_CMD_SYNC,
	
	CAN_CMD_SET_TARGET_POSITION,
	CAN_CMD_SET_TARGET_VELOCITY,
	CAN_CMD_SET_TARGET_CURRENT,
	
	CAN_CMD_GET_POSITION,
	CAN_CMD_GET_VELOCITY,
	CAN_CMD_GET_CURRENT,
	CAN_CMD_GET_VBUS,
	CAN_CMD_GET_IBUS,
	
	CAN_CMD_SET_CONFIG,
	CAN_CMD_GET_CONFIG,
	CAN_CMD_UPDATE_CONFIGS,
	CAN_CMD_RESET_ALL_CONFIGS,
	
	CAN_CMD_GET_FW_VERSION,
	
	CAN_CMD_HEARTBEAT,
}tCanCmd;

typedef enum eCanConfigs{
	CAN_CONFIG_MOTOR_POLE_PAIRS = 1,
	CAN_CONFIG_MOTOR_PHASE_RESISTANCE,
	CAN_CONFIG_MOTOR_PHASE_INDUCTANCE,
	CAN_CONFIG_INERTIA,
	
	CAN_CONFIG_CALIB_VALID,
	CAN_CONFIG_CALIB_CURRENT,
	CAN_CONFIG_CALIB_MAX_VOLTAGE,
	
	CAN_CONFIG_ANTICOGGING_ENABLE,
	CAN_CONFIG_ANTICOGGING_POS_THRESHOLD,
	CAN_CONFIG_ANTICOGGING_VEL_THRESHOLD,
	
	CAN_CONFIG_CONTROL_MODE,
	CAN_CONFIG_CURRENT_RAMP_RATE,
	CAN_CONFIG_VEL_RAMP_RATE,
	CAN_CONFIG_TRAJ_VEL,
	CAN_CONFIG_TRAJ_ACCEL,
	CAN_CONFIG_TRAJ_DECEL,
	CAN_CONFIG_POS_GAIN,
	CAN_CONFIG_VEL_GAIN,
	CAN_CONFIG_VEL_INTEGRATOR_GAIN,
	CAN_CONFIG_VEL_LIMIT,
	CAN_CONFIG_CURRENT_LIMIT,
	CAN_CONFIG_CURRENT_CTRL_P_GAIN,
	CAN_CONFIG_CURRENT_CTRL_I_GAIN,
	CAN_CONFIG_CURRENT_CTRL_BW,
	
	CAN_CONFIG_PROTECT_UNDER_VOLTAGE,
	CAN_CONFIG_PROTECT_OVER_VOLTAGE,
	CAN_CONFIG_PROTECT_OVER_SPEED,
	
	CAN_CONFIG_CAN_ID,
	CAN_CONFIG_CAN_TIMEOUT_MS,
	CAN_CONFIG_CAN_SYNC_TARGET_ENABLE,
} tCanConfigs;

typedef struct {
	uint32_t can_id;
	uint8_t can_dlc;
	uint8_t data[8];
} CanFrame;

void CAN_init(void);
bool can_send(uint32_t frameID, uint8_t* pData, uint8_t len);
bool CAN_receive(CanFrame *rx_frame);
void CAN_report_error(int32_t ecode);
void CAN_report_calibration(int step, uint8_t* data);
void CAN_report_anticogging(int step, uint8_t* data);
void CAN_reset_timeout(void);
void CAN_timeout_check_loop(void);
void CAN_rx_callback(CanFrame* rx_frame);

#endif /* __CAN_H__ */
