#ifndef _FOC_CAN_PROTOCOL_OPCODES_ENUM_H
#define _FOC_CAN_PROTOCOL_OPCODES_ENUM_H

#include "stdint.h"
#include "stddef.h"

typedef enum CAN_OPCODES
{
    GET_VERSION = 0x00,
    HEARTBEAT = 0x01,
    ESTOP     = 0x02,
    GET_ERROR = 0x03,
    ACCESS_ENDPOINT = 0x04, // [0] - Opcode(0:read, 1:write), [1:2] - Endpoint_ID, [3] - Reserved, [4:7] - Value (float)
    RESPONSE_ENDPOINT = 0x05, // Response for READ process, [0] - Reserved0, [1:2] - Endpoint_ID, [3] - Reserved1, [4:7] - Value (float)
    ADDRESS = 0x06,
    SET_STATE = 0x07,
    GET_ESTIMATES = 0x09,
    SET_DRIVE_MODE = 0x0B,
    SET_INPUT_POS = 0x0C,
    SET_INPUT_VEL = 0x0D,
    SET_INPUT_TORQUE = 0x0E,
    SET_LIMITS = 0x0F,
    SET_TRAJ_VEL_LIMIT = 0x11,
    SET_TRAJ_ACCEL_LIMITS = 0x12,
    SET_TRAJ_INERTIA = 0x13,
    GET_IQ = 0x14,
    GET_TEMP = 0x15,
    REBOOT = 0x16,
    GET_BUS_SENSE = 0x17,
    CLEAR_ERRORS = 0x18,
    SET_ABS_POSITION = 0x19,
    SET_POS_PID_PARAMS = 0x1A,
    SET_VEL_PID_PARAMS = 0x1B,
    GET_TORQUES = 0x1C,
    GET_POWERS = 0x1D,
    HOME = 0x1E,
}CAN_OPCODES;

#endif