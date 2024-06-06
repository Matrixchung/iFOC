// https://docs.odriverobotics.com/v/latest/guides/can-guide.html#can-endpoint-access
#ifndef _FOC_BASE_PROTOCOL_H
#define _FOC_BASE_PROTOCOL_H

#include "foc.hpp"
#include "foc_header.h"
#include "endpoints_enum.h"

// All protocols share same UID(aka node_id) externed from BaseProtocol, which fits the CANSimple Protocol ID
// ID range: 0x00 - 0x3E (0x3F is the unaddressed ID, also global broadcast ID)
// BaseProtocol only process "Endpoints"

PROTOCOL_ENDPOINT GetEndpointFromIndex(int i) { return static_cast<PROTOCOL_ENDPOINT>(i); }

template<typename T_FOC>
class BaseProtocol
{
private:
    T_FOC& instance;
public:
    BaseProtocol(T_FOC& _ref, uint8_t _idx): instance(_ref), sub_dev_index(_idx) {};
    float GetEndpointValue(PROTOCOL_ENDPOINT endpoint);
    FOC_CMD_RET SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value);
    uint8_t node_id = 0x3F;
    uint8_t sub_dev_index = 0; // used to mark the index among same physical device, for example motor1, motor2, ...
    uint64_t serial_number = 0;
};

template<typename U>
float BaseProtocol<U>::GetEndpointValue(PROTOCOL_ENDPOINT endpoint)
{
    switch(endpoint)
    {
        case NODE_ID:
            return (float)node_id;
        case VBUS:
            return instance.bus_sense.Vbus;
        case IBUS:
            return instance.bus_sense.Ibus;
        case POWER:
            break;
        // case SERIAL_NUMBER:
        //     return (float)serial_number;
        case DRIVE_ERROR_CODE:
            return (float)instance.error_code;
        case OUTPUT_STATE:
            return (float)instance.est_input.output_state;
        case DRIVE_MODE:
            return (float)instance.estimator->GetMode();
        case CURRENT_PHASE_A:
            return instance.current_sense.Iabc.a;
        case CURRENT_PHASE_B:
            return instance.current_sense.Iabc.b;
        case CURRENT_PHASE_C:
            return instance.current_sense.Iabc.c;
        case CURRENT_IALPHA:
            return instance.est_input.Ialphabeta_fb.alpha;
        case CURRENT_IBETA:
            return instance.est_input.Ialphabeta_fb.beta;
        case CURRENT_IQ:
            return instance.est_output.Iqd_fb.q;
        case CURRENT_ID:
            return instance.est_output.Iqd_fb.d;
        case VQ_OUT:
            return instance.est_output.Uqd.q;
        case VD_OUT:
            return instance.est_output.Uqd.d;
        case VALPHA_OUT:
            return instance.svpwm.Ualphabeta.alpha;
        case VBETA_OUT:
            return instance.svpwm.Ualphabeta.beta;
        case COMPARE_A:
            return (float)instance.svpwm.compare_a;
        case COMPARE_B:
            return (float)instance.svpwm.compare_b;
        case COMPARE_C:
            return (float)instance.svpwm.compare_c;
        case IQ_TARGET:
            return instance.est_input.Iqd_target.q;
        case ID_TARGET:
            return instance.est_input.Iqd_target.d;
        case IQ_SET:
            return instance.est_output.Iqd_set.q;
        case ID_SET:
            return instance.est_output.Iqd_set.d;
        case SPEED_TARGET:
            return instance.est_input.target_speed;
        case POS_ABS_SET_RAD:
            return instance.est_input.set_abs_pos;
        case POS_ABS_SET_DEG:
            return RAD2DEG(instance.est_input.set_abs_pos);
        case CURRENT_KP:
            return instance.estimator->Iq_PID.Kp;
        case CURRENT_KI:
            return instance.estimator->Iq_PID.Ki;
        case VPHASE_LIMIT:
            return instance.config.Vphase_limit;
        case CURRENT_RAMP_LIMIT:
            return instance.config.current_ramp_limit;
        case SPEED_KP:
            return instance.config.speed_kp;
        case SPEED_KI:
            return instance.config.speed_ki;
        case SPEED_KD:
            return instance.config.speed_kd;
        case SPEED_CURRENT_LIMIT:
            return instance.config.speed_current_limit;
        case SPEED_RAMP_LIMIT:
            return instance.config.speed_ramp_limit;
        case POS_KP:
            return instance.config.position_kp;
        case POS_KI:
            return instance.config.position_ki;
        case POS_KD:
            return instance.config.position_kd;
        case POS_SPEED_LIMIT:
            return instance.config.position_speed_limit;
        case POS_RAMP_LIMIT:
            return instance.config.position_ramp_limit;
        case ELECTRIC_ANGLE_RAD:
            return instance.est_output.electric_angle;
        case ELECTRIC_ANGLE_DEG:
            return RAD2DEG(instance.est_output.electric_angle);
        case ESTIMATED_ANGLE_RAD:
            return instance.est_output.estimated_angle;
        case ESTIMATED_ANGLE_DEG:
            return RAD2DEG(instance.est_output.estimated_angle);
        case ESTIMATED_RAW_ANGLE_RAD:
            return instance.est_output.estimated_raw_angle;
        case ESTIMATED_RAW_ANGLE_DEG:
            return RAD2DEG(instance.est_output.estimated_raw_angle);
        case ESTIMATED_SPEED:
            return instance.est_output.estimated_speed;
        case ESTIMATED_ACCELERATION:
            return instance.est_output.estimated_acceleration;
        case BREAK_MODE:
            return instance.config.break_mode;
        case TRAJ_STATE:
            return instance.estimator->trajController.GetState();
        case TRAJ_TARGET_RAD:
            return instance.estimator->trajController.GetFinalPos();
        case TRAJ_TARGET_DEG:
            return RAD2DEG(instance.estimator->trajController.GetFinalPos());
        case TRAJ_CRUISE_SPEED:
            return instance.config.traj_cruise_speed;
        case TRAJ_MAX_ACCEL:
            return instance.config.traj_max_accel;
        case TRAJ_MAX_DECEL:
            return instance.config.traj_max_decel;
        case TRAJ_CURRENT_POS:
            return instance.estimator->trajController.GetPos();
        case TRAJ_CURRENT_SPEED:
            return instance.estimator->trajController.GetSpeed();
        case TRAJ_CURRENT_ACCEL:
            return instance.estimator->trajController.GetAccel();
        default: break;
    }
    return 0.0f;
}
template<typename U>
FOC_CMD_RET BaseProtocol<U>::SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value)
{
    FOC_CMD_RET result = CMD_SUCCESS;
    switch(endpoint)
    {
        case NODE_ID:
            if(set_value > 63)
            {
                // node_id_configured = false;
                result = CMD_NOT_SUPPORTED;
            }
            else
            {
                node_id = (uint8_t)set_value;
                // node_id_configured = true;
            }
            break;
        // case SERIAL_NUMBER:
        //     if(serial_number == 0) // can be set only once from startup
        //     {
        //         serial_number = (uint64_t)set_value;
        //     }
        //     else
        //     {
        //         result = CMD_FORBIDDEN;
        //     }
        //     break;
        case OUTPUT_STATE:
            // instance.est_input.output_state = ((uint8_t)set_value == 1);
            // if(!instance.est_input.output_state)
            // {
            //     instance.EmergencyStop();
            // }
            // else instance.driver.EnableOutput();
            instance.SetOutputState((bool)((uint8_t)set_value == 1));
            break;
        case DRIVE_MODE:
            if((uint8_t)set_value <= 3) result = instance.estimator->SetMode((FOC_MODE)set_value);
            else result = CMD_NOT_SUPPORTED;
            break;
        case IQ_TARGET:
            instance.est_input.Iqd_target.q = set_value;
            break;
        case ID_TARGET:
            instance.est_input.Iqd_target.d = set_value;
            break;
        case SPEED_TARGET:
            instance.est_input.target_speed = set_value;
            break;
        case POS_ABS_SET_RAD:
            instance.est_input.set_abs_pos = set_value;
            break;
        case POS_INC_RAD:
            instance.est_input.set_abs_pos += set_value;
            break;
        case POS_ABS_SET_DEG:
            instance.est_input.set_abs_pos = DEG2RAD(set_value);
            break;
        case POS_INC_DEG:
            instance.est_input.set_abs_pos += DEG2RAD(set_value);
            break;
        case CURRENT_KP:
            instance.config.current_kp = set_value;
            instance.estimator->Iq_PID.Kp = set_value;
            instance.estimator->Id_PID.Kp = set_value;
            break;
        case CURRENT_KI:
            instance.config.current_ki = set_value;
            instance.estimator->Iq_PID.Ki = set_value;
            instance.estimator->Id_PID.Ki = set_value;
            break;
        case VPHASE_LIMIT:
            instance.config.Vphase_limit = set_value;
            instance.estimator->Iq_PID.limit = set_value;
            instance.estimator->Id_PID.limit = set_value;
            break;
        case CURRENT_RAMP_LIMIT:
            instance.config.current_ramp_limit = set_value;
            instance.estimator->Iq_PID.ramp_limit = set_value;
            instance.estimator->Id_PID.ramp_limit = set_value;
            break;
        case SPEED_KP:
            instance.config.speed_kp = set_value;
            instance.estimator->Speed_PID.Kp = set_value;
            break;
        case SPEED_KI:
            instance.config.speed_ki = set_value;
            instance.estimator->Speed_PID.Ki = set_value;
            break;
        case SPEED_KD:
            instance.config.speed_kd = set_value;
            instance.estimator->Speed_PID.Kd = set_value;
            break;
        case SPEED_CURRENT_LIMIT:
            instance.config.speed_current_limit = set_value;
            instance.estimator->Speed_PID.limit = set_value;
            break;
        case SPEED_RAMP_LIMIT:
            instance.config.speed_ramp_limit = set_value;
            instance.estimator->Speed_PID.ramp_limit = set_value;
            break;
        case POS_KP:
            instance.config.position_kp = set_value;
            instance.estimator->Position_PID.Kp = set_value;
            break;
        case POS_KI:
            instance.config.position_ki = set_value;
            instance.estimator->Position_PID.Ki = set_value;
            break;
        case POS_KD:
            instance.config.position_kd = set_value;
            instance.estimator->Position_PID.Kd = set_value;
            break;
        case POS_SPEED_LIMIT:
            instance.config.position_speed_limit = set_value;
            instance.estimator->Position_PID.limit = set_value;
            break;
        case POS_RAMP_LIMIT:
            instance.config.position_ramp_limit = set_value;
            instance.estimator->Position_PID.ramp_limit = set_value;
            break;
        case GO_HOME:
            instance.estimator->SetSubMode(SUBMODE_HOME);
            break;
        case BREAK_MODE:
            instance.config.break_mode = (uint8_t)set_value;
            break;
        case TRAJ_TARGET_RAD:
            if(!instance.estimator->trajController.GetState())
            {
                result = CMD_FORBIDDEN;
                break;
            }
            instance.estimator->trajController.PlanTrajectory(set_value, 
                                                            instance.est_output.estimated_raw_angle, 
                                                            instance.est_output.estimated_speed, 
                                                            instance.config.traj_cruise_speed,
                                                            instance.config.traj_max_accel,
                                                            instance.config.traj_max_decel);
            break;
        case TRAJ_TARGET_DEG:
            return SetEndpointValue(TRAJ_TARGET_RAD, DEG2RAD(set_value));
        case TRAJ_CRUISE_SPEED:
            instance.config.traj_cruise_speed = set_value;
            break;
        case TRAJ_MAX_ACCEL:
            instance.config.traj_max_accel = set_value;
            break;
        case TRAJ_MAX_DECEL:
            instance.config.traj_max_decel = set_value;
            break;
        default: 
            result = CMD_FORBIDDEN; 
            break;
    }
    return result;
}

#endif