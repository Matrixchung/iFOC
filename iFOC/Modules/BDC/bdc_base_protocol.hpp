#ifndef _BDC_BASE_PROTOCOL_H
#define _BDC_BASE_PROTOCOL_H

#include "bdc.hpp"
#include "base_protocol.hpp"

template<typename T1, typename T2, typename T3>
class BaseProtocol<BDC<T1, T2, T3>>
{
private:
    BDC<T1, T2, T3>& instance;
public:
    BaseProtocol(BDC<T1, T2, T3>& _ref, uint8_t _idx): instance(_ref), sub_dev_index(_idx) {};
    float GetEndpointValue(PROTOCOL_ENDPOINT endpoint);
    FOC_CMD_RET SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value);
    uint8_t node_id = 0x3F;
    uint8_t sub_dev_index = 0;
    uint64_t serial_number = 0;
};

template<typename T1, typename T2, typename T3>
float BaseProtocol<BDC<T1, T2, T3>>::GetEndpointValue(PROTOCOL_ENDPOINT endpoint)
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
        case DRIVE_ERROR_CODE:
            return (float)instance.error_code;
        case OUTPUT_STATE:
            return (float)instance.est_input.output_state;
        case DRIVE_MODE:
            return (float)instance.estimator.GetMode();
        case CURRENT_PHASE_A:
        case CURRENT_PHASE_B:
        case CURRENT_PHASE_C:
        case CURRENT_IALPHA:
        case CURRENT_IBETA:
        case CURRENT_IQ:
        case CURRENT_ID:
            return instance.est_input.Idc;
        case VQ_OUT:
        case VD_OUT:
        case VALPHA_OUT:
        case VBETA_OUT:
            return instance.est_output.Udc;
        case SPEED_SET:
            return instance.est_input.set_speed;
        case POS_ABS_SET_RAD:
            return instance.est_input.set_abs_pos;
        case POS_ABS_SET_DEG:
            return RAD2DEG(instance.est_input.set_abs_pos);
        case CURRENT_KP:
            return instance.estimator.Idc_PID.Kp;
        case CURRENT_KI:
            return instance.estimator.Idc_PID.Ki;
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
        case ESTIMATED_ANGLE_RAD:
            return instance.est_output.estimated_angle;
        case ELECTRIC_ANGLE_DEG:
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
            return instance.estimator.trajController.GetState();
        case TRAJ_TARGET_RAD:
            return instance.estimator.trajController.GetFinalPos();
        case TRAJ_TARGET_DEG:
            return RAD2DEG(instance.estimator.trajController.GetFinalPos());
        case TRAJ_CRUISE_SPEED:
            return instance.config.traj_cruise_speed;
        case TRAJ_MAX_ACCEL:
            return instance.config.traj_max_accel;
        case TRAJ_MAX_DECEL:
            return instance.config.traj_max_decel;
        case TRAJ_CURRENT_POS:
            return instance.estimator.trajController.GetPos();
        case TRAJ_CURRENT_SPEED:
            return instance.estimator.trajController.GetSpeed();
        case TRAJ_CURRENT_ACCEL:
            return instance.estimator.trajController.GetAccel();
        default: break;
    }
    return 0.0f;
}

template<typename T1, typename T2, typename T3>
FOC_CMD_RET BaseProtocol<BDC<T1, T2, T3>>::SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value)
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
            instance.SetOutputState((bool)((uint8_t)set_value == 1));
            break;
        case DRIVE_MODE:
            if((uint8_t)set_value <= 3) result = instance.estimator.SetMode((FOC_MODE)set_value);
            else result = CMD_NOT_SUPPORTED;
            break;
        // case IQ_SET:
        //     instance.est_input.Iqd_set.q = set_value;
        //     break;
        // case ID_SET:
        //     instance.est_input.Iqd_set.d = set_value;
        //     break;
        case IQ_SET:
        case ID_SET:
            instance.est_output.Udc = set_value;
            break;
        case SPEED_SET:
            instance.est_input.set_speed = set_value;
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
            instance.estimator.Idc_PID.Kp = set_value;
            break;
        case CURRENT_KI:
            instance.config.current_ki = set_value;
            instance.estimator.Idc_PID.Ki = set_value;
            break;
        case VPHASE_LIMIT:
            instance.config.Vphase_limit = set_value;
            instance.estimator.Idc_PID.limit = set_value;
            break;
        case CURRENT_RAMP_LIMIT:
            instance.config.current_ramp_limit = set_value;
            instance.estimator.Idc_PID.ramp_limit = set_value;
            break;
        case SPEED_KP:
            instance.config.speed_kp = set_value;
            instance.estimator.Speed_PID.Kp = set_value;
            break;
        case SPEED_KI:
            instance.config.speed_ki = set_value;
            instance.estimator.Speed_PID.Ki = set_value;
            break;
        case SPEED_KD:
            instance.config.speed_kd = set_value;
            instance.estimator.Speed_PID.Kd = set_value;
            break;
        case SPEED_CURRENT_LIMIT:
            instance.config.speed_current_limit = set_value;
            instance.estimator.Speed_PID.limit = set_value;
            break;
        case SPEED_RAMP_LIMIT:
            instance.config.speed_ramp_limit = set_value;
            instance.estimator.Speed_PID.ramp_limit = set_value;
            break;
        case POS_KP:
            instance.config.position_kp = set_value;
            instance.estimator.Position_PID.Kp = set_value;
            break;
        case POS_KI:
            instance.config.position_ki = set_value;
            instance.estimator.Position_PID.Ki = set_value;
            break;
        case POS_KD:
            instance.config.position_kd = set_value;
            instance.estimator.Position_PID.Kd = set_value;
            break;
        case POS_SPEED_LIMIT:
            instance.config.position_speed_limit = set_value;
            instance.estimator.Position_PID.limit = set_value;
            break;
        case POS_RAMP_LIMIT:
            instance.config.position_ramp_limit = set_value;
            instance.estimator.Position_PID.ramp_limit = set_value;
            break;
        case GO_HOME:
            instance.estimator.SetSubMode(SUBMODE_HOME);
            break;
        case BREAK_MODE:
            instance.config.break_mode = (uint8_t)set_value;
            break;
        case TRAJ_TARGET_RAD:
            if(!instance.estimator.trajController.GetState())
            {
                result = CMD_FORBIDDEN;
                break;
            }
            instance.estimator.trajController.PlanTrajectory(set_value, 
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