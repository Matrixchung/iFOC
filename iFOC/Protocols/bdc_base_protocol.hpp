#ifndef _BDC_BASE_PROTOCOL_H
#define _BDC_BASE_PROTOCOL_H

#include "bdc.hpp"
#include "base_protocol.hpp"

template<class A, class B, class C>
class BaseProtocol<FOC<DriverBDCBase<A>, B, C>>
{
private:
    FOC<DriverBDCBase<A>, B, C>& instance;
public:
    BaseProtocol(FOC<DriverBDCBase<A>, B, C>& _ref, uint8_t _idx): instance(_ref), sub_dev_index(_idx) {};
    float GetEndpointValue(PROTOCOL_ENDPOINT endpoint);
    FOC_CMD_RET SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value);
    uint8_t node_id = 0x3F;
    uint8_t sub_dev_index = 0;
    uint64_t serial_number = 0;
};

template<class A, class B, class C>
float BaseProtocol<FOC<DriverBDCBase<A>, B, C>>::GetEndpointValue(PROTOCOL_ENDPOINT endpoint)
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
            return instance.bus_sense.Vbus * instance.bus_sense.Ibus;
        case DRIVE_ERROR_CODE:
            return (float)instance.error_code;
        case OUTPUT_STATE:
            return (float)instance.output_state;
        case DRIVE_MODE:
            return (float)instance.mode;
        case CURRENT_PHASE_A:
        case CURRENT_PHASE_B:
        case CURRENT_PHASE_C:
        case CURRENT_IALPHA:
        case CURRENT_IBETA:
        case CURRENT_IQ:
        case CURRENT_ID:
            return instance.est_input.Ialphabeta_fb.alpha;
        case VQ_OUT:
        case VD_OUT:
        case VALPHA_OUT:
        case VBETA_OUT:
            return instance.est_output->Uqd.q;
        case IQ_TARGET:
        case ID_TARGET:
            return instance.est_input.Iqd_target.q;
        case IQ_SET:
        case ID_SET:
            return instance.est_output->Iqd_set.q;
        case SPEED_TARGET:
            return origin_to_shaft(instance.est_input.target_speed, instance.config.motor.gear_ratio);
        case POS_ABS_SET_RAD:
            return origin_to_shaft(instance.est_input.target_pos, instance.config.motor.gear_ratio);
        case POS_ABS_SET_DEG:
            return RAD2DEG(GetEndpointValue(POS_ABS_SET_RAD));
        case CURRENT_KP:
            return instance.config.current_pid.Kp;
        case CURRENT_KI:
            return instance.config.current_pid.Ki;
        case VPHASE_LIMIT:
            return instance.config.current_pid.limit;
        case CURRENT_RAMP_LIMIT:
            return instance.config.current_pid.ramp_limit;
        case SPEED_KP:
            return instance.config.speed_pid.Kp;
        case SPEED_KI:
            return instance.config.speed_pid.Ki;
        case SPEED_KD:
            return instance.config.speed_pid.Kd;
        case SPEED_CURRENT_LIMIT:
            return instance.config.speed_pid.limit;
        case SPEED_RAMP_LIMIT:
            return instance.config.speed_pid.ramp_limit;
        case POS_KP:
            return instance.config.position_pid.Kp;
        case POS_KI:
            return instance.config.position_pid.Ki;
        case POS_KD:
            return instance.config.position_pid.Kd;
        case POS_SPEED_LIMIT:
            return instance.config.position_pid.limit;
        case POS_RAMP_LIMIT:
            return instance.config.position_pid.ramp_limit;
        case ELECTRIC_ANGLE_RAD:
        case ESTIMATED_ANGLE_RAD:
            return normalize_rad(GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD));
        case ELECTRIC_ANGLE_DEG:
        case ESTIMATED_ANGLE_DEG:
            return RAD2DEG(GetEndpointValue(ESTIMATED_ANGLE_RAD));
        case ESTIMATED_RAW_ANGLE_RAD:
            return origin_to_shaft(instance.est_output->estimated_raw_angle, instance.config.motor.gear_ratio);
        case ESTIMATED_RAW_ANGLE_DEG:
            return RAD2DEG(GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD));
        case ESTIMATED_SPEED:
            return origin_to_shaft(instance.est_output->estimated_speed, instance.config.motor.gear_ratio);
        // case ESTIMATED_ACCELERATION:
        //     return instance.est_output.estimated_acceleration;
        case TRAJ_TARGET_RAD:
            return instance.trajController.GetFinalPos();
        case TRAJ_TARGET_DEG:
            return RAD2DEG(instance.trajController.GetFinalPos());
        case TRAJ_CRUISE_SPEED:
            return instance.config.traj_cruise_speed;
        case TRAJ_MAX_ACCEL:
            return instance.config.traj_max_accel;
        case TRAJ_MAX_DECEL:
            return instance.config.traj_max_decel;
        case TRAJ_CURRENT_POS:
            return instance.trajController.GetPos();
        case TRAJ_CURRENT_SPEED:
            return rad_speed_to_RPM(instance.trajController.GetSpeed(), 1) * instance.config.motor.gear_ratio;
        case TRAJ_CURRENT_ACCEL:
            return rad_speed_to_RPM(instance.trajController.GetAccel(), 1) * instance.config.motor.gear_ratio;
        case MECHANIC_SPEED_LIMIT:
            return instance.config.motor.max_mechanic_speed;
        default: break;
    }
    return 0.0f;
}

template<class A, class B, class C>
FOC_CMD_RET BaseProtocol<FOC<DriverBDCBase<A>, B, C>>::SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value)
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
        case OUTPUT_STATE:
            instance.SetOutputState((bool)((uint8_t)set_value == 1));
            break;
        case DRIVE_MODE:
            if((FOC_MODE)set_value >= LAST_MODE_PLACEHOLDER) result = CMD_NOT_SUPPORTED;
            else instance.mode = (FOC_MODE)set_value;
            break;
        case IQ_TARGET:
        case ID_TARGET:
            instance.est_input.Iqd_target.q = set_value;
            break;
        case SPEED_TARGET:
            instance.est_input.target_speed = shaft_to_origin(set_value, instance.config.motor.gear_ratio);
            break;
        case POS_ABS_SET_RAD:
            instance.est_input.target_pos = shaft_to_origin(set_value, instance.config.motor.gear_ratio);
            break;
        case POS_INC_RAD:
            instance.est_input.target_pos += shaft_to_origin(set_value, instance.config.motor.gear_ratio);
            break;
        case POS_ABS_SET_DEG:
            return SetEndpointValue(POS_ABS_SET_RAD, DEG2RAD(set_value));
            break;
        case POS_INC_DEG:
            return SetEndpointValue(POS_INC_RAD, DEG2RAD(set_value));
            break;
        case CURRENT_KP:
            instance.config.current_pid.Kp = set_value;
            break;
        case CURRENT_KI:
            instance.config.current_pid.Ki = set_value;
            break;
        case VPHASE_LIMIT:
            instance.config.current_pid.limit = set_value;
            break;
        case CURRENT_RAMP_LIMIT:
            instance.config.current_pid.ramp_limit = set_value;
            break;
        case SPEED_KP:
            instance.config.speed_pid.Kp = set_value;
            break;
        case SPEED_KI:
            instance.config.speed_pid.Ki = set_value;
            break;
        case SPEED_KD:
            instance.config.speed_pid.Kd = set_value;
            break;
        case SPEED_CURRENT_LIMIT:
            instance.config.speed_pid.limit = set_value;
            break;
        case SPEED_RAMP_LIMIT:
            instance.config.speed_pid.ramp_limit = set_value;
            break;
        case POS_KP:
            instance.config.position_pid.Kp = set_value;
            break;
        case POS_KI:
            instance.config.position_pid.Ki = set_value;
            break;
        case POS_KD:
            instance.config.position_pid.Kd = set_value;
            break;
        case POS_SPEED_LIMIT:
            instance.config.position_pid.limit = set_value;
            break;
        case POS_RAMP_LIMIT:
            instance.config.position_pid.ramp_limit = set_value;
            break;
        case GO_HOME:
            // instance.estimator.SetSubMode(SUBMODE_HOME);
            break;
        case BREAK_MODE:
            instance.config.break_mode = (uint8_t)set_value;
            break;
        case TRAJ_TARGET_RAD:
            // if(!instance.estimator.trajController.GetState()) // we can override
            // {
            //     result = CMD_FORBIDDEN;
            //     break;
            // }
            instance.trajController.PlanTrajectory(shaft_to_origin(set_value, instance.config.motor.gear_ratio),    // rad(shaft)
                                                   instance.est_input.target_pos,                                   // rad(shaft)
                                                   RPM_speed_to_rad(instance.est_output->estimated_speed, 1),       // RPM(output) -> rad(shaft)
                                                   RPM_speed_to_rad(shaft_to_origin(instance.config.traj_cruise_speed, instance.config.motor.gear_ratio), 1),    // RPM(out) -> rad(shaft)
                                                   instance.config.traj_max_accel,                                  // RPM/s -> rad(shaft)
                                                   instance.config.traj_max_decel);                                 // RPM/s -> rad(shaft)
            break;
        case TRAJ_TARGET_DEG:
            return SetEndpointValue(TRAJ_TARGET_RAD, DEG2RAD(set_value));
        case TRAJ_TARGET_INC_RAD:
            return SetEndpointValue(TRAJ_TARGET_RAD, instance.trajController.GetFinalPos() + set_value);
        case TRAJ_TARGET_INC_DEG:
            return SetEndpointValue(TRAJ_TARGET_INC_RAD, DEG2RAD(set_value));
        case TRAJ_CRUISE_SPEED:
            instance.config.traj_cruise_speed = set_value;
            break;
        case TRAJ_MAX_ACCEL:
            instance.config.traj_max_accel = set_value;
            break;
        case TRAJ_MAX_DECEL:
            instance.config.traj_max_decel = set_value;
            break;
        case SET_HOME:
            
            break;
        case MECHANIC_SPEED_LIMIT:
            instance.config.motor.max_mechanic_speed = set_value;
            break;
#ifdef FOC_USING_INDICATOR
        case INDICATOR_STATE:
            // if(instance.error_code) break; // Modifying indicator state when ERROR_CODE present is not allowed
            if(instance.indicator != nullptr)
            {
                instance.indicator.get()->SetState(set_value != 0.0f);
            }
            break;
        case INDICATOR_TOGGLE:
            if(instance.indicator != nullptr)
            {
                instance.indicator.get()->Toggle();
            }
            break;
#endif
        default: 
            result = CMD_FORBIDDEN; 
            break;
    }
    return result;
}

#endif