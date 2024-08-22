// https://docs.odriverobotics.com/v/latest/guides/can-guide.html#can-endpoint-access
#ifndef _FOC_BASE_PROTOCOL_H
#define _FOC_BASE_PROTOCOL_H

#include "foc.hpp"
#include "foc_header.h"
#include "endpoints_enum.h"
#include "eeprom_interface.hpp"

// All protocols share same UID(aka node_id) externed from BaseProtocol, which fits the CANSimple Protocol ID
// ID range: 0x00 - 0x3E (0x3F is the unaddressed ID, also global broadcast ID)
// BaseProtocol only process "Endpoints"

// For EEPROM:
// At init, we will read all packet based on given sud_dev_index, and check crc.
// If CRC correct, then we will apply those settings to FOC config struct.
// If CRC WRONG, we will writeback all default settings to EEPROM here.

typedef struct config_blob_t
{
    uint8_t crc_8;           // CRC8 checksum of all variables below
    uint8_t node_id;
    uint32_t serial_number_lsb;
}config_blob_t;

PROTOCOL_ENDPOINT GetEndpointFromIndex(int i) { return static_cast<PROTOCOL_ENDPOINT>(i); }

template<class T_FOC>
class BaseProtocol
{
private:
    T_FOC& instance;
public:
    BaseProtocol(T_FOC& _ref, uint8_t _idx): instance(_ref), sub_dev_index(_idx) {};
    EEPROM<I2CBase> *pROM = nullptr;
    template<class T> void AttachEEPROM(EEPROM<T> *ptr);
    float GetEndpointValue(PROTOCOL_ENDPOINT endpoint);
    FOC_CMD_RET SetEndpointValue(PROTOCOL_ENDPOINT endpoint, float set_value);
    uint8_t node_id = 0x3F;
    uint8_t sub_dev_index = 0; // used to mark the index among same physical device, for example motor1, motor2, ...
    uint64_t serial_number = 0;
private:
    config_blob_t config_blob;
    bool ReadConfig();
    bool SaveConfig();
    bool EraseConfig();
};

template<class U>
template<class T>
void BaseProtocol<U>::AttachEEPROM(EEPROM<T> *ptr)
{
    pROM = reinterpret_cast<EEPROM<I2CBase>*>(ptr); // physical address: sub_dev_index * sizeof(config_blob_t)
    if(pROM != nullptr) ReadConfig();
}

template<class U>
bool BaseProtocol<U>::ReadConfig()
{
    if(pROM == nullptr) return false;
    config_blob = pROM->ReadVar<config_blob_t>(sub_dev_index * sizeof(config_blob_t));
    uint8_t buffer[sizeof(config_blob_t) - sizeof(uint8_t)];
    uint8_t *ptr = (uint8_t*)&config_blob;
    memcpy(buffer, ptr + 1, sizeof(buffer));
    if(config_blob.crc_8 == getCRC8(buffer, sizeof(buffer)) &&   // checksum valid
       config_blob.serial_number_lsb == (uint32_t)serial_number  // serial number match
    ) 
    {
        node_id = config_blob.node_id;
        return true;
    }
    return false;
}

template<class U>
bool BaseProtocol<U>::SaveConfig()
{
    if(pROM == nullptr) return false;
    config_blob.node_id = node_id;
    config_blob.serial_number_lsb = (uint32_t)serial_number;
    uint8_t buffer[sizeof(config_blob_t) - sizeof(uint8_t)];
    uint8_t *ptr = (uint8_t*)&config_blob;
    memcpy(buffer, ptr + 1, sizeof(buffer));
    config_blob.crc_8 = getCRC8(buffer, sizeof(buffer));
    pROM->WriteVar<config_blob_t>(sub_dev_index * sizeof(config_blob_t), config_blob);
    return true;
}

template<class U>
bool BaseProtocol<U>::EraseConfig()
{
    if(pROM == nullptr) return false;
    pROM->FlushPage(0x00);
    node_id = 0x3F;
    return true;
}

template<class U>
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
            return instance.bus_sense.Vbus * instance.bus_sense.Ibus;
            break;
        // case SERIAL_NUMBER:
        //     return (float)serial_number;
        case DRIVE_ERROR_CODE:
            return (float)instance.error_code;
        case OUTPUT_STATE:
            return (float)instance.output_state;
        case DRIVE_MODE:
            return (float)instance.mode;
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
            return instance.est_output->Iqd_fb.q;
        case CURRENT_ID:
            return instance.est_output->Iqd_fb.d;
        case VQ_OUT:
            return instance.est_output->Uqd.q;
        case VD_OUT:
            return instance.est_output->Uqd.d;
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
            return instance.est_output->Iqd_set.q;
        case ID_SET:
            return instance.est_output->Iqd_set.d;
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
            return instance.est_output->electric_angle;
        case ELECTRIC_ANGLE_DEG:
            return RAD2DEG(instance.est_output->electric_angle);
        case ESTIMATED_ANGLE_RAD:
            return normalize_rad(GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD));
        case ESTIMATED_ANGLE_DEG:
            return RAD2DEG(GetEndpointValue(ESTIMATED_ANGLE_RAD));
        case ESTIMATED_RAW_ANGLE_RAD:
            return origin_to_shaft(instance.est_output->estimated_raw_angle, instance.config.motor.gear_ratio);
        case ESTIMATED_RAW_ANGLE_DEG:
            return RAD2DEG(GetEndpointValue(ESTIMATED_RAW_ANGLE_RAD));
        case ESTIMATED_SPEED:
            return origin_to_shaft(instance.est_output->estimated_speed, instance.config.motor.gear_ratio);
        case BREAK_MODE:
            return instance.config.break_mode;
        case TRAJ_POS_STATE:
            return instance.GetTrajPosState();
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
            return instance.trajController.GetSpeed();
        case TRAJ_CURRENT_ACCEL:
            return instance.trajController.GetAccel();
        case MECHANIC_SPEED_LIMIT:
            return instance.config.motor.max_mechanic_speed;
        default: break;
    }
    return 0.0f;
}
template<class U>
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
        case CONFIGURATION:
            if(set_value == 0.0f) result = ReadConfig() ? CMD_SUCCESS : CMD_UNKNOWN_FAILURE;
            else if(set_value == 1.0f) result = SaveConfig() ? CMD_SUCCESS : CMD_UNKNOWN_FAILURE;
            else if(set_value == 2.0f) result = EraseConfig() ? CMD_SUCCESS : CMD_UNKNOWN_FAILURE;
            else result = CMD_NOT_SUPPORTED;
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
            if((uint8_t)set_value >= LAST_MODE_PLACEHOLDER) result = CMD_NOT_SUPPORTED;
            else instance.mode = (FOC_MODE)set_value;
            break;
        case IQ_TARGET:
            instance.est_input.Iqd_target.q = set_value;
            break;
        case ID_TARGET:
            instance.est_input.Iqd_target.d = set_value;
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
        case POS_INC_DEG:
            return SetEndpointValue(POS_INC_RAD, DEG2RAD(set_value));
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
            // instance.estimator->SetSubMode(SUBMODE_HOME);
            break;
        case BREAK_MODE:
            instance.config.break_mode = (uint8_t)set_value;
            break;
        case TRAJ_TARGET_RAD:
            // if(!instance.trajController.GetState())
            // {
            //     result = CMD_FORBIDDEN;
            //     break;
            // }
            instance.trajController.PlanTrajectory(shaft_to_origin(set_value, instance.config.motor.gear_ratio), // rad 
                                                    instance.est_input.target_pos,                            // rad, use target
                                                    RPM_speed_to_rad(instance.est_output->estimated_speed, 1),   // rpm -> rad/s
                                                    RPM_speed_to_rad(shaft_to_origin(instance.config.traj_cruise_speed, instance.config.motor.gear_ratio), 1), // rpm -> rad/s
                                                    instance.config.traj_max_accel,
                                                    instance.config.traj_max_decel);
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

template<typename T, size_t N>
class ProtocolSerializer
{
public:
    template<class...U>
    ProtocolSerializer(BaseProtocol<U>&... inst) : instances{ {(BaseProtocol<FOC<DriverDefault, CurrentSenseDefault, BusSenseDefault>>*)(&inst)...} } 
    {
        static_assert(sizeof...(inst) == N, "Number of BaseProtocol instances must match size N.");
    };
    BaseProtocol<T>* get(size_t index)
    {
        auto ptr = instances.at(index);
        return reinterpret_cast<BaseProtocol<T>*>(ptr);
    }
    size_t size()
    {
        return instances.size();
    }
private:
    std::array<BaseProtocol<FOC<DriverDefault, CurrentSenseDefault, BusSenseDefault>>*, N> instances;
};

#endif