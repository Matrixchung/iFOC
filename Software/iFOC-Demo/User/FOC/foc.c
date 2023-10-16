#include "foc.h"
float zero_electric_angle = 0;
static float voltage_power = 12.0f;
static float shaft_angle = 0;
static float U_alpha = 0, U_beta = 0, U_a = 0, U_b = 0, U_c = 0;
static float dc_a = 0, dc_b = 0, dc_c = 0;

void FOC_SetPWM(FOC_TypeDef *foc)
{
    foc->dc_a = _constrain(foc->U_a / foc->voltage_limit, 0.0f, 1.0f);
    foc->dc_b = _constrain(foc->U_b / foc->voltage_limit, 0.0f, 1.0f);
    foc->dc_c = _constrain(foc->U_c / foc->voltage_limit, 0.0f, 1.0f);
    DRV8313_SetPWMPercent(foc->dc_a, foc->dc_b, foc->dc_c);
}

void FOC_SetPhaseVoltage(FOC_TypeDef *foc, float U_q, float target_angle)
{
    if (U_q == 0)
    {
        foc->U_alpha = 0;
        foc->U_beta = 0;
        foc->U_a = foc->half_voltage_limit;
        foc->U_b = foc->half_voltage_limit;
        foc->U_c = foc->half_voltage_limit;
        FOC_SetPWM(foc);
        return;
    }
    target_angle = _normalizeRad(target_angle);
    // inverted Park transform
    foc->U_alpha = -U_q * sinf(target_angle);
    foc->U_beta = U_q * cosf(target_angle);

    // inverse Clarke transform
    foc->U_a = foc->U_alpha;
    foc->U_b = (_sqrt3 * foc->U_beta - foc->U_alpha) * 0.5f;
    foc->U_c = (_sqrt3neg * foc->U_beta - foc->U_alpha) * 0.5f;

    // Generate SVPWM
    // float temp1 = foc->U_beta;
    // float temp2 = 0.5f * foc->U_beta + _sqrt3_2 * foc->U_alpha;
    // float temp3 = temp2 - temp1;
    // uint8_t sector = 3;
    // sector = temp2 > 0 ? (sector - 1) : sector;
    // sector = temp3 > 0 ? (sector - 1) : sector;
    // sector = temp1 < 0 ? (7 - sector) : sector;
    // if(sector == 1 || sector == 4)
    // {
    //     foc->U_a = temp2;
    //     foc->U_b = temp1 - temp3;
    //     foc->U_c = -temp2;
    // }
    // else if(sector == 2 || sector == 5)
    // {
    //     foc->U_a = temp3 + temp2;
    //     foc->U_b = temp1;
    //     foc->U_c = -temp1;
    // }
    // else
    // {
    //     foc->U_a = temp3;
    //     foc->U_b = -temp3;
    //     foc->U_c = - temp1 - temp2;
    // }
    // prevent negative voltage
    foc->U_a += foc->half_voltage_limit;
    foc->U_b += foc->half_voltage_limit;
    foc->U_c += foc->half_voltage_limit;
    // Set PWM Output
    FOC_SetPWM(foc);
}

FOC_TypeDef FOC_Driver_Init(float voltage_limit)
{
    FOC_TypeDef foc;
    foc.zero_electric_angle = 0;
    foc.voltage_limit = voltage_limit;
    foc.half_voltage_limit = voltage_limit * 0.5f;
    foc.U_alpha = 0;
    foc.U_beta = 0;
    foc.U_a = 0;
    foc.U_b = 0;
    foc.U_c = 0;
    foc.dc_a = 0;
    foc.dc_b = 0;
    foc.dc_c = 0;
    LPFilter_TypeDef vel_filter = LPFilter_Init(0.01f); // velocity filter Tf = 10ms
    foc.velocity_filter = vel_filter;
    DRV8313_PWM_Init();
    DRV8313_SetState(1);
    // PWM_Encoder_SoftwareInit(); // PWM Encoder

    // zero_elec_angle calibration start
    FOC_SetPhaseVoltage(&foc, 3, _3PI_2);
    delay_ms(800);
    foc.zero_electric_angle = FOC_GetElecRad();
    FOC_SetPhaseVoltage(&foc, 0, _3PI_2);

    // foc interrupt start (5 KHz)
    LL_TIM_ClearFlag_UPDATE(TIM2);
    LL_TIM_EnableIT_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    return foc;
}

void setPwmVoltage(void)
{
    dc_a = _constrain(U_a / voltage_power, 0.0f, 1.0f);
    dc_b = _constrain(U_b / voltage_power, 0.0f, 1.0f);
    dc_c = _constrain(U_c / voltage_power, 0.0f, 1.0f);
    DRV8313_SetPWMPercent(dc_a, dc_b, dc_c);
}

void setPhaseVoltage(float U_q, float target_angle)
{
    U_q = _constrain(U_q, voltage_power * -0.5f, voltage_power * 0.5f);
    target_angle = _normalizeRad(target_angle);

    // inverted Park transform
    // U_alpha = -U_q * sinf(target_angle) + U_d * cosf(target_angle);
    // U_beta = U_q * cosf(target_angle) + U_d * sinf(target_angle);
    U_alpha = -U_q * sinf(target_angle);
    U_beta = U_q * cosf(target_angle);

    // inverse Clarke transform
    U_a = U_alpha + voltage_power * 0.5f; // prevent negative voltage
    U_b = (_sqrt3 * U_beta - U_alpha) * 0.5f + voltage_power * 0.5f;
    U_c = (_sqrt3neg * U_beta - U_alpha) * 0.5f + voltage_power * 0.5f;

    // setPwmVoltage();
}
float FOC_GetElecRad(void)
{
    return _normalizeRad((float)(MOTOR_ENC_DIR * MOTOR_POLE_PAIRS * (float)(SPI_Encoder_GetRawRad()) - zero_electric_angle));
}
void FOC_CalcVelocity(FOC_TypeDef *foc)
{
    foc->velocity = LPFilter_GetOutput(&(foc->velocity_filter), SPI_Encoder_GetVelocity());
}
void FOC_Position_Closeloop(FOC_TypeDef *foc, float motor_target)
{
    FOC_SetPhaseVoltage(foc, 0.099f*(motor_target+(AS5048A_DIR*(SPI_Encoder_GetAngle()))), FOC_GetElecRad());
}
void FOC_Velocity_Closeloop(FOC_TypeDef *foc, float target_rad)
{
    FOC_SetPhaseVoltage(foc, 0.01f*(radToDeg(target_rad - foc->velocity)), FOC_GetElecRad());
}
float FOC_Velocity_Openloop(FOC_TypeDef *foc, float target_rad)
{
    static uint32_t timestamp = 0;
    uint32_t nowMicros = get_micros();
    float multiplier = (nowMicros - timestamp) * 1e-6f; // convert to seconds
    if(multiplier <= 0 || multiplier > 0.05f) multiplier = 0.001f;
    shaft_angle = _normalizeRad(shaft_angle + target_rad * multiplier);
    float U_q = voltage_power * 0.3f;
    FOC_SetPhaseVoltage(foc, U_q, shaft_angle * MOTOR_POLE_PAIRS * MOTOR_ENC_DIR);
    timestamp = nowMicros;
    return U_q;
}

// void FOC_Driver_Init(void)
// {
//     // PWM_Encoder_Init(&htim1);
//     PWM_Encoder_Init_LL(TIM1);
//     DRV8313_PWM_Init();
//     DRV8313_SetState(1);
//     delay_ms(500);
//     // Calibrate zero electric angle
//     setPhaseVoltage(3, 0, _3PI_2);
//     delay_ms(1000);
//     zero_electric_angle = FOC_GetElecRad();
//     FOC_Driver_ResetOutput();
// }