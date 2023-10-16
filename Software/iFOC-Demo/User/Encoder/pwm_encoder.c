#include "pwm_encoder.h"
// TIM_HandleTypeDef *encoder_htim;
TIM_TypeDef *encoder_tim;
uint32_t pwm_rising_count = 0;
uint32_t pwm_falling_count = 0;
float encoder_pwm_duty = 0.0f;
float encoder_pwm_freq = 0.0f;
float encoder_rad = 0.0f;
int encoder_full_rotations = 0;

static float _rad_prev = -1.0f; // used in rotation detection, init -1.0f
static uint32_t _vel_rad_ts = 0;
static float _vel_rad_prev = -1.0f;
static uint32_t _vel_rad_prev_ts = 0;
static int _vel_full_rotations = 0;
static float _rad_delta = 0.0f;
// void PWM_Encoder_Init(TIM_HandleTypeDef *htim)
// {
//     // By default its input at CHANNEL_1
//     encoder_htim = htim;
//     HAL_TIM_IC_Start_IT(encoder_htim, TIM_CHANNEL_1);
//     HAL_TIM_IC_Start_IT(encoder_htim, TIM_CHANNEL_2);
// }
void PWM_Encoder_HardwareInit_LL(TIM_TypeDef *tim)
{
    encoder_tim = tim;
    LL_TIM_ClearFlag_CC1(encoder_tim);
    LL_TIM_EnableIT_CC1(encoder_tim);
    LL_TIM_ClearFlag_CC2(encoder_tim);
    LL_TIM_EnableIT_CC2(encoder_tim);
    LL_TIM_CC_EnableChannel(encoder_tim, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(encoder_tim, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(encoder_tim);
}
void PWM_Encoder_SoftwareInit(void)
{
    while(encoder_pwm_freq <= AS5048A_PWM_FREQUENCY * 0.9f); // wait till encoder pwm capture gets still
    _vel_rad_prev = encoder_rad;
    _vel_rad_prev_ts = get_micros();
    delay_ms(1);
    _rad_prev = encoder_rad;
    _vel_rad_ts = get_micros();
    _rad_delta = 0.0f;
}
float PWM_Encoder_GetVelocity(void)
{
    static float _last_Ts = 0.0f;
    if(_rad_prev < 0.0f) return 0.0f;
    _vel_rad_ts = get_micros();
    float Ts = (float) (_vel_rad_ts - _vel_rad_prev_ts) * 1e-6f;
    if(Ts <= 0) Ts = _last_Ts;
    float vel = ((float)(encoder_full_rotations - _vel_full_rotations) * _2PI + (_rad_prev - _vel_rad_prev)) / Ts;
    _vel_rad_prev = _rad_prev;
    _vel_rad_prev_ts = _vel_rad_ts;
    _vel_full_rotations = encoder_full_rotations;
    _last_Ts = Ts;
    return vel;
}
void PWM_Encoder_IRQHandler_LL(void)
{
    if(LL_TIM_IsActiveFlag_CC1(encoder_tim) != RESET)
    {
        LL_TIM_ClearFlag_CC1(encoder_tim);
        if(LL_TIM_IC_GetCaptureCH1(encoder_tim) != 0)
        {
            encoder_pwm_freq = 1000000.0f/(float)LL_TIM_IC_GetCaptureCH1(encoder_tim); // 1 MHz
            if(encoder_pwm_freq >= AS5048A_PWM_FREQUENCY_MAX || encoder_pwm_freq <= AS5048A_PWM_FREQUENCY_MIN)
            {
                encoder_pwm_duty = 0;
                encoder_rad = _rad_prev;
            }
            else
            {
                encoder_pwm_duty = (float)pwm_falling_count/(float)LL_TIM_IC_GetCaptureCH1(encoder_tim);
                encoder_rad = AS5048A_DIR == 1 ? (float)(encoder_pwm_duty * _2PI) : (float)((1.0f-encoder_pwm_duty) * _2PI);
            } 
            if(_rad_prev >= 0.0f) // stable sampling
            {
                _rad_delta = encoder_rad - _rad_prev;
                if(_rad_delta >= ROTATION_CHECK_THRESHOLD) encoder_full_rotations--;
                else if(_rad_delta <= -ROTATION_CHECK_THRESHOLD) encoder_full_rotations++;
            }
            _rad_prev = encoder_rad;
        }
    }
    if(LL_TIM_IsActiveFlag_CC2(encoder_tim) != RESET)
    {
        LL_TIM_ClearFlag_CC2(encoder_tim);
        pwm_falling_count = LL_TIM_IC_GetCaptureCH2(encoder_tim);
    }
}
// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
// {
//     if(htim == encoder_htim)
//     {
//         if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//         {
//             pwm_rising_count = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//             if(pwm_rising_count != 0 && pwm_falling_count != 0)
//             {
//                 encoder_pwm_duty = (float)(pwm_falling_count)/(float)(pwm_rising_count);
//                 encoder_pwm_freq = 1000000.0f/(float)(pwm_rising_count); // 1000000 Hz sampling rate
//             }
//         }
//         else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//         {
//             pwm_falling_count = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//         }
//     }
// }
