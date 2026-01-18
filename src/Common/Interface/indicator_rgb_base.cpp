#include "indicator_rgb_base.hpp"

#include "../foc_math.hpp"

namespace iFOC::HAL
{
void IndicatorRGB::Update(uint8_t motor_id, Error error, MotorState state, MotorControlMode control_mode)
{
    if(timer_tick++ >= 9)
    {
        timer_blink_tick++;
        if(timer_blink_tick > 20) timer_blink_tick = 0;
        if(error == 0)
        {
            SetBrightness(0.2f);
            // green gradient range: 170 - 255
            if(!gradient_flag)
            {
                color_wheel_i -= 1;
                if(color_wheel_i <= 170)
                {
                    color_wheel_i = 170;
                    gradient_flag = true;
                }
                else if(color_wheel_i >= 252) color_wheel_i = 252;
            }
            else
            {
                color_wheel_i += 1;
                if(color_wheel_i >= 252)
                {
                    color_wheel_i = 252;
                    gradient_flag = false;
                }
                else if(color_wheel_i <= 170) color_wheel_i = 170;
            }
            uint8_t r = 0, g = 0, b = 0;
            get_rgb_color_wheel(color_wheel_i, &r, &g, &b);
            SetColor(0, r, g, b);
        }
        else // error
        {
            SetBrightness(0.8f);
            // red gradient: 60 - 83
            if(!gradient_flag)
            {
                color_wheel_i -= 1;
                if(color_wheel_i <= 60)
                {
                    color_wheel_i = 60;
                    gradient_flag = true;
                }
                else if(color_wheel_i >= 83) color_wheel_i = 83;
            }
            else
            {
                color_wheel_i += 1;
                if(color_wheel_i >= 83)
                {
                    color_wheel_i = 83;
                    gradient_flag = false;
                }
                else if(color_wheel_i <= 60) color_wheel_i = 60;
            }
            auto error_count = count_bits(error);
            uint8_t r = 0, g = 0, b = 0;
            if(timer_blink_tick < error_count * 2)
            {
                if(timer_blink_tick % 2 == 0) // bright
                {
                    get_rgb_color_wheel(color_wheel_i, &r, &g, &b);
                }
                else // dim
                {

                }
            }
            else get_rgb_color_wheel(color_wheel_i, &r, &g, &b);
            SetColor(0, r, g, b);
        }
        Update();
        timer_tick = 0;
    }
}

}
