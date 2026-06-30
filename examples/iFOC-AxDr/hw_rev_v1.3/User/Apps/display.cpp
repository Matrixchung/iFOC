#include "cpp_classes.hpp"

void display_foc(void)
{
	// State display
	if(motor_1->GetError() > 0) LCD_ShowString(10, 5,(uint8_t *)"[ERROR]", RED, BLACK, 32, 0);
	else
	{
		switch(motor_1->GetCurrentState())
		{
			case iFOC::MotorState::BASIC_PARAM_CALIBRATION:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[PARAM]", BLUE, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::ENCODER_CALIBRATION:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[ENC_C]", YELLOW, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::ENCODER_INDEX_SEARCH:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[ENC_S]", LGRAY, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::EXTEND_PARAM_CALIBRATION:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[EXT_P]", BRED, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::OPEN_LOOP_VELOCITY_CONTROL:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[OPENL]", LIGHTGREEN, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::SENSORED_CLOSED_LOOP_CONTROL:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[SNSOR]", LIGHTGREEN, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::SENSORLESS_CLOSED_LOOP_CONTROL:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[SLESS]", LIGHTGREEN, BLACK, 32, 0);
				break;
			}
			case iFOC::MotorState::IDLE:
			default:
			{
				LCD_ShowString(10, 5,(uint8_t *)"[IDLE] ", GREEN, BLACK, 32, 0);
				break;
			}
		}
	}

	LCD_ShowString(130, 5,(uint8_t *)"Vbus:", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(170, 5, motor_1->GetBusSense()->voltage, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(220, 5, (uint8_t *)"V", WHITE, BLACK, 16, 0);

	LCD_ShowString(130, 21,(uint8_t *)"Ibus:", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum(170, 21, motor_1->GetBusSense()->current, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(220, 21, (uint8_t *)"A", WHITE, BLACK, 16, 0);

	LCD_ShowString(130, 37,(uint8_t *)"Tmos:", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(170, 37, 31.27, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(220, 37, (uint8_t *)"C", WHITE, BLACK, 16, 0);

	LCD_DrawLine(10, 60, 10, 135, WHITE);
	LCD_DrawLine(230, 60, 230, 135, WHITE);

	LCD_ShowString(20, 40, (uint8_t *)"Mode :", WHITE, BLACK, 16, 0);
	LCD_ShowString(20, 60, (uint8_t *)"Iq.  :", WHITE, BLACK, 16, 0);
	LCD_ShowString(20, 80, (uint8_t *)"Vel. :", WHITE, BLACK, 16, 0);
	LCD_ShowString(20, 100,(uint8_t *)"Pos. :", WHITE, BLACK, 16, 0);
	LCD_ShowString(20, 120,(uint8_t *)"Tor. :", WHITE, BLACK, 16, 0);

	LCD_ShowString(76, 40, (uint8_t *)"[Pos.]", WHITE, BLACK, 16, 0);

	LCD_ShowFloatNum1(68, 60, 00.00, 3, 2, BRRED, BLACK, 16);
	LCD_ShowString(124, 60, (uint8_t *)"->", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(140, 60, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(196, 60, (uint8_t *)"A", WHITE, BLACK, 16, 0);

	LCD_ShowFloatNum1(68, 80, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(124, 80, (uint8_t *)"->", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(140, 80, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(196, 80, (uint8_t *)"r/s", WHITE, BLACK, 16, 0);

	LCD_ShowFloatNum1(68, 100, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(124, 100, (uint8_t *)"->", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(140, 100, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(196, 100, (uint8_t *)"rad", WHITE, BLACK, 16, 0);

	LCD_ShowFloatNum1(68, 120, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(124, 120, (uint8_t *)"->", WHITE, BLACK, 16, 0);
	LCD_ShowFloatNum1(140, 120, 00.00, 3, 2, GREEN, BLACK, 16);
	LCD_ShowString(196, 120, (uint8_t *)"N/m", WHITE, BLACK, 16, 0);

}