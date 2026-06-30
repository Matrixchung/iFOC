#include "cpp_classes.hpp"

#include "w25q16.h"

#include "board_default_config.h"
#include "adc_handler.h"
#include "ascii_protocol.hpp"

#include "usbd_cdc_if.h"

iFOC::HAL::UART* uart3;
iFOC::FOCMotor* motor_1;

void HAL_Delay(uint32_t Delay)
{
	iFOC::HAL::DelayMs(Delay);
}

class TestTask : public RTOSTask
{
public:
    void loop() override
    {
        LL_GPIO_TogglePin(R_EN_GPIO_Port, R_EN_Pin);
        // LL_GPIO_TogglePin(LCD_DC_GPIO_Port, LCD_DC_Pin);
        sleep(500);
    }
};
TestTask testTask;

class LCDTask : public RTOSTask
{
public:
	void setup() override
	{
		LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
	}
	void loop() override
	{
		display_foc();
		sleep(30);
	}
};

LCDTask lcdTask;

void vofa_output()
{
	iFOC::Motion target{};
	iFOC::Motion current{};
	motor_1->GetTargetMotion(target,
				iFOC::Motion::Ref::BASE,
				iFOC::Motion::TorqueUnit::AMP,
				iFOC::Motion::SpeedUnit::RPM,
				iFOC::Motion::PosUnit::DEG);
	motor_1->GetCurrentMotion(current,
				iFOC::Motion::Ref::BASE,
				iFOC::Motion::TorqueUnit::AMP,
				iFOC::Motion::SpeedUnit::RPM,
				iFOC::Motion::PosUnit::DEG);
	// vofa.add(0, target.speed.value);
	// vofa.add(1, current.speed.value);
	// vofa.add(2, target.pos.value);
	// vofa.add(3, current.pos.value);
	// vofa.add(4, motor_1->Iqd_measured.q);
	// vofa.add(5, motor_1->Iqd_measured.d);
	vofa.add(0, motor_1->GetBusSense()->voltage);
	vofa.add(1, motor_1->GetBusSense()->current);
	vofa.add(2, motor_1->Iqd_measured.q);
	vofa.add(3, motor_1->Iqd_measured.d);
	// if(const auto task = motor_1->GetTaskByName("HFIMain"))
	// {
	// 	const auto hfi = reinterpret_cast<iFOC::FOC::ObserverHFI*>(task);
	// 	vofa.add(2, hfi->pll.angle_rad);
	// 	vofa.add(3, hfi->pll.omega_rad_s);
	// }
	vofa.add(4, target.speed.value);
	vofa.add(5, current.speed.value);
	vofa.add(6, target.pos.value);
	vofa.add(7, current.pos.value);
	// vofa.add(4, motor_1->elec_angle_rad);
	// vofa.add(5, motor_1->elec_omega_rad_s);
	// uart3->WriteBytes(vofa.buffer(), sizeof(vofa));
	// uart3->StartTransmit(false);
	CDC_Transmit_FS(vofa.buffer(), sizeof(vofa));
}

namespace iFOC::HAL::Bootloader
{
    void JumpToBL(const BootloaderMsg& msg)
    {

    }


    bool HasBL()
    {
        return false;
    }

    void GetBLVersion(uint8_t& major, uint8_t& minor, uint32_t& vcs)
    {
        major = 0;
        minor = 0;
        vcs = 0;
    }

    void SetAppInitSuccessFlag()
    {

    }

    void OnHardFault()
    {

    }
}

void app_main(void)
{
	uart3 = new iFOC::HAL::UART(&huart3);
	motor_1 = new iFOC::FOCMotor();

    __HAL_DBGMCU_FREEZE_TIM1();

	LL_GPIO_SetOutputPin(LCD_CS_GPIO_Port, LCD_CS_Pin);
	LL_GPIO_SetOutputPin(SPI3_CS_GPIO_Port, SPI3_CS_Pin);
	iFOC::HAL::DelayInit();

	RTC_TimeTypeDef sTime{};
	RTC_DateTypeDef sDate{};
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	BSP_W25Qx_Init();
	LCD_Init();

	if(iFOC::BoardConfig().ReadNVMConfig() != iFOC::FuncRetCode::OK ||
		iFOC::BoardConfig().GetConfig().pwm_wave_freq() == 0 ||
		iFOC::BoardConfig().GetConfig().speed_loop_freq() == 0)
	{
		motor_1->ThrowError(iFOC::MotorError::SYSTEM_BOARD_CONFIG_READ_ERROR);
		iFOC::BoardConfig().GetConfig().clear();
		APPLY_DEFAULT_BOARD_CONFIG();
		if(iFOC::BoardConfig().SaveNVMConfig() != iFOC::FuncRetCode::OK)
			motor_1->ThrowError(iFOC::MotorError::SYSTEM_BOARD_CONFIG_WRITE_ERROR);
	}

	iFOC::RT_LOOP_TS = 1.0f / (float)iFOC::BoardConfig().GetConfig().pwm_wave_freq();
	iFOC::MID_LOOP_TS = 1.0f / (float)iFOC::BoardConfig().GetConfig().speed_loop_freq();

	uart3->Init(iFOC::BoardConfig().GetConfig().uart_1_baudrate());

	ADC_Start_DMA();

	lcdTask.start("LCDTask", 1024, tskIDLE_PRIORITY + 1);

	auto* adc = new iFOC::Sense::BusSenseADC(new iFOC::HAL::ADCPort((uint16_t*)&V_BUS_RAW, (uint16_t*)&VREFINT_RAW), 20.965f,
													   new iFOC::HAL::ADCPort((uint16_t*)&I_BUS_RAW, (uint16_t*)&VREFINT_RAW), 20.0f);

	adc->SampleDCOffset(50);

	motor_1->LinkDriver(new iFOC::Driver::FOCDriver6PWM(TIM1));

	motor_1->LinkCurrSense(new iFOC::Sense::CurrSenseThreeShunts(new iFOC::HAL::ADCPort((uint16_t*)&ADC1->JDR3, (uint16_t*)&VREFINT_RAW),
																 new iFOC::HAL::ADCPort((uint16_t*)&ADC1->JDR2, (uint16_t*)&VREFINT_RAW),
																 new iFOC::HAL::ADCPort((uint16_t*)&ADC1->JDR1, (uint16_t*)&VREFINT_RAW),
																 true, true, true));

	motor_1->LinkBusSense(adc);

	motor_1->LinkCoreTempSense(new iFOC::Sense::TempSenseCore(new iFOC::HAL::ADCPort((uint16_t*)&MCU_TEMP_RAW, (uint16_t*)&VREFINT_RAW)));

	motor_1->LinkIndicator(new iFOC::HAL::WS2812(TIM2, LL_TIM_CHANNEL_CH1, DMA2, LL_DMA_CHANNEL_8));

	motor_1->RegisterProtocol(new iFOC::Protocol::ASCIIProtocol<iFOC::FOCMotor>(uart3));

	motor_1->Init(true);

	ADC_Start_Injected();
	LL_TIM_EnableIT_UPDATE(TIM16);
	LL_TIM_EnableCounter(TIM16);

	testTask.start("TestTask", 128, tskIDLE_PRIORITY + 1);
	// uartTask.start("UARTTask", 512, tskIDLE_PRIORITY + 1);

    osKernelStart();
    while(1);
}