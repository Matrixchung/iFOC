#include "ascii_protocol.hpp"
#include "at32f403a_407_int.h"
#include "cpp_classes.hpp"
#include "board_default_config.h"
#include "foc_task_update_sense.hpp"
#include "i2c_sw.hpp"
#include "vofa.hpp"
#include "rtos_task.hpp"
#include "cyphal_protocol.hpp"
#include "dronecan_protocol.hpp"
#include "encoder_off_axis_uart.hpp"

iFOC::FOCMotor* motor_1 = nullptr;

iFOC::HAL::UART* uart1 = nullptr;

iFOC::HAL::UART* uart3 = nullptr;

iFOC::HAL::CAN* can1 = nullptr;

// iFOC::Protocol::USBProtocolFOC* usb_protocol = nullptr;

iFOC::Driver::FOCDriverDRV830x* drv830x = nullptr;

class UARTTask : public RTOSTask
{
public:
    void loop() final
    {
        iFOC::Motion target;
        iFOC::Motion current;
        motor_1->GetTargetMotion(target,
                    iFOC::Motion::Ref::OUTPUT,
                    iFOC::Motion::TorqueUnit::AMP,
                    iFOC::Motion::SpeedUnit::RADS,
                    iFOC::Motion::PosUnit::DEG);
        motor_1->GetCurrentMotion(current,
                    iFOC::Motion::Ref::OUTPUT,
                    iFOC::Motion::TorqueUnit::AMP,
                    iFOC::Motion::SpeedUnit::RADS,
                    iFOC::Motion::PosUnit::DEG);
        vofa.add(0, target.speed.value);
        vofa.add(1, current.speed.value);
        vofa.add(2, target.pos.value);
        vofa.add(3, current.pos.value);
        // vofa.add(4, motor_1->Iqd_measured.q);
        // vofa.add(5, motor_1->Iqd_measured.d);

        auto* encoder = motor_1->GetEncoderByName("EncOffAxis");
        if(encoder)
        {
            auto* ptr = (iFOC::Encoder::EncoderOffAxisBase*)encoder;
            vofa.add(4, iFOC::RAD2DEG(ptr->multi_round_angle_rad));
            vofa.add(5, ptr->angular_speed_rad_s);
        }

        // vofa.add(6, gpio_input_data_bit_read(DRV_FAULT_GPIO_PORT, DRV_FAULT_PIN));
        // vofa.add(6, motor_1->GetCurrSense()->shunt_values[0]);
        // vofa.add(7, motor_1->GetCurrSense()->shunt_values[1]);
        uart1->WriteBytes(vofa.buffer(), sizeof(vofa));
        uart1->StartTransmit(false);
        // uart1.Print(1, "Vin/Iin:%.2f,%.2f, Vout:%.2f,%.2f\n", ina237_in.voltage, ina237_in.current, ina237_out.voltage, ina237_out.current);
        sleep(10);
    }
};
UARTTask uartTask;

void usb_init()
{
    crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE); // Enable USB clock
    /*
     * Note: from AT32F403Ax Reference Manual, when both USB & CAN peripheral activated,
     *       USB interrupts will be remapped to IRQ Line 73 & 74 (USBFS_MAPH/L)
     *       see: crm_usb_interrupt_remapping_set(CRM_USB_INT73_INT74);
     */
    usbd_core_init(&usb_core_dev, USB, &custom_hid_class_handler, &custom_hid_desc_handler, 0);
    usbd_connect(&usb_core_dev);
}

extern "C"
{
    extern const volatile uint32_t __firmware_start;
}

void app_main(void)
{
    // SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk; // Enable BusFault interrupt
    SCB->CCR |= SCB_CCR_STKALIGN_Msk; // Enable stack align
    SCB->CFSR = 0x00000000; // Clear previous CFSR flag

    motor_1 = new iFOC::FOCMotor();
    uart1 = new iFOC::HAL::UART(USART1, DMA2_CHANNEL1, DMA2_CHANNEL2);
    uart3 = new iFOC::HAL::UART(USART3, DMA2_CHANNEL3, DMA2_CHANNEL4);
    can1 = new iFOC::HAL::CAN(CAN1);
    // usb_protocol = new iFOC::Protocol::USBProtocolFOC(64);

    auto* swi2c = new iFOC::HAL::I2CSW(new iFOC::HAL::GPIO(SWI2C_SCL_GPIO_PORT, SWI2C_SCL_PIN),
                                 new iFOC::HAL::GPIO(SWI2C_SDA_GPIO_PORT, SWI2C_SDA_PIN),
                                 1000000);

    debug_periph_mode_set(DEBUG_TMR1_PAUSE, TRUE); // __HAL_DBGMCU_FREEZE_TIM1()
    debug_periph_mode_set(DEBUG_WDT_PAUSE, TRUE);

    // IAP Part
    SCB->VTOR = (uint32_t)&__firmware_start;
    __enable_irq();
    wdt_register_write_enable(TRUE);
    wdt_enable();
    wdt_register_write_enable(FALSE);
    wdt_counter_reload(); // feed the dog

    iFOC::HAL::DelayInit();
    iFOC::HAL::GetFirmwareCRC64(); // calculate CRC during startup
    rtc_counter_set(0); // reset the uptime counter

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

    uart1->Init(iFOC::BoardConfig().GetConfig().uart_1_baudrate());
    uart3->Init(iFOC::DataType::Comm::UARTBaudrate::BAUD_921600);
    can1->Init(iFOC::DataType::Comm::CANBaudrate::BAUD_1_MBPS);

    gpio_bits_set(SPI1_CS1_GPIO_PORT, SPI1_CS1_PIN);
    gpio_bits_set(SPI1_CS2_GPIO_PORT, SPI1_CS2_PIN);

    if(iFOC::BoardConfig().GetConfig().enable_can_terminal_resistor()) gpio_bits_reset(CAN_RES_TRIG_GPIO_PORT, CAN_RES_TRIG_PIN);
    else gpio_bits_set(CAN_RES_TRIG_GPIO_PORT, CAN_RES_TRIG_PIN);

    // usb_init();

    drv830x = new iFOC::Driver::FOCDriverDRV830x(new iFOC::Driver::FOCDriver6PWM(TMR1),
                                                new iFOC::HAL::SPI(SPI1, new iFOC::HAL::GPIO(SPI1_CS1_GPIO_PORT, SPI1_CS1_PIN)),
                                                new iFOC::HAL::GPIO(DRV_EN_GPIO_PORT, DRV_EN_PIN));
    motor_1->LinkDriver(drv830x);
    // motor_1->LinkDriver(new iFOC::Driver::FOCDriver6PWM(TMR1));

    motor_1->LinkCurrSense(new iFOC::Sense::CurrSenseTwoShunts<iFOC::V, iFOC::W>(new iFOC::HAL::ADCPort((uint16_t*)(&ADC3->pdt1), (uint16_t*)&VREFINT_FILTERED),
                                                                                 new iFOC::HAL::ADCPort((uint16_t*)(&ADC2->pdt1), (uint16_t*)&VREFINT_FILTERED),
                                                                                  true, false));
    motor_1->LinkBusSense(new iFOC::Sense::BusSenseSeries(new iFOC::Sense::BusSenseINA237(swi2c, 0x40, false),
                                                        new iFOC::Sense::BusSenseINA237(swi2c, 0x41, true)));

    motor_1->LinkCoreTempSense(new iFOC::Sense::TempSenseCore(new iFOC::HAL::ADCPort((uint16_t*)&MCU_TEMP_FILTERED, (uint16_t*)&VREFINT_FILTERED)));
    motor_1->LinkMosfetTempSense(new iFOC::Sense::TempSenseNTC(new iFOC::HAL::ADCPort((uint16_t*)&NTC_IN_1_FILTERED, (uint16_t*)&VREFINT_FILTERED), 10000.0f));
    motor_1->LinkMotorTempSense(new iFOC::Sense::TempSenseNTC(new iFOC::HAL::ADCPort((uint16_t*)&NTC_IN_2_FILTERED, (uint16_t*)&VREFINT_FILTERED), 10000.0f));

    motor_1->LinkIndicator(new iFOC::HAL::WS2812(TMR3, TMR_SELECT_CHANNEL_4, DMA2_CHANNEL7));

    // motor_1->RegisterProtocol(usb_protocol);
    // motor_1->RegisterProtocol(new iFOC::Protocol::ASCIIProtocol<iFOC::FOCMotor>(uart1));
    motor_1->RegisterProtocol(new iFOC::Protocol::DroneCANProtocol(can1));

    motor_1->Init(true);

    motor_1->AppendEncoder(new iFOC::Encoder::EncoderMT6835(new iFOC::HAL::SPI(SPI1, new iFOC::HAL::GPIO(SPI1_CS2_GPIO_PORT, SPI1_CS2_PIN))));
    if(ABS(motor_1->GetConfig().deduction_ratio() - 8.0f) <= 0.001f) // GIM6010-48 not applicable
    {
        motor_1->AppendEncoder(new iFOC::Encoder::EncoderOffAxisUART(uart3));
        motor_1->SetPrimaryEncoderIndex(0);
    }

    // motor_1->GetDriver()->EnableBridges(iFOC::Driver::FOCDriverBase::Bridge::HB_U,
    //     iFOC::Driver::FOCDriverBase::Bridge::LB_U, iFOC::Driver::FOCDriverBase::Bridge::HB_W, iFOC::Driver::FOCDriverBase::Bridge::LB_W);
    // motor_1->GetDriver()->EnableBridges(iFOC::Driver::FOCDriverBase::Bridge::HB_V, iFOC::Driver::FOCDriverBase::Bridge::LB_V);
    // motor_1->GetDriver()->SetOutput3CHPu(0.5f, 0.5f, 0.3f);

    ADC_Start();
    tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
    tmr_counter_enable(TMR2, TRUE);

    iFOC::HAL::Bootloader::SetAppInitSuccessFlag();

    // uartTask.start("UARTTask", 512, tskIDLE_PRIORITY + 1);
    vTaskStartScheduler();
    while(1);
}