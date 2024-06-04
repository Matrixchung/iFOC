## 2024-04-26
* Added AS5048A SPI Encoder

## 2024-04-28
* Rework the implementations of Encoder MT6835/AS5048A/ABZ base, making it easier to port
* Added PWM Encoder with Interrupt Watchdog
* Added auxiliary limiter switch class to start from known initial location (aligning mechanic angle) at motor init, can be implemented as Hall Sensor or Light Sensor
* Added detection of error code: FOC_ERROR_FEEDBACK, also added a new error code: FOC_ERROR_MCU_OVERTEMP

## 2024-04-29
* Added Ramp Limit (also known as output derivative limit) to the PID class, allowing us to limit the maximum acceleration of Position Loop (outputs speed), or the maximum dIq of Velocity Loop (outputs Iq).
(TODO) Implement S-Curve algorithm (Trajectory) to Position Loop 
* Integrated auxiliary limiter switch to the initial align process
* Reworked the communication protocol, and implemented it in USB/UART/CAN physics layer

## 2024-05-06
* Completely reworked communication protocol with "Endpoint", and restructured framework

## 2024-05-07
* Implemented simple CAN Protocol
* Added homing process for limiter switch
* Added virtual_endstop for limiter switch, now it becomes much smarter

## 2024-05-09
* Applied CRTP Templates to Hardware Abstract Layer(HAL) parts which are constant in same hardware board for zero-overhead polymorphism at **compile-time**, including Gate Driver, Current Sense Amplifier, and Bus Current/Voltage Sensor, and leave Estimator with class pointer and virtual functions because we may need to switch Estimator type at runtime, for sensorless/parameter identification use. 
* Restructured communication protocols, now all different physics layer implementations are **interfaces** to BaseProtocol. This will allow multi-interface comm, so we can use both CAN and ASCII(USB, UART) to control the same motor, and EEPROM interface to save configs.
* Fixed the initialization of FOC class, now the template classes are passed through reference, which eliminates overhead.
* Added new error code: FOC_ERROR_ALIGN, which triggers when the align current is not enough to keep the motor static for correct alignment. For example, if the motor keeps spinning during the align steady stage, the following error will be triggered.
* Edited Timer Configuration: **Enable OSSI**, and set CHxN Idle State to High, enabling Active Short Circuit(ASC). Reference: https://zhuanlan.zhihu.com/p/648629584
* Added config.break_mode, 0: 3 Phase 50% Duty; 1: Safety Pulse Off(SPO), 2: Active Short Circuit(ASC)(Default). Reference: https://www.zhihu.com/question/531779213/answer/2830576781, https://zhuanlan.zhihu.com/p/143723798
* Added new error code: FOC_ERROR_OVERSPEED

## 2024-05-10
* Fixed EncoderPWM issue
* Added config.current_bandwidth and config.current_damping_coefficient for Kp/Ki autotune(requires accurate Rs/Ld measurement). Reference: https://blog.csdn.net/weixin_45182459/article/details/136971188


## 2024-05-17
* Removed encoder->UseSpeedPLL(), replaced with encoder->Update()/UpdateMidInterval()
* Added Estimator.UpdateIdleTask()
* SVPWM Max Voltage: Vbus * 2/3
* (TODO)Optimize ADC Sampling strategy
* (TODO)Calculate software UVP/OVP/OCP protection in fast update()
* (TODO)Rework state machine, prevent changing it in high frequency interrupts
* (TODO)Draw stateflow for every state 

## 2024-05-29
* (IMPORTANT)Added Trajectory Controller
* Added Software I2C support, enabling interface to INA226(BusSense) and I2C EEPROM, as well as AS5600 Encoder, etc.

## 2024-06-03/04
* Split BDC and Stepper into independent module
* Implemented Software/Hardware I2C, simplified the interfaces with I2C peripherals.
* Added Trajectory Controller params to BaseProtocol
* Added eeprom_interface with CRC algorithm.
* Fixed BDC part
* Modified Speed PLL and Lowpass Filter classes, enabling modification of parameters