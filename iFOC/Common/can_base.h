/**
 * @file can_base.h
 * @author Matrixchung (xin_zhong@std.uestc.edu.cn)
 * @brief CAN (Controller Area Network) base class, called by CANProtocol.
 *        You must override all virtual functions for your own implementation.
 *        See implementations: iFOC-Port/STM32/can_interface.hpp (for standard CAN)
 *                             iFOC-Port/STM32/canfd_interface.hpp (for CANFD on STM32H7)
 * @version 0.1
 * @date 2024-11-03
 * 
 * Copyright © 2024, iFOC Project
 * 
 */
#ifndef _FOC_CAN_BASE_H
#define _FOC_CAN_BASE_H

#include "stdint.h"

class CANBase
{
public:
    /**
     * @brief Config the CAN Filter. Will be called in CANProtocol.Init() before InitHW().
     *        Done when CAN hardware hasn't been initialized.
     * @param id BaseProtocol.node_id, plus pre-defined global broadcast ID 0x3F.
     * @param mask for Standard CAN Protocol (11-bit ID), device ID (max 0x3F) takes the lower 6 bits, so the higher 5 bits
     *             will be used for masking.
     * @param fifo_index parameter for STM32 CAN, which usually has over 1 CAN mailbox/fifo queue.
     * @param filter_index also for STM32 CAN, with multi filters distinguished by index.
     */
    virtual void ConfigFilter(uint32_t id, uint32_t mask, uint8_t fifo_index, uint8_t filter_index) = 0;

    /**
     * @brief Initialize CAN Hardware. Will be called after ConfigFilter() in CANProtocol.Init().
     * 
     */
    virtual void InitHW() = 0;

    /**
     * @brief Send CAN Payload. This should be a non-blocking function.
     * 
     * @param id node_id which will receives the payload. Maximum is broadcast ID 0x3F.
     * @param payload payload buffer pointer in byte. 
     * @param len payload buffer length, should not exceed hardware limit.
     */
    virtual void SendPayload(uint32_t id, uint8_t *payload, uint8_t len) = 0;
    
    /**
     * @brief Pre-allocated Rx buffer for storing income payload.
     *        Usually handled in platform-dependent CAN Rx IRQ functions.
     * @see   Implementations: iFOC-Port/STM32/can_interface.hpp (for standard CAN)
     *        iFOC-Port/STM32/canfd_interface.hpp (for CANFD on STM32H7)
     */
    uint8_t rx_buffer[16] = {0x00};
};

template<typename T>
concept CANImpl = std::is_base_of<CANBase, T>::value;

#endif