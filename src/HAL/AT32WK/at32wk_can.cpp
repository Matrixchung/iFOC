#include "at32wk_can.hpp"

#if defined(AT32WK_ENV) && defined(CAN_MODULE_ENABLED)

#include "../../Common/foc_math.hpp"

namespace iFOC::HAL
{
CAN::CAN(can_type* _hcan) : hcan(_hcan) {}

FuncRetCode CAN::Init(DataType::Comm::CANBaudrate baud)
{
    can_reset(hcan);

    can_base_type can_base_struct
    {
        .mode_selection = CAN_MODE_COMMUNICATE,
        .ttc_enable = FALSE,
        .aebo_enable = TRUE,   // Automatic exit bus-off
        .aed_enable = TRUE,    // Automatic exit from doze(hibernation)
        .prsf_enable = FALSE,  // PROHIBIT retransmission when sending fails (disable == Auto Retransmission)
        .mdrsel_selection = CAN_DISCARDING_FIRST_RECEIVED,
        .mmssr_selection = CAN_SENDING_BY_REQUEST
    };
    can_base_init(hcan, &can_base_struct);

    can_baudrate_type can_baudrate_struct;
    can_baudrate_default_para_init(&can_baudrate_struct);

    // Using baudcal.exe: Device = CANFDNET_TCP
#ifdef AT32F403Axx // CAN bus on APB1 @ 120 MHz, rsaw(sync jump width) [1,4], bts1(tseg1) [1,16], bts2(tseg2) [1,8]
    switch(baud)
    {
        case DataType::Comm::CANBaudrate::BAUD_1_MBPS:
        {
            can_baudrate_struct.baudrate_div = 10;        // BRP
            can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ; // SJW
            can_baudrate_struct.bts1_size = CAN_BTS1_8TQ; // TSEG1
            can_baudrate_struct.bts2_size = CAN_BTS2_3TQ; // TSEG2
            break;
        }
        case DataType::Comm::CANBaudrate::BAUD_800_KBPS:
        {
            can_baudrate_struct.baudrate_div = 10;
            can_baudrate_struct.rsaw_size = CAN_RSAW_3TQ;
            can_baudrate_struct.bts1_size = CAN_BTS1_11TQ;
            can_baudrate_struct.bts2_size = CAN_BTS2_3TQ;
            break;
        }
        case DataType::Comm::CANBaudrate::BAUD_500_KBPS:
        {
            can_baudrate_struct.baudrate_div = 15;
            can_baudrate_struct.rsaw_size = CAN_RSAW_2TQ;
            can_baudrate_struct.bts1_size = CAN_BTS1_13TQ;
            can_baudrate_struct.bts2_size = CAN_BTS2_2TQ;
            break;
        }
        default: break;
    }
#endif
    can_baudrate_set(hcan, &can_baudrate_struct);

    can_filter_init_type can_filter_init_struct
    {
        .filter_activate_enable = TRUE,
        .filter_mode = CAN_FILTER_MODE_ID_MASK,
        .filter_fifo = CAN_FILTER_FIFO0,
        .filter_number = 0,
        .filter_bit = CAN_FILTER_16BIT,
        .filter_id_high = 0x0000,
        .filter_id_low = 0x0000,
        .filter_mask_high = 0x0000,
        .filter_mask_low = 0x0000
    };

    can_filter_init(hcan, &can_filter_init_struct);

    can_interrupt_enable(hcan, CAN_RF0MIEN_INT | CAN_ETRIEN_INT | CAN_EOIEN_INT, TRUE);

    return FuncRetCode::OK;
}

FuncRetCode CAN::TransmitMessage(DataType::Comm::CANMessage& msg)
{
    // if(xSemaphoreTakeAuto(tx_sem, WRITE_TIMEOUT_MS) == pdTRUE)
    {
        can_tx_message_type tx_msg{};
        if(msg.is_ext)
        {
            tx_msg.standard_id = 0;
            tx_msg.extended_id = msg.cob_id;
            tx_msg.id_type = CAN_ID_EXTENDED;
        }
        else
        {
            tx_msg.standard_id = (uint16_t)msg.cob_id;
            tx_msg.extended_id = 0;
            tx_msg.id_type = CAN_ID_STANDARD;
        }
        tx_msg.frame_type = msg.is_rtr ? CAN_TFT_REMOTE : CAN_TFT_DATA;
        tx_msg.dlc = msg.len;
        memcpy(tx_msg.data, msg.data, msg.len);
        uint8_t selected_mailbox = can_message_transmit(hcan, &tx_msg);

        if(selected_mailbox == CAN_TX_STATUS_NO_EMPTY)
        {
            // xSemaphoreGiveAuto(tx_sem);
            return FuncRetCode::BUFFER_FULL;
        }

        // if(selected_mailbox == CAN_TX_STATUS_NO_EMPTY) // forced wait approach
        // {
        //     const auto start_tick = xTaskGetTickCountFromISR();
        //     while(!hcan->tsts_bit.tm0ef && !hcan->tsts_bit.tm1ef && !hcan->tsts_bit.tm2ef)
        //     {
        //         if(xTaskGetTickCountFromISR() - start_tick >= 10) break;
        //     }
        //     selected_mailbox = can_message_transmit(hcan, &tx_msg);
        //     if(selected_mailbox == CAN_TX_STATUS_NO_EMPTY)
        //     {
        //         xSemaphoreGiveAuto(tx_sem);
        //         return FuncRetCode::BUFFER_FULL;
        //     }
        // }

        // uint8_t max_retry = 32;
        // while(selected_mailbox == CAN_TX_STATUS_NO_EMPTY)
        // {
        //     selected_mailbox = can_message_transmit(hcan, &tx_msg);
        //     if(max_retry > 0) max_retry--;
        //     else break;
        // }
        // if(max_retry == 0)
        // {
        //     xSemaphoreGiveAuto(tx_sem);
        //     return FuncRetCode::BUFFER_FULL;
        // }
        // while(can_transmit_status_get(hcan, (can_tx_mailbox_num_type)selected_mailbox) != CAN_TX_STATUS_SUCCESSFUL) {}

        // xSemaphoreGiveAuto(tx_sem);
        return FuncRetCode::OK;
    }
    return FuncRetCode::REMOTE_TIMEOUT;
}

FuncRetCode CAN::SetHWFilter(uint8_t filter_idx, uint32_t id_u32, uint32_t mask_u32, bool ext_only, bool accept_rtr)
{
    id_u32 <<= 3;
    mask_u32 <<= 3; // SEE DATASHEET MAPPING
    if(ext_only) // EXT ONLY means IDT = 1
    {
        id_u32 |= (1 << 2);
        mask_u32 |= (1 << 2);
    }
    if(!accept_rtr) // do not accept remote frame
    {
        // id_u32 |= (0 << 1);
        mask_u32 |= (1 << 1);
    }
    // CAN_FiFB1[31:21] | CAN_FiFB1[20:3] | CAN_FiFB1[2:0]
    // StandardID[10:0] |   ExtID[17:0]   | IDT   RTR   0
    can_filter_init_type can_filter_init_struct
    {
        .filter_activate_enable = TRUE,
        .filter_mode = CAN_FILTER_MODE_ID_MASK,
        .filter_fifo = CAN_FILTER_FIFO0,
        .filter_number = filter_idx,
        .filter_bit = CAN_FILTER_32BIT,
        .filter_id_high = (uint16_t)(id_u32 >> 16),
        .filter_id_low = (uint16_t)(id_u32 & 0xFFFF),
        .filter_mask_high = (uint16_t)(mask_u32 >> 16),
        .filter_mask_low = (uint16_t)(mask_u32 & 0xFFFF)
    };
    can_filter_init(hcan, &can_filter_init_struct);
    return FuncRetCode::OK;
}

void CAN::OnIRQ() const
{
    // if(can_interrupt_flag_get(hcan, CAN_ETR_FLAG) != RESET) // error irq
    if(hcan->ests_bit.etr)
    {
        // can_flag_clear(hcan, CAN_ETR_FLAG);
        hcan->msts = CAN_MSTS_EOIF_VAL;
        hcan->ests = 0;
    }
    // else if(can_interrupt_flag_get(hcan, CAN_RF0MN_FLAG) != RESET)
    else if(hcan->rf0_bit.rf0mn)
    {
        can_rx_message_type rx_msg{};
        can_message_receive(hcan, CAN_RX_FIFO0, &rx_msg);
        // if(rx_msg.id_type != CAN_ID_STANDARD) return;
        // DataType::Comm::CANMessage dt_can_msg
        // {
        //     .cob_id = (uint16_t)rx_msg.standard_id,
        //     .is_rtr = rx_msg.frame_type == CAN_TFT_REMOTE,
        //     .len = (uint8_t)_constrain(rx_msg.dlc, 0, sizeof(DataType::Comm::CANMessage::data))
        // };
        DataType::Comm::CANMessage dt_can_msg{};
        dt_can_msg.is_ext = rx_msg.id_type == CAN_ID_EXTENDED;
        dt_can_msg.cob_id = dt_can_msg.is_ext ? rx_msg.extended_id : rx_msg.standard_id;
        dt_can_msg.is_rtr = rx_msg.frame_type == CAN_TFT_REMOTE;
        dt_can_msg.len = (uint8_t)_constrain(rx_msg.dlc, 0, sizeof(DataType::Comm::CANMessage::data));
        memcpy(dt_can_msg.data, rx_msg.data, dt_can_msg.len);
        ProcessIncomingMsg(dt_can_msg);
    }
}
}

#endif