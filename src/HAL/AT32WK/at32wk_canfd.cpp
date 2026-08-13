#include "at32wk_canfd.hpp"

#if defined(AT32WK_ENV) && defined(CAN_MODULE_ENABLED) && defined(IFOC_CANFD_AVAILABLE)

namespace iFOC::HAL
{
    CANFD::CANFD(can_type* _hcan) : hcan(_hcan) {}

    FuncRetCode CANFD::Init(const DataType::Comm::CANBaudrate arbit_baud, const DataType::Comm::CANBaudrate data_baud)
    {
        if(crm_flag_get(CRM_HEXT_STABLE_FLAG) != SET) return FuncRetCode::HARDWARE_ERROR; // HEXT clock unstable
        can_reset(hcan);
#ifdef CAN1
        if(hcan == CAN1)
        {
            crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
            crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);
            can_software_reset(hcan, TRUE);
        }
#ifdef CAN2
        else if(hcan == CAN2)
        {
            crm_periph_clock_enable(CRM_CAN2_PERIPH_CLOCK, TRUE);
            crm_can_clock_select(CRM_CAN2, CRM_CAN_CLOCK_SOURCE_PCLK);
            can_software_reset(hcan, TRUE);
        }
#endif
#ifdef CAN3
        else if(hcan == CAN3)
        {
            crm_periph_clock_enable(CRM_CAN3_PERIPH_CLOCK, TRUE);
            crm_can_clock_select(CRM_CAN3, CRM_CAN_CLOCK_SOURCE_PCLK);
            can_software_reset(hcan, TRUE);
        }
#endif
#else
        return FuncRetCode::NOT_SUPPORTED;
#endif
        can_bittime_type can_bittime_struct;
        can_bittime_default_para_init(&can_bittime_struct);
#ifdef AT32F456xx
        // Setting up arbitration baudrate
        can_bittime_struct.bittime_div = 1;
        // baseline bts1, bts2 & rsaw here (1Mbps)
        // can_bittime_struct.ac_bts1_size = 144;
        // can_bittime_struct.ac_bts2_size = 48;
        // can_bittime_struct.ac_rsaw_size = 48;
        switch(arbit_baud)
        {
            case DataType::Comm::CANBaudrate::BAUD_6_MBPS:
            {
                can_bittime_struct.ac_bts1_size = 24;
                can_bittime_struct.ac_bts2_size = 8;
                can_bittime_struct.ac_rsaw_size = 8;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_4_MBPS:
            {
                can_bittime_struct.ac_bts1_size = 36;
                can_bittime_struct.ac_bts2_size = 12;
                can_bittime_struct.ac_rsaw_size = 12;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_3_MBPS:
            {
                can_bittime_struct.ac_bts1_size = 48;
                can_bittime_struct.ac_bts2_size = 16;
                can_bittime_struct.ac_rsaw_size = 16;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_2_MBPS:
            {
                can_bittime_struct.ac_bts1_size = 72;
                can_bittime_struct.ac_bts2_size = 24;
                can_bittime_struct.ac_rsaw_size = 24;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_1_MBPS:
            {
                can_bittime_struct.ac_bts1_size = 144;
                can_bittime_struct.ac_bts2_size = 48;
                can_bittime_struct.ac_rsaw_size = 48;
                break;
            }
            default: return FuncRetCode::NOT_SUPPORTED; // 8Mbps arbit is not supported
        }
        switch(data_baud)
        {
            case DataType::Comm::CANBaudrate::BAUD_2_MBPS:
            {
                can_bittime_struct.fd_bts1_size = 72;
                can_bittime_struct.fd_bts2_size = 24;
                can_bittime_struct.fd_rsaw_size = 24;
                can_bittime_struct.fd_ssp_offset = 73;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_6_MBPS:
            {
                can_bittime_struct.fd_bts1_size = 24;
                can_bittime_struct.fd_bts2_size = 8;
                can_bittime_struct.fd_rsaw_size = 8;
                can_bittime_struct.fd_ssp_offset = 25;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_8_MBPS:
            {
                can_bittime_struct.fd_bts1_size = 18;
                can_bittime_struct.fd_bts2_size = 6;
                can_bittime_struct.fd_rsaw_size = 6;
                can_bittime_struct.fd_ssp_offset = 19;
                break;
            }
            case DataType::Comm::CANBaudrate::BAUD_5_MBPS: // 192MHz no divider found??
            {
                return FuncRetCode::NOT_SUPPORTED;
            }
            case DataType::Comm::CANBaudrate::BAUD_10_MBPS:
            {
                return FuncRetCode::NOT_SUPPORTED;
            }
            case DataType::Comm::CANBaudrate::BAUD_1_MBPS:
            {
                can_bittime_struct.fd_bts1_size = 144;
                can_bittime_struct.fd_bts2_size = 48;
                can_bittime_struct.fd_rsaw_size = 48;
                can_bittime_struct.fd_ssp_offset = 145;
                break;
            }
            default: return FuncRetCode::NOT_SUPPORTED;
        }
#else
        return FuncRetCode::NOT_SUPPORTED;
#endif
        can_bittime_set(hcan, &can_bittime_struct);

        can_fd_iso_mode_enable(hcan, TRUE); // ISO CAN-FD
        can_stb_transmit_mode_set(hcan, CAN_STB_TRANSMIT_BY_FIFO);
        can_rxbuf_overflow_mode_set(hcan, CAN_RXBUF_OVERFLOW_BE_OVWR); // override oldest frame if overflow

        can_software_reset(hcan, FALSE); // disable software reset mode

        // can_retransmission_limit_set(hcan, CAN_RE_TRANS_TIMES_UNLIMIT);
        // can_rearbitration_limit_set(hcan, CAN_RE_ARBI_TIMES_UNLIMIT);
        can_retransmission_limit_set(hcan, CAN_RE_TRANS_TIMES_3);
        can_rearbitration_limit_set(hcan, CAN_RE_ARBI_TIMES_UNLIMIT);
        // can_rxbuf_warning_set(hcan, 3);
        can_mode_set(hcan, CAN_MODE_COMMUNICATE);
        // can_mode_set(hcan, CAN_MODE_EXT_LOOPBACK_ACK);

        // can_timestamp_position_set(hcan, CAN_TIMESTAMP_AT_SOF);
        // can_timestamp_enable(hcan, TRUE);
        can_error_warning_set(hcan, 11);
        can_restricted_operation_enable(hcan, FALSE);
        can_receive_all_enable(hcan, FALSE);

        // can_interrupt_enable(hcan, CAN_RAFIE_INT, TRUE); // Rx Buffer almost full interrupt
        // can_interrupt_enable(hcan, CAN_RFIE_INT, TRUE); // Rx Buffer full interrupt
        // can_interrupt_enable(hcan, CAN_ROIE_INT, TRUE); // Rx Buffer overflow interrupt
        can_interrupt_enable(hcan, CAN_RIE_INT, TRUE); // Receiver interrupt
        can_interrupt_enable(hcan, CAN_BEIE_INT, TRUE); // Bus Error Interrupt

        return FuncRetCode::OK;
    }

    FuncRetCode CANFD::TransmitMessage(DataType::Comm::CANFDMessage& msg)
    {
        can_txbuf_type tx_buf;
        tx_buf.id = (uint32_t)(msg.cob_id[0]) | ((uint32_t)msg.cob_id[1] << 8) | ((uint32_t)msg.cob_id[2] << 16) | ((uint32_t)msg.cob_id[3] << 24);
        tx_buf.id_type = msg.is_ext ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
        tx_buf.frame_type = CAN_FRAME_DATA;
        tx_buf.fd_format = CAN_FORMAT_FD;
        tx_buf.fd_rate_switch = CAN_BRS_ON; // Baudrate switch on
        switch(msg.dlc)
        {
            case DataType::Comm::canfd_dlc::DLC_1_BYTES: tx_buf.data_length = CAN_DLC_BYTES_1; break;
            case DataType::Comm::canfd_dlc::DLC_2_BYTES: tx_buf.data_length = CAN_DLC_BYTES_2; break;
            case DataType::Comm::canfd_dlc::DLC_3_BYTES: tx_buf.data_length = CAN_DLC_BYTES_3; break;
            case DataType::Comm::canfd_dlc::DLC_4_BYTES: tx_buf.data_length = CAN_DLC_BYTES_4; break;
            case DataType::Comm::canfd_dlc::DLC_5_BYTES: tx_buf.data_length = CAN_DLC_BYTES_5; break;
            case DataType::Comm::canfd_dlc::DLC_6_BYTES: tx_buf.data_length = CAN_DLC_BYTES_6; break;
            case DataType::Comm::canfd_dlc::DLC_7_BYTES: tx_buf.data_length = CAN_DLC_BYTES_7; break;
            case DataType::Comm::canfd_dlc::DLC_8_BYTES: tx_buf.data_length = CAN_DLC_BYTES_8; break;
            case DataType::Comm::canfd_dlc::DLC_12_BYTES: tx_buf.data_length = CAN_DLC_BYTES_12; break;
            case DataType::Comm::canfd_dlc::DLC_16_BYTES: tx_buf.data_length = CAN_DLC_BYTES_16; break;
            case DataType::Comm::canfd_dlc::DLC_20_BYTES: tx_buf.data_length = CAN_DLC_BYTES_20; break;
            case DataType::Comm::canfd_dlc::DLC_24_BYTES: tx_buf.data_length = CAN_DLC_BYTES_24; break;
            case DataType::Comm::canfd_dlc::DLC_32_BYTES: tx_buf.data_length = CAN_DLC_BYTES_32; break;
            case DataType::Comm::canfd_dlc::DLC_48_BYTES: tx_buf.data_length = CAN_DLC_BYTES_48; break;
            case DataType::Comm::canfd_dlc::DLC_64_BYTES: tx_buf.data_length = CAN_DLC_BYTES_64; break;
            default: tx_buf.data_length = CAN_DLC_BYTES_0; break;
        }
        memcpy(tx_buf.data, msg.data, static_cast<uint8_t>(msg.dlc));
        tx_buf.handle = 0;
        // tx_buf.tx_timestamp = TRUE; // FALSE?
        tx_buf.tx_timestamp = FALSE;
        tx_buf.handle = 0;
        // if(can_flag_get(hcan, CAN_TPIF_FLAG)) can_flag_clear(hcan, CAN_TPIF_FLAG);
        // if(can_flag_get(hcan, CAN_TSIF_FLAG)) can_flag_clear(hcan, CAN_TSIF_FLAG);
        if(can_txbuf_write(hcan, CAN_TXBUF_PTB, &tx_buf) == SUCCESS) // first try PTB (priority buffer)
        {
            if(can_txbuf_transmit(hcan, CAN_TRANSMIT_PTB) != SUCCESS) return FuncRetCode::REMOTE_TIMEOUT; // write OK but Tx failed
        }
        else if(can_txbuf_write(hcan, CAN_TXBUF_STB, &tx_buf) == SUCCESS) // STB
        {
            if(can_txbuf_transmit(hcan, CAN_TRANSMIT_STB_ALL) != SUCCESS) return FuncRetCode::REMOTE_TIMEOUT; // STB_ALL
        }
        else return FuncRetCode::BUFFER_FULL;
        return FuncRetCode::OK;
    }

    FuncRetCode CANFD::SetHWFilter(uint8_t filter_idx, uint32_t id_u32, uint32_t mask_u32, bool ext_only)
    {
        // See: AN0232_AT32M412_416_CAN_Application_Note
        // In AT32 CANFD implementation, FILTER_MASK means "not care", \n
        // For example, CODE = 0x18F5F100, MASK = 0x000000FF (EXT ID)
        // then id can receive from 0x18F5F100 to 0x18F5F1FF. \n
        // CODE = 0x04F0, MASK = 0x000F (STD ID)
        // then id can receive from 0x04F0 to 0x04FF.
        if(filter_idx > CAN_FILTER_NUM_15) return FuncRetCode::INVALID_INPUT;

        can_filter_config_type filter;
        can_filter_default_para_init(&filter);
        filter.code_para.id = id_u32;
        filter.code_para.id_type = ext_only ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
        filter.mask_para.id = ~mask_u32; // INVERTED MASK!!
        filter.mask_para.id_type = FALSE; // no ignore, mask the selected code_param.id_type
        filter.mask_para.data_length = 0xF; // ignore data length
        filter.mask_para.frame_type = TRUE; // ignore frame type
        filter.mask_para.fd_error_state = TRUE; // ignore error state
        filter.mask_para.fd_format = TRUE; // ignore frame format
        filter.mask_para.fd_rate_switch = TRUE; // ignore BRS
        filter.mask_para.recv_frame = TRUE; // ignore recv_frame

        // "CAN critical area"
        can_software_reset(hcan, TRUE);
        can_filter_set(hcan, (can_filter_type)filter_idx, &filter);
        can_software_reset(hcan, FALSE);

        can_filter_enable(hcan, (can_filter_type)filter_idx, TRUE);

        return FuncRetCode::OK;
    }

    void CANFD::OnIRQ() const
    {
        // Process ALL messages from Rx FIFO
        if(can_interrupt_flag_get(hcan, CAN_RIF_FLAG) != RESET)
        {
            can_rxbuf_type rx_buf;
            DataType::Comm::CANFDMessage msg;
            can_flag_clear(hcan, CAN_RIF_FLAG);
            while(can_rxbuf_read(hcan, &rx_buf) != ERROR)
            {
                msg.cob_id[0] = rx_buf.id & 0xFF;
                msg.cob_id[1] = rx_buf.id >> 8;
                msg.cob_id[2] = rx_buf.id >> 16;
                msg.cob_id[3] = rx_buf.id >> 24;
                msg.is_ext = rx_buf.id_type == CAN_ID_EXTENDED;
                switch(rx_buf.data_length)
                {
                    case CAN_DLC_BYTES_1: msg.dlc = DataType::Comm::canfd_dlc::DLC_1_BYTES; break;
                    case CAN_DLC_BYTES_2: msg.dlc = DataType::Comm::canfd_dlc::DLC_2_BYTES; break;
                    case CAN_DLC_BYTES_3: msg.dlc = DataType::Comm::canfd_dlc::DLC_3_BYTES; break;
                    case CAN_DLC_BYTES_4: msg.dlc = DataType::Comm::canfd_dlc::DLC_4_BYTES; break;
                    case CAN_DLC_BYTES_5: msg.dlc = DataType::Comm::canfd_dlc::DLC_5_BYTES; break;
                    case CAN_DLC_BYTES_6: msg.dlc = DataType::Comm::canfd_dlc::DLC_6_BYTES; break;
                    case CAN_DLC_BYTES_7: msg.dlc = DataType::Comm::canfd_dlc::DLC_7_BYTES; break;
                    case CAN_DLC_BYTES_8: msg.dlc = DataType::Comm::canfd_dlc::DLC_8_BYTES; break;
                    case CAN_DLC_BYTES_12: msg.dlc = DataType::Comm::canfd_dlc::DLC_12_BYTES; break;
                    case CAN_DLC_BYTES_16: msg.dlc = DataType::Comm::canfd_dlc::DLC_16_BYTES; break;
                    case CAN_DLC_BYTES_20: msg.dlc = DataType::Comm::canfd_dlc::DLC_20_BYTES; break;
                    case CAN_DLC_BYTES_24: msg.dlc = DataType::Comm::canfd_dlc::DLC_24_BYTES; break;
                    case CAN_DLC_BYTES_32: msg.dlc = DataType::Comm::canfd_dlc::DLC_32_BYTES; break;
                    case CAN_DLC_BYTES_48: msg.dlc = DataType::Comm::canfd_dlc::DLC_48_BYTES; break;
                    case CAN_DLC_BYTES_64: msg.dlc = DataType::Comm::canfd_dlc::DLC_64_BYTES; break;
                    default: msg.dlc = DataType::Comm::canfd_dlc::DLC_0_BYTES; break;
                }
                memcpy(msg.data, rx_buf.data, static_cast<uint8_t>(msg.dlc));
                ProcessIncomingMsg(msg);
            }
        }

        /* rx_buffer almost full */
        if(can_interrupt_flag_get(hcan, CAN_RAFIF_FLAG) != RESET)
        {
            can_flag_clear(hcan, CAN_RAFIF_FLAG);
        }
        /* rx_buffer full */
        if(can_interrupt_flag_get(hcan, CAN_RFIF_FLAG) != RESET)
        {
            can_flag_clear(hcan, CAN_RFIF_FLAG);
        }
        /* rx_buffer overflow */
        if(can_interrupt_flag_get(hcan, CAN_ROIF_FLAG) != RESET)
        {
            can_flag_clear(hcan, CAN_ROIF_FLAG);
        }
    }

    void CANFD::OnErrorIRQ() const
    {
        // Bus Error
        if(can_interrupt_flag_get(hcan, CAN_BEIF_FLAG) != RESET)
        {
            can_flag_clear(hcan, CAN_BEIF_FLAG);
        }
    }
}

#endif