#include "gd32_can.hpp"

#if defined(GD32_ENV)

#ifdef GD32G5X3
// Classic CAN mode
constexpr uint32_t FILTER_FORMAT_AND_NUMBER = CAN_RXFIFO_FILTER_A_NUM_8;
constexpr uint8_t FIRST_AVAIL_TX_MAILBOX_IDX = 8;
constexpr uint8_t LAST_AVAIL_TX_MAILBOX_IDX = 31;
#endif

namespace iFOC::HAL
{
CAN::CAN(uint32_t _hcan) : hcan(_hcan) {}

FuncRetCode CAN::Init(DataType::Comm::CANBaudrate baud)
{
    can_deinit(hcan);

    can_parameter_struct can_param;
    can_struct_para_init(CAN_INIT_STRUCT, &can_param);

    can_param.internal_counter_source = CAN_TIMER_SOURCE_BIT_CLOCK;
    can_param.self_reception = DISABLE;
    // can_param.mb_tx_order = CAN_TX_HIGH_PRIORITY_MB_FIRST;
    can_param.mb_tx_order = CAN_TX_LOW_NUM_MB_FIRST; // FIFO mailbox
    can_param.mb_tx_abort_enable = ENABLE;
    can_param.local_priority_enable = DISABLE;
    can_param.mb_rx_ide_rtr_type = CAN_IDE_RTR_FILTERED;
    can_param.mb_remote_frame = CAN_STORE_REMOTE_REQUEST_FRAME;
    can_param.rx_private_filter_queue_enable = ENABLE; // RPFQEN = 1, we will have separate mask & id for each filter
    can_param.edge_filter_enable = ENABLE; // EFDIS = 0, See User Manual P1321
    can_param.protocol_exception_enable = DISABLE;
    can_param.rx_filter_order = CAN_RX_FILTER_ORDER_FIFO_FIRST; // Rx FIFO first (mailbox not used)
    can_param.memory_size = CAN_MEMSIZE_32_UNIT; // 512 BYTES
    can_param.mb_public_filter = 0U;

#ifdef GD32G5X3
    switch(baud)
    {
        default:
        case DataType::Comm::CANBaudrate::BAUD_1_MBPS:
        {
            can_param.prescaler = 6U; // BRP
            can_param.resync_jump_width = 9U; // SJW
            can_param.prop_time_segment = 13U;
            can_param.time_segment_1 = 13U; // time_segment_1 + pts = TSEG1
            can_param.time_segment_2 = 9U;
            break;
        }
        case DataType::Comm::CANBaudrate::BAUD_800_KBPS:
        {
            can_param.prescaler = 6U;
            can_param.resync_jump_width = 9U;
            can_param.prop_time_segment = 18U;
            can_param.time_segment_1 = 17U;
            can_param.time_segment_2 = 9U;
            break;
        }
        case DataType::Comm::CANBaudrate::BAUD_500_KBPS:
        {
            can_param.prescaler = 6U;
            can_param.resync_jump_width = 9U;
            can_param.prop_time_segment = 31U;
            can_param.time_segment_1 = 31U;
            can_param.time_segment_2 = 9U;
            break;
        }
    }
#else
#warning "CAN baudrate calculation result not set for current platform!"
#endif

    can_init(hcan, &can_param);

    can_auto_busoff_recovery_enable(hcan);
    can_time_sync_disable(hcan);
    can_bsp_mode_config(hcan, CAN_BSP_MODE_THREE_SAMPLES);
    can_bsp_syn_config(hcan, CAN_BSP_TWO_STAGES_SYN);

    // Rx FIFO & Filter settings
    can_fifo_parameter_struct can_fifo_param;
    can_struct_para_init(CAN_FIFO_INIT_STRUCT, &can_fifo_param);

    can_fifo_param.dma_enable = DISABLE;
    can_fifo_param.filter_format_and_number = CAN_RXFIFO_FILTER_A_NUM_8; // Maximum number of 8 filters
    can_fifo_param.fifo_public_filter = 0; // use private mask

    can_rx_fifo_config(hcan, &can_fifo_param);

    /* Mailbox RAM (512 bytes) split in Classic CAN Mode:
     * each mailbox is 16bytes, with a total of 32
     * Rx FIFO: [0:5]
     * FILTER (8, each 4 bytes -> 2 mailbox): [6:7]
     * Tx Mailbox (usable): [8:31] with a total of 24 mailbox.
     */

    // initialize tx mailbox
    for(uint8_t i = FIRST_AVAIL_TX_MAILBOX_IDX; i <= LAST_AVAIL_TX_MAILBOX_IDX; i++)
    {
        can_mailbox_transmit_inactive(hcan, i);
        CAN_STAT(hcan) = STAT_MS(i);
        (void)(CAN_TIMER(hcan));
    }

    can_interrupt_enable(hcan, CAN_INT_FIFO_AVAILABLE);

    can_operation_mode_enter(hcan, CAN_NORMAL_MODE);

    return FuncRetCode::OK;
}

FuncRetCode CAN::TransmitMessage(DataType::Comm::CANMessage& msg)
{
    // for(uint8_t i = FIRST_AVAIL_TX_MAILBOX_IDX; i <= LAST_AVAIL_TX_MAILBOX_IDX; i++)
    constexpr uint8_t max_retry_time = (LAST_AVAIL_TX_MAILBOX_IDX - FIRST_AVAIL_TX_MAILBOX_IDX + 1) * 2;
    uint8_t retry_time = 0;
    while(true)
    {
        if(pending_tx_mailbox_idx < FIRST_AVAIL_TX_MAILBOX_IDX || pending_tx_mailbox_idx > LAST_AVAIL_TX_MAILBOX_IDX) pending_tx_mailbox_idx = FIRST_AVAIL_TX_MAILBOX_IDX;
        if(!IsMailboxEmpty(pending_tx_mailbox_idx))
        {
            pending_tx_mailbox_idx++;
            retry_time++;
            if(retry_time >= max_retry_time) return FuncRetCode::BUFFER_FULL;
            continue;
        }
        can_mailbox_descriptor_struct desc;
        can_struct_para_init(CAN_MDSC_STRUCT, &desc);
        desc.ide = msg.is_ext;
        desc.rtr = msg.is_rtr;
        desc.id = msg.cob_id;
        desc.prio = 0U;
        desc.data_bytes = msg.is_rtr ? 0U : msg.len;
        desc.code = CAN_MB_TX_STATUS_DATA;
        memcpy(desc.data, msg.data, desc.data_bytes);
        can_mailbox_config(hcan, pending_tx_mailbox_idx, &desc);
        return FuncRetCode::OK;
    }
    // return FuncRetCode::BUFFER_FULL;
}

FuncRetCode CAN::SetHWFilter(const uint8_t filter_idx, const uint32_t id_u32, const uint32_t mask_u32, const bool ext_only, const bool accept_rtr)
{
    if(filter_idx >= 8) return FuncRetCode::PARAM_OUT_BOUND;
    const auto previous_mode = can_operation_mode_get(hcan);
    if(previous_mode != CAN_INACTIVE_MODE) can_operation_mode_enter(hcan, CAN_INACTIVE_MODE);
    // Use FIFO filter format Type A
    //  31    30    29    [28:16]       [15:0]
    // RTR_A IDE_A  RS  EXD_A[28:16]  EXD_A[15:0]
    uint32_t filter_val = 0;
    uint32_t filter_mask = 0;
    if(ext_only)
    {
        filter_val |= CAN_FDESX_IDE_A;
        filter_val |= FIFO_FILTER_ID_EXD_A(id_u32 & 0x1FFFFFFFU);

        filter_mask |= (mask_u32 & 0x1FFFFFFFU); // mask extended id [28:0]
        filter_mask |= CAN_RFIFOMPF_FMFD30; // also mask IDE (extended) bit (30)
    }
    else
    {
        // IDE = 0, standard frame only
        filter_val |= FIFO_FILTER_ID_STD_A(id_u32 & 0x7FFU);

        filter_mask |= ((mask_u32 & 0x7FFU) << 18);

        filter_mask |= CAN_RFIFOMPF_FMFD30; // also mask IDE (extended) bit (30), IDE is forced to 0
    }

    if(!accept_rtr)
    {
        filter_mask |= CAN_RFIFOMPF_FMFD31; // also mask RTR_A, RTR is forced to 0
    }

    auto *filter_table = (volatile uint32_t*)(CAN_RAM(hcan) + 0x60U);
    filter_table[filter_idx] = filter_val;

    can_private_filter_config(hcan, filter_idx, filter_mask);

    can_operation_mode_enter(hcan, previous_mode);

    return FuncRetCode::OK;
}

void CAN::OnIRQ() const
{
    // CAN_INT_FLAG_FIFO_AVAIL == CAN_FLAG_FIFO_AVAIL
    while(can_interrupt_flag_get(hcan, CAN_INT_FLAG_FIFO_AVAILABLE))
    {
        can_rx_fifo_struct rx_msg;
        can_rx_fifo_read(hcan, &rx_msg);
        DataType::Comm::CANMessage msg;
        msg.is_ext = (rx_msg.ide != 0);
        msg.cob_id = rx_msg.id;
        msg.is_rtr = (rx_msg.rtr != 0);
        msg.len = msg.is_rtr ? 0 : (uint8_t)_constrain(rx_msg.dlc, 0, sizeof(DataType::Comm::CANMessage::data));
        memcpy(msg.data, rx_msg.data, msg.len);
        ProcessIncomingMsg(msg);
        // can_interrupt_flag_clear(hcan, CAN_INT_FLAG_FIFO_AVAILABLE);
        (void)CAN_TIMER(hcan);
    }
}

bool CAN::IsMailboxEmpty(const uint8_t mb) const
{
    if(CAN_STAT(hcan) & STAT_MS(mb))
    {
        CAN_STAT(hcan) = STAT_MS(mb);
        (void)CAN_TIMER(hcan);
    }
    const auto status_code = can_mailbox_code_get(hcan, mb);
    if(status_code == CAN_MB_TX_STATUS_INACTIVE || status_code == CAN_MB_TX_STATUS_ABORT) return true;
    return false;
}
}

#endif