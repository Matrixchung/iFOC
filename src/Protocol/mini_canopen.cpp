#include "mini_canopen.hpp"
#include "cpp_classes.hpp"

#define SLSS_ADDRESS 0x7E4
#define MLSS_ADDRESS 0x7E5

#define SDO_TIMEOUT_MS 2000 // Only through timeout can error transfer line be reseted.

// #define DEBUG_PRINT(...) uart_1.Print(true, __VA_ARGS__)

#ifndef DEBUG_PRINT(...)
#define DEBUG_PRINT(...)
#endif

namespace iFOC
{
MiniCANOpen::MiniCANOpen(MiniCANOpen::CANSendFunc func)
{
    can_send = func;
    // sdo_transfers.reserve(SDO_MAX_SIMULTANEOUS_TRANSFERS);
    sdo_transfers.resize(SDO_MAX_SIMULTANEOUS_TRANSFERS);
}

MiniCANOpen::~MiniCANOpen()
{
    for(auto& object : m_objectDict)
    {
        for(auto& subIndex : object.second.subIndexes)
        {
            if(subIndex.pObject != nullptr)
            {
                allocator.deallocate(subIndex.pObject, 1);
                subIndex.pObject = nullptr;
            }
        }
    }
}

void MiniCANOpen::OnRxMessage(const DataType::Comm::CANMessage& msg)
{
    // DEBUG_PRINT("Rx: %d\n", msg.cob_id);
    auto func_code = GetFuncCodeFromCOB(msg.cob_id);
    auto target_node_id = GetNodeIDFromCOB(msg.cob_id);
    switch(func_code)
    {
        case NMT:
        {
            ProcessNMTMessage(msg);
            break;
        }
        case SYNC:
        {
            if(target_node_id == 0x00) // global broadcast
            {
                if(curr_handle_state.handle_sync) ProcessSYNCMessage(msg);
            }
            else if(curr_handle_state.handle_emergency) ProcessEMCYMessage(msg);
            break;
        }
        case TX_PDO_1:
        case RX_PDO_1:
        case TX_PDO_2:
        case RX_PDO_2:
        case TX_PDO_3:
        case RX_PDO_3:
        case TX_PDO_4:
        case RX_PDO_4:
        {
            if(curr_handle_state.handle_pdo) ProcessPDOMessage(msg);
            break;
        }
        case TX_SDO:
        case RX_SDO:
        {
            if(curr_handle_state.handle_sdo) ProcessSDOMessage(msg);
            break;
        }
        case LSS:
        {
            if(curr_handle_state.handle_lss && msg.cob_id == MLSS_ADDRESS)
            {
                ProcessLSSMessage(msg);
            }
            break;
        }
        default: break;
    }
}

void MiniCANOpen::Tick(uint32_t ms)
{
    alarm_timer.UpdateTimer(ms);
}

void MiniCANOpen::SetNodeState(NodeState new_state)
{
    if(new_state != m_NodeState || new_state == NodeState::Initialisation)
    {
        switch(new_state)
        {
            case NodeState::Initialisation:
            {
                HandleState new_handle_state = { true,
                                                false,
                                                false,
                                                false,
                                                false,
                                                false,
                                                false};
                if(node_state_callback) node_state_callback(m_NodeState, new_state);
                m_NodeState = NodeState::Initialisation;
                SwitchCurrentHandleState(new_handle_state);
                /* Automatic transition - No break statement! */
                /* Transition from Initialisation to PreOperational */
                /* is automatic as defined in DS301. */
                /* App doesn't have to call SetNodeState(PreOperational) */
            }
            case NodeState::PreOperational:
            {
                HandleState new_handle_state = {false,
                                                true,
                                                true,
                                                true,
                                                true,
                                                false,
                                                true};
                if(node_state_callback) node_state_callback(m_NodeState, new_state);
                m_NodeState = NodeState::PreOperational;
                SwitchCurrentHandleState(new_handle_state);
                break;
            }
            case NodeState::Operational:
            {
                if(m_NodeState == NodeState::Initialisation) break; // Directly from Initialisation to Operational is not allowed
                HandleState new_handle_state = {false,
                                                true,
                                                true,
                                                true,
                                                true,
                                                true,
                                                false};
                if(node_state_callback) node_state_callback(m_NodeState, new_state);
                m_NodeState = NodeState::Operational;
                SwitchCurrentHandleState(new_handle_state);
                break;
            }
            case NodeState::Stopped:
            {
                if(m_NodeState == NodeState::Initialisation) break; // Directly from init to Stopped is not allowed
                HandleState new_handle_state = {false,
                                                false,
                                                false,
                                                false,
                                                true,
                                                false,
                                                true};
                if(node_state_callback) node_state_callback(m_NodeState, new_state);
                m_NodeState = NodeState::Stopped;
                SwitchCurrentHandleState(new_handle_state);
                break;
            }
            default: break;
        }
    }
}

void MiniCANOpen::SetEMCYError(uint16_t eec, uint8_t error_reg, uint32_t mef)
{
    WriteODValue(0x1001, 0x00, sizeof(uint8_t), &error_reg); // Error Register
    uint8_t temp = 1;
    WriteODValue(0x1003, 0x00, sizeof(uint8_t), &temp); // Number of errors
    WriteODValue(0x1003, 0x01, sizeof(uint32_t), &mef); // Standard Error Field
    // Emergency COB id from 0x1014:00
    uint8_t emcy_cob_id = 0x80;
    uint8_t actual_size = 0;
    ReadODValue(0x1014, 0x00, actual_size, &emcy_cob_id);
    if(actual_size < 1) return;
    DataType::Comm::CANMessage msg
    {
        .cob_id = emcy_cob_id,
        .is_rtr = false,
        .len = 8
    };
    msg.data[0] = eec & 0xFF;
    msg.data[1] = (eec >> 8) & 0xFF;
    msg.data[2] = error_reg;
    msg.data[3] = (uint8_t)mef;
    msg.data[4] = (uint8_t)(mef >> 8);
    msg.data[5] = (uint8_t)(mef >> 16);
    msg.data[6] = (uint8_t)(mef >> 24);
    can_send(msg);
}

void MiniCANOpen::ClearEMCYError()
{
    uint32_t temp = 0;
    WriteODValue(0x1001, 0x00, sizeof(uint8_t), &temp); // Clear Error Register
    WriteODValue(0x1003, 0x01, sizeof(uint32_t), &temp); // Clear Standard Error Field
    // Emergency COB id from 0x1014:00
    uint8_t emcy_cob_id = 0x80;
    uint8_t actual_size = 0;
    ReadODValue(0x1014, 0x00, actual_size, &emcy_cob_id);
    if(actual_size < 1) return;
    DataType::Comm::CANMessage msg
    {
        .cob_id = emcy_cob_id,
        .is_rtr = false,
        .len = 8
    };
    memset(msg.data, 0, sizeof(msg.data));
    can_send(msg);
}

void MiniCANOpen::AddODEntry(const ODEntry& entry)
{
    m_objectDict[entry.index] = entry;
}

FuncRetCode MiniCANOpen::SetVendorID(uint32_t vendor_id)
{
    return WriteODValue(0x1018, 0x01, 4, &vendor_id);
}

FuncRetCode MiniCANOpen::SetProductCode(uint32_t product_code)
{
    return WriteODValue(0x1018, 0x02, 4, &product_code);
}

FuncRetCode MiniCANOpen::SetRevisionNumber(uint32_t revision_number)
{
    return WriteODValue(0x1018, 0x03, 4, &revision_number);
}

FuncRetCode MiniCANOpen::SetSerialNumber(uint32_t serial_number)
{
    return WriteODValue(0x1018, 0x04, 4, &serial_number);
}

FuncRetCode MiniCANOpen::WriteODValue(uint16_t index, uint8_t subindex, uint8_t value_size, const void* value, bool bypass)
{
    auto entry = FindODEntry(index);
    if(!entry || subindex >= entry.value()->subIndexes.size()) return FuncRetCode::PARAM_NOT_EXIST;
    auto& sub = entry.value()->subIndexes[subindex];
    if(!bypass && sub.accessType == ODAccessType::RO) return FuncRetCode::ACCESS_VIOLATION; // Disable access check
    if(value_size > sub.size) return FuncRetCode::PARAM_OUT_BOUND;
    memcpy(sub.pObject, value, value_size);
    if(entry.value()->rw_callback) entry.value()->rw_callback(ODRWType::WRITE, entry.value(), subindex);
    return FuncRetCode::OK;
}

FuncRetCode MiniCANOpen::ReadODValue(uint16_t index, uint8_t subindex, uint8_t& actual_size, void* dest, bool bypass)
{
    auto entry = FindODEntry(index);
    if(!entry || subindex >= entry.value()->subIndexes.size())
    {
        actual_size = 0;
        return FuncRetCode::PARAM_NOT_EXIST;
    }
    auto& sub = entry.value()->subIndexes[subindex];
    if(!bypass && sub.accessType == ODAccessType::WO) return FuncRetCode::ACCESS_VIOLATION;
    actual_size = sub.size;
    memcpy(dest, sub.pObject, actual_size);
    if(entry.value()->rw_callback) entry.value()->rw_callback(ODRWType::READ, entry.value(), subindex);
    return FuncRetCode::OK;
}

uint8_t MiniCANOpen::GetODValueSize(uint16_t index, uint8_t subindex)
{
    auto entry = FindODEntry(index);
    if(!entry || subindex >= entry.value()->subIndexes.size()) return 0;
    auto& sub = entry.value()->subIndexes[subindex];
    return sub.size;
}

std::optional<MiniCANOpen::ODEntry*> MiniCANOpen::FindODEntry(const uint16_t index)
{
    if(const auto& it = m_objectDict.find(index); it != m_objectDict.end()) return &it->second;
    return std::nullopt;
}

uint8_t MiniCANOpen::GetNodeID() const
{
    return m_NodeId;
}

void MiniCANOpen::SetNodeID(uint8_t node_id)
{
    if(!(node_id > 0 && node_id < 0xFF)) return;
    // Change Object Dictionary ID first
    uint32_t temp = 0x00;
    // Emergency COB ID
    temp = 0x80 + node_id;
    WriteODValue(0x1014, 0x00, sizeof(temp), &temp);
    // TPDO 1 (0x180 + node_id)
    temp = 0x180 + node_id;
    WriteODValue(0x1800, 0x01, sizeof(temp), &temp);
    // TPDO 2 (0x280 + node_id)
    temp = 0x280 + node_id;
    WriteODValue(0x1801, 0x01, sizeof(temp), &temp);
    // TPDO 3 (0x380 + node_id)
    temp = 0x380 + node_id;
    WriteODValue(0x1802, 0x01, sizeof(temp), &temp);
    // TPDO 4 (0x480 + node_id)
    temp = 0x480 + node_id;
    WriteODValue(0x1803, 0x01, sizeof(temp), &temp);
    // RPDO 1 (0x200 + node_id)
    temp = 0x200 + node_id;
    WriteODValue(0x1400, 0x01, sizeof(temp), &temp);
    // RPDO 2 (0x300 + node_id)
    temp = 0x300 + node_id;
    WriteODValue(0x1401, 0x01, sizeof(temp), &temp);
    // RPDO 3 (0x400 + node_id)
    temp = 0x400 + node_id;
    WriteODValue(0x1402, 0x01, sizeof(temp), &temp);
    // RPDO 4 (0x500 + node_id)
    temp = 0x500 + node_id;
    WriteODValue(0x1403, 0x01, sizeof(temp), &temp);
    // SDO Parameter
    temp = 0x600 + node_id;
    WriteODValue(0x1200, 0x01, sizeof(temp), &temp);
    temp = 0x580 + node_id;
    WriteODValue(0x1200, 0x02, sizeof(temp), &temp);
    m_NodeId = node_id;
}

void MiniCANOpen::RegisterNMTCallback(NMTCallback cb)
{
    nmt_callback = cb;
}

void MiniCANOpen::RegisterNodeStateCallback(NodeStateCallback cb)
{
    node_state_callback = cb;
}

void MiniCANOpen::RegisterLSSCallback(LSSCallback cb)
{
    lss_transfer.callback = cb;
}

void MiniCANOpen::RegisterLSSStoreCallback(LSSStoreCallback cb)
{
    lss_transfer.store_callback = cb;
}

void MiniCANOpen::RegisterEMCYCallback(EMCYCallback cb)
{
    emcy_callback = cb;
}

// void MiniCANOpen::RegisterSDOCallback(SDOCallback cb)
// {
//     sdo_callback = cb;
// }

FuncRetCode MiniCANOpen::SDOTransfer::push_back(const uint8_t* ptr, size_t count)
{
    if((data.size() + count) > SDO_MAX_TRANSFER_SIZE) return FuncRetCode::BUFFER_FULL;
    data.insert(data.end(), ptr, ptr + count);
    return FuncRetCode::OK;
}

FuncRetCode MiniCANOpen::SDOTransfer::extract_from_head(uint8_t* dst, size_t count)
{
    if(count == 0) return FuncRetCode::INVALID_INPUT;
    if(data.empty()) return FuncRetCode::INVALID_RESULT;
    if(count > data.size()) return FuncRetCode::PARAM_OUT_BOUND;
    std::copy_n(data.begin(), count, dst);
    data.erase(data.begin(), data.begin() + count);
    return FuncRetCode::OK;
}

void MiniCANOpen::ProcessNMTMessage(const DataType::Comm::CANMessage& msg)
{
    if(msg.is_rtr || msg.len < 2) return;
    const auto target_id = msg.data[1];
    const auto command = (NMTCommand)msg.data[0];
    // Initialisation is not controlled by external NMT Server
    if(m_NodeState == NodeState::PreOperational || m_NodeState == NodeState::Operational || m_NodeState == NodeState::Stopped)
    {
        /* Check if this NMT-message is for this node */
        /* byte 1 = 0 : all the nodes are concerned (broadcast) */
        if(target_id == 0x00 || target_id == m_NodeId)
        {
            switch(command)
            {
                case NMTCommand::Start_Node:
                {
                    if(m_NodeState == NodeState::PreOperational || m_NodeState == NodeState::Stopped)
                    {
                        if(nmt_callback) nmt_callback(command);
                        SetNodeState(NodeState::Operational);
                    }
                    break;
                }
                case NMTCommand::Stop_Node:
                {
                    if(m_NodeState == NodeState::PreOperational || m_NodeState == NodeState::Operational)
                    {
                        if(nmt_callback) nmt_callback(command);
                        SetNodeState(NodeState::Stopped);
                    }
                    break;
                }
                case NMTCommand::Enter_PreOperational:
                {
                    if(m_NodeState == NodeState::Operational || m_NodeState == NodeState::Stopped)
                    {
                        if(nmt_callback) nmt_callback(command);
                        SetNodeState(NodeState::PreOperational);
                    }
                    break;
                }
                case NMTCommand::Reset_Node:
                {
                    if(nmt_callback) nmt_callback(command);
                    SetNodeState(NodeState::Initialisation);
                    break;
                }
                case NMTCommand::Reset_Comm:
                {
                    // Handle LSS node_id changes
                    auto current_id = GetNodeID();
                    if(current_id != lss_transfer.node_id)
                    {
                        current_id = lss_transfer.node_id;
                        DEBUG_PRINT("LSS changed node_id to %d, reinitializing...\n", lss_transfer.node_id);
                        SetNodeID(current_id);
                    }
                    if(nmt_callback) nmt_callback(command);
                    SetNodeState(NodeState::Initialisation);
                    break;
                }
                default: break;
            }
        }
    }
}

void MiniCANOpen::ProcessSYNCMessage(const DataType::Comm::CANMessage& msg)
{

}

void MiniCANOpen::ProcessEMCYMessage(const DataType::Comm::CANMessage& msg) const
{
    if(msg.len != 8 || !emcy_callback) return;
    uint8_t emcy_node_id = GetNodeIDFromCOB(msg.cob_id);
    uint16_t error_code = msg.data[0] | ((uint16_t)msg.data[1] << 8);
    uint8_t error_reg = msg.data[2];
    uint32_t mef = msg.data[3] | ((uint32_t)msg.data[4] << 8) | ((uint32_t)msg.data[5] << 16) | ((uint32_t)msg.data[6] << 24);
    emcy_callback(emcy_node_id, error_code, error_reg, mef);
}

void MiniCANOpen::ProcessPDOMessage(const DataType::Comm::CANMessage& msg)
{

}

/* SDO (un)packing macros */
#define getSDOcs(byte) ((byte) >> 5) // Returns the command specifier (cs, ccs, scs) from the first byte of the SDO
#define getSDOtoggle(byte) (((byte) >> 4) & 1)  // Returns the toggle from the first byte of the SDO
#define getSDOn2(byte) ((byte >> 2) & 3)   // Returns the number of bytes without data from the first byte of the SDO. Coded in 2 bits
#define getSDOn3(byte) (((byte) >> 1) & 7)  // Returns the number of bytes without data from the first byte of the SDO. Coded in 3 bits
#define getSDOe(byte) ((byte >> 1) & 1) // Returns the transfer type from the first byte of the SDO, Expedited or Normal
#define getSDOs(byte) (byte & 1)  // Returns the size indicator from the first byte of the SDO
#define getSDOc(byte) (byte & 1) // Returns the indicator of end transmission from the first byte of the SDO
#define getSDOindex(byte1, byte2) (((uint16_t)byte2 << 8) | ((uint16_t)byte1)) // Returns the index from the bytes 1 and 2 of the SDO
#define getSDOsubIndex(byte3) (byte3) // Returns the subIndex from the byte 3 of the SDO
#define getSDOblockSC(byte) (byte & 3) // Returns the subcommand in SDO block transfer

/* SDO block upload client subcommand */
#define SDO_BCS_INITIATE_UPLOAD_REQUEST 0
#define SDO_BCS_END_UPLOAD_REQUEST      1
#define SDO_BCS_UPLOAD_RESPONSE         2
#define SDO_BCS_START_UPLOAD            3

/* SDO block upload server subcommand */
#define SDO_BSS_INITIATE_UPLOAD_RESPONSE 0
#define SDO_BSS_END_UPLOAD_RESPONSE      1

/* SDO block download client subcommand */
#define SDO_BCS_INITIATE_DOWNLOAD_REQUEST 0
#define SDO_BCS_END_DOWNLOAD_REQUEST      1

/* SDO block download server subcommand */
#define SDO_BSS_INITIATE_DOWNLOAD_RESPONSE 0
#define SDO_BSS_END_DOWNLOAD_RESPONSE      1
#define SDO_BSS_DOWNLOAD_RESPONSE          2

#define SDO_BLOCK_SIZE 16

// Implemented SDO server first (0x1200)
/*
 * Expedited SDO Protocol: Byte[0]
 * 0x2F = Write one byte            (>> 5 = 1)
 * 0x2B = Write two bytes
 * 0x27 = Write three bytes
 * 0x23 = Write four bytes
 * 0x60 = Write successful response (>> 5 = 3)
 *
 * 0x40 = Read                      (>> 5 = 2)
 * 0x4F = Read one byte
 * 0x4B = Read two bytes
 * 0x47 = Read three bytes
 * 0x43 = Read four bytes
 *
 * 0x80 = Error response            (>> 5 = 4)
 *
 * Normal SDO Protocol: Byte[0]
 * 1) Download Protocol (Client -> Server, Server stores SDO to OD):
 *    i) Initial C->S CS = 0x21 (>> 5 = 1), S->C CS = 0x60 (>> 5 = 3)
 *    ii) Response: 0x00 / 0x10 / 0x20 / 0x30, >> 5 = 0 / 0 / 1 / 1
 *    iii) If last segment's Byte[0] = 0x10: (can be extracted by getSDOtoggle)
 *       a) 0x0F = Write one byte      (>> 5 = 0)
 *       b) 0x0D = Write two bytes
 *       c) 0x0B = Write three bytes
 *       d) 0x09 = Write four bytes
 *       e) 0x07 = Write five bytes
 *       f) 0x05 = Write six bytes
 *       g) 0x03 = Write seven bytes
 *    iv) If last segment's Byte[0] = 0x00:
 *       a) 0x1F = Write one byte      (>> 5 = 0)
 *       b) 0x1D = Write two bytes
 *       c) 0x1B = Write three bytes
 *       d) 0x19 = Write four bytes
 *       e) 0x17 = Write five bytes
 *       f) 0x15 = Write six bytes
 *       g) 0x13 = Write seven bytes
 * 2) Upload Protocol (Server -> Client, Server uploads OD to SDO):
 *    i) Initial C->S CS = 0x40 (>> 5 = 2), S->C CS = 0x41 (>> 5 = 2)
 *    ii) Response: 0x00 / 0x10 / 0x60 / 0x70, >> 5 = 0 / 0 / 3 / 3
 *    iii) & iv) Same as Download Protocol
 */
void MiniCANOpen::ProcessSDOMessage(const DataType::Comm::CANMessage& msg)
{
    // First, determine the role
    SDORole who_am_i = SDORole::SDO_UNKNOWN;
    // Starting from Server SDO Parameter, 0x1200
    uint8_t sdo_number = 0;
    uint8_t actual_size = 0;
    uint32_t readout_cob_id = 0;
    for(uint16_t start = 0x1200; start < 0x12FF; start++)
    {
        if(ReadODValue(start, 0x01, actual_size, &readout_cob_id) == FuncRetCode::OK && actual_size >= 4)
        {
            if(msg.cob_id == (uint16_t)readout_cob_id)
            {
                who_am_i = SDORole::SDO_SERVER;
                sdo_number = (uint8_t)(start - 0x1200);
                break;
            }
        }
    }
    if(who_am_i == SDORole::SDO_UNKNOWN) // still in UNKNOWN state, indicating that we are a client.
    {
        // TODO: Handle SDO Client.
        return;
    }
    if(msg.len != 8) // Message size incorrect, only respond after cob_id correct
    {
        SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_GENERAL_ERROR);
        return;
    }

    /* Look for an SDO transfer already initiated. */
    auto on_use_line = GetSDOLineOnUse(sdo_number, who_am_i);
    /* Let's find cs value, first it is set as "not valid" */
    uint8_t cs = 0xFF;
    /* Special cases for block transfer: in frames with segment data cs is not specified, and transfer line is already initiated. */
    if(on_use_line)
    {
        if((who_am_i == SDORole::SDO_SERVER && on_use_line.value()->state == SDOState::STATE_BLOCK_DOWNLOAD_IN_PROGRESS) ||
            (who_am_i == SDORole::SDO_CLIENT && on_use_line.value()->state == SDOState::STATE_BLOCK_UPLOAD_IN_PROGRESS))
        {
            cs = (msg.data[0] == 0x80) ? 4 : 6;
        }
    }
    /* Other cases: cs is specified in data packet. */
    if(cs == 0xFF) cs = getSDOcs(msg.data[0]);

    /* Testing the command specifier */
    /* Allowed : cs = 0, 1, 2, 3, 4, 5, 6 */
    /* cs = other : Not allowed -> abort. */
    switch(cs)
    {
        case 0:
        {
            if(who_am_i == SDORole::SDO_SERVER)
            {
                if(!on_use_line || on_use_line.value()->state != SDOState::STATE_DOWNLOAD_IN_PROGRESS)
                {
                    /* Receiving a download segment data : an SDO transfer should have been yet initiated. */
                    DEBUG_PRINT("SDO error: Received download segment for unstarted trans: %d\n", sdo_number);
                    SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                ResetSDOTimer(*on_use_line.value()); // Reset watchdog
                DEBUG_PRINT("Received SDO DL segment at %d\n", sdo_number);
                auto index = on_use_line.value()->target_index;
                auto subindex = on_use_line.value()->target_subindex;
                /* Toggle test. */
                if(on_use_line.value()->toggle != getSDOtoggle(msg.data[0]))
                {
                    DEBUG_PRINT("SDO error: toggle error %d\n", getSDOtoggle(msg.data[0]));
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_TOGGLE_NOT_ALTERNED);
                    return;
                }
                /* Nb of data to be downloaded */
                uint8_t nb_bytes = 7 - getSDOn3(msg.data[0]);
                /* Store the data in the transfer structure. */
                if(on_use_line.value()->push_back(&msg.data[1], nb_bytes) != FuncRetCode::OK)
                {
                    DEBUG_PRINT("SDO error: cs=0 buffer full\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                    return;
                }
                /* Sending the SDO response, CS = 1 */
                uint8_t data[8]{};
                data[0] = (uint8_t)((1 << 5) | (on_use_line.value()->toggle << 4));
                SendSDO(sdo_number, who_am_i, data);
                /* Inverting the toggle for the next segment. */
                on_use_line.value()->toggle = !on_use_line.value()->toggle;
                /* If it was the last segment, */
                if(getSDOc(msg.data[0]))
                {
                    /* Transfering line data to object dictionary. */
                    if(ParseSDOLineToOD(*on_use_line.value()) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO error: cs=0 unable to copy data to od\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    ResetSDOLine(*on_use_line.value());
                    DEBUG_PRINT("SDO end of download @ index %d\n", sdo_number);
                }
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 1:
        {
            if(who_am_i == SDORole::SDO_SERVER)
            {
                auto index = (uint16_t)getSDOindex(msg.data[1], msg.data[2]);
                auto subindex = getSDOsubIndex(msg.data[3]);
                DEBUG_PRINT("SDO cs 1 initiate dl @ 0x1200 + %d, write:%x:%x\n", sdo_number, index, subindex);
                if(on_use_line) // If there's a SDO transfer which is already initiated
                {
                    DEBUG_PRINT("SDO cs 1 error: transfer yet started\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                // No line on use, try to open a new line
                auto free_line = GetSDOLineFree(who_am_i);
                if(!free_line)
                {
                    DEBUG_PRINT("SDO cs 1 error: can't get free line\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                InitSDOLine(*free_line.value(), sdo_number, index, subindex, SDOState::STATE_DOWNLOAD_IN_PROGRESS);
                if(getSDOe(msg.data[0])) // if sdo expedited (fast sdo protocol)
                {
                    uint8_t nbBytes = 4 - getSDOn2(msg.data[0]);
                    if(free_line.value()->push_back(&msg.data[4], nbBytes) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO cs 1 error: can't push_back %d element\n", nbBytes);
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    /* SDO expedited -> transfer finished. Data can be stored in the dictionary. */
                    /* The line will be reseted when it is downloading in the dictionary. */
                    DEBUG_PRINT("SDO expedited transfer finished, len=%d\n", nbBytes);
                    if(ParseSDOLineToOD(*free_line.value()) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO cs 1 error: can't copy data to od\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    ResetSDOLine(*free_line.value());
                }
                else // Normal SDO protocol
                {
                    if(getSDOs(msg.data[0]))
                    {
                        uint32_t nbBytes = (msg.data[4]) + ((uint32_t)(msg.data[5])<<8) + ((uint32_t)(msg.data[6])<<16) + ((uint32_t)(msg.data[7])<<24);
                        free_line.value()->expected_count = nbBytes;
                        DEBUG_PRINT("SDO cs 1: normal protocol nbBytes:%d\n", nbBytes);
                    }
                }
                // Generating a response, cs = 3
                uint8_t data[8]{};
                data[0] = 3 << 5;
                data[1] = (uint8_t)(index & 0xFF);
                data[2] = (uint8_t)((index >> 8) & 0xFF);
                data[3] = subindex;
                SendSDO(sdo_number, who_am_i, data);
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 2:
        {
            if(who_am_i == SDORole::SDO_SERVER)
            {
                auto index = (uint16_t)getSDOindex(msg.data[1], msg.data[2]);
                auto subindex = getSDOsubIndex(msg.data[3]);
                DEBUG_PRINT("SDO cs 2 initiate ul @ 0x1200 + %d, write:%x:%x\n", sdo_number, index, subindex);
                if(on_use_line) // If there's a SDO transfer which is already initiated
                {
                    DEBUG_PRINT("SDO cs 2 error: transfer yet started\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                // No line on use, try to open a new line
                auto free_line = GetSDOLineFree(who_am_i);
                if(!free_line)
                {
                    DEBUG_PRINT("SDO cs 2 error: can't get free line\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                InitSDOLine(*free_line.value(), sdo_number, index, subindex, SDOState::STATE_UPLOAD_IN_PROGRESS);
                /* Transfer data from dictionary to the line structure. */
                if(ParseODToSDOLine(*free_line.value()) != FuncRetCode::OK)
                {
                    DEBUG_PRINT("SDO cs 2 error: can't copy data to od\n");
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                /* Preparing the response.*/
                auto nbBytes = free_line.value()->data.size();
                uint8_t data[8]{};
                if(nbBytes > 4)
                {
                    // Normal transfer (Segmented)
                    /* code to send the initiate upload response. (cs = 2) */
                    data[0] = (uint8_t)((2 << 5) | 1);
                    data[1] = (uint8_t)(index & 0xFF);        /* LSB */
                    data[2] = (uint8_t)((index >> 8) & 0xFF); /* MSB */
                    data[3] = subindex;
                    data[4] = (uint8_t) nbBytes;
                    data[5] = (uint8_t) (nbBytes >> 8);
                    data[6] = (uint8_t) (nbBytes >> 16);
                    data[7] = (uint8_t) (nbBytes >> 24);
                    DEBUG_PRINT("SDO cs 2: sending normal ul init response @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                }
                else
                {
                    /* Expedited upload. (cs = 2 ; e = 1) */
                    data[0] = (uint8_t)((2 << 5) | ((4 - nbBytes) << 2) | 3);
                    data[1] = (uint8_t)(index & 0xFF);        /* LSB */
                    data[2] = (uint8_t)((index >> 8) & 0xFF); /* MSB */
                    data[3] = subindex;
                    if(free_line.value()->extract_from_head(data + 4, nbBytes) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO cs 2 error: can't copy expedit data from line to buf\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    DEBUG_PRINT("SDO cs 2: send expedit ul init response @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                    ResetSDOLine(*free_line.value());
                }
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 3:
        {
            if(who_am_i == SDORole::SDO_SERVER)
            {
                /* Receiving a upload segment. */
                /* A SDO transfer should have been yet initiated. */
                if(!on_use_line || on_use_line.value()->state != SDOState::STATE_UPLOAD_IN_PROGRESS)
                {
                    DEBUG_PRINT("SDO cs 3 error: received upload segment for unstarted trans: %d\n", sdo_number);
                    SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                    return;
                }
                ResetSDOTimer(*on_use_line.value()); // Reset the watchdog
                auto index = on_use_line.value()->target_index;
                auto subindex = on_use_line.value()->target_subindex;
                if(on_use_line.value()->toggle != getSDOtoggle(msg.data[0]))
                {
                    DEBUG_PRINT("SDO cs 3 error: toggle error: %d\n", getSDOtoggle(msg.data[0]));
                    SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_TOGGLE_NOT_ALTERNED);
                    return;
                }
                /* Uploading next segment. We need to know if it will be the last one. */
                auto rest_bytes = on_use_line.value()->data.size();
                uint8_t data[8]{};
                if(rest_bytes > 7)
                {
                    /* The segment to transfer is not the last one.*/
                    /* code to send the next segment. (cs = 0; c = 0) */
                    data[0] = (uint8_t)(on_use_line.value()->toggle << 4);
                    if(on_use_line.value()->extract_from_head(data + 1, 7) != FuncRetCode::OK)
                    {
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    on_use_line.value()->toggle = !on_use_line.value()->toggle;
                    DEBUG_PRINT("SDO cs 3: Sending ul segment @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                }
                else
                {
                    /* Last segment. */
                    /* code to send the last segment. (cs = 0; c = 1) */
                    data[0] = (uint8_t)((on_use_line.value()->toggle << 4) | ((7 - rest_bytes) << 1) | 1);
                    if(on_use_line.value()->extract_from_head(data + 1, rest_bytes) != FuncRetCode::OK)
                    {
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    DEBUG_PRINT("SDO cs 3: Sending last ul segment @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                    ResetSDOLine(*on_use_line.value());
                }
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 4:
        {
            /* Received SDO abort. */
            uint32_t abort_code = (uint32_t)msg.data[4] | ((uint32_t)msg.data[5] << 8) | ((uint32_t)msg.data[6] << 16) | ((uint32_t)msg.data[7] << 24);
            if(who_am_i == SDORole::SDO_SERVER)
            {
                if(on_use_line)
                {
                    ResetSDOLine(*on_use_line.value());
                    DEBUG_PRINT("SDO: recv sdo abort, release line #%d, code:%d\n", sdo_number, abort_code);
                }
                else DEBUG_PRINT("SDO: recv sdo abort, no active line\n");
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 5:
        {
            /* Command specifier for data transmission - the client or server is the data producer */
            uint8_t sub_command = getSDOblockSC(msg.data[0]);
            uint8_t data[8]{};
            if(who_am_i == SDORole::SDO_SERVER) /* Server block upload */
            {
                if(sub_command == SDO_BCS_INITIATE_UPLOAD_REQUEST)
                {
                    uint16_t index = (uint16_t)getSDOindex(msg.data[1], msg.data[2]);
                    uint8_t subindex = getSDOsubIndex(msg.data[3]);
                    DEBUG_PRINT("Recv SDO init block upload @ %d, idx:%x:%x\n", sdo_number, index, subindex);
                    if(on_use_line)
                    {
                        DEBUG_PRINT("SDO cs 5 error: transfer yet started %d\n", sdo_number);
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    /* Check block size */
                    if(msg.data[4] > 127)
                    {
                        DEBUG_PRINT("SDO cs 5 error: invalid block size %d\n", msg.data[4]);
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_INVALID_BLOCK_SIZE);
                        return;
                    }
                    /* Try to open a new line.*/
                    auto free_line = GetSDOLineFree(who_am_i);
                    if(!free_line)
                    {
                        DEBUG_PRINT("SDO cs 5 error: can't get free line\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    InitSDOLine(*free_line.value(), sdo_number, index, subindex, SDOState::STATE_BLOCK_UPLOAD_IN_PROGRESS);
                    free_line.value()->peer_crc_support = (uint8_t)((msg.data[0] >> 2) & 1);
                    free_line.value()->block_size = msg.data[4];
                    /* Transfer data from dictionary to the line structure. */
                    if(ParseODToSDOLine(*free_line.value()) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO cs 5 error: can't copy data to od\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    auto nbBytes = free_line.value()->data.size();
                    free_line.value()->obj_size = nbBytes;
                    data[0] = (6 << 5) | (1 << 1) | SDO_BSS_INITIATE_UPLOAD_RESPONSE;
                    data[1] = (uint8_t)(index & 0xFF);
                    data[2] = (uint8_t)((index >> 8) & 0xFF);
                    data[3] = subindex;
                    data[4] = (uint8_t)nbBytes;
                    data[5] = (uint8_t)(nbBytes >> 8);
                    data[6] = (uint8_t)(nbBytes >> 16);
                    data[7] = (uint8_t)(nbBytes >> 24);
                    DEBUG_PRINT("SDO cs 5, send normal block ul init response @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                }
                else if(sub_command == SDO_BCS_END_UPLOAD_REQUEST)
                {
                    DEBUG_PRINT("Recv SDO end block upload @ %d\n", sdo_number);
                    if(!on_use_line || on_use_line.value()->state != SDOState::STATE_BLOCK_UPLOAD_IN_PROGRESS)
                    {
                        DEBUG_PRINT("SDO cs 5 error: received block upload request for unstarted trans: %d\n", sdo_number);
                        SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    ResetSDOLine(*on_use_line.value());
                }
                else if(sub_command == SDO_BCS_UPLOAD_RESPONSE || sub_command == SDO_BCS_START_UPLOAD)
                {
                    if(!on_use_line || on_use_line.value()->state != SDOState::STATE_BLOCK_UPLOAD_IN_PROGRESS)
                    {
                        DEBUG_PRINT("SDO cs 5 error: received block upload response for unstarted trans: %d\n", sdo_number);
                        SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    ResetSDOTimer(*on_use_line.value()); // Reset the watchdog
                    uint16_t index = on_use_line.value()->target_index;
                    uint8_t subindex = on_use_line.value()->target_subindex;
                    if(sub_command == SDO_BCS_UPLOAD_RESPONSE)
                    {
                        DEBUG_PRINT("SDO cs 5: received block upload response %d\n", sdo_number);
                        on_use_line.value()->block_size = msg.data[2];
                        uint8_t ack_seq = (msg.data[1]) & 0x7F;
                        size_t nbBytes = on_use_line.value()->data.size();
                        /* If everything has been sent and acknowledged, we send a block end upload */
                        if(nbBytes == 0 && ack_seq == on_use_line.value()->sequence_number)
                        {
                            data[0] = (uint8_t)((6 << 5) | (on_use_line.value()->end_field << 2) | SDO_BSS_END_UPLOAD_RESPONSE);
                            DEBUG_PRINT("SDO cs 5: sending block END upload response @ &d\n", sdo_number);
                            SendSDO(sdo_number, who_am_i, data);
                            return;
                        }
                    }
                    else DEBUG_PRINT("SDO cs 5: recv block START upload @ %d\n", sdo_number);
                    for(uint8_t seq_no = 1; seq_no <= on_use_line.value()->block_size; seq_no++)
                    {
                        on_use_line.value()->sequence_number = seq_no;
                        size_t nbBytes = on_use_line.value()->data.size();
                        if(nbBytes > 7) /* The segment to transfer is not the last one.*/
                        {
                            data[0] = seq_no;
                            if(on_use_line.value()->extract_from_head(data + 1, 7) != FuncRetCode::OK)
                            {
                                SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                                return;
                            }
                            DEBUG_PRINT("SDO cs 5: sending upload segment @ %d\n", sdo_number);
                            SendSDO(sdo_number, who_am_i, data);
                        }
                        else /* Last segment is in this block */
                        {
                            data[0] = 0x80 | seq_no;
                            if(on_use_line.value()->extract_from_head(data + 1, nbBytes) != FuncRetCode::OK)
                            {
                                SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                                return;
                            }
                            DEBUG_PRINT("SDO cs 5: sending last upload segment @ %d\n", sdo_number);
                            SendSDO(sdo_number, who_am_i, data);
                            on_use_line.value()->end_field = (uint8_t)(7 - nbBytes);
                            break;
                        }
                    }
                }
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        case 6:
        {
            /* Command specifier for data reception - the client or server is the data consumer */
            uint8_t data[8]{};
            if(who_am_i == SDORole::SDO_SERVER)
            {
                if(!on_use_line)
                {
                    /* Nothing already started */
                    uint8_t sub_command = msg.data[0] & 1;
                    if(sub_command != SDO_BCS_INITIATE_DOWNLOAD_REQUEST)
                    {
                        DEBUG_PRINT("SDO cs 6 error: rcv wrong subcommand from node id", GetNodeIDFromCOB(msg.cob_id));
                        SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    uint16_t index = (uint16_t)getSDOindex(msg.data[1], msg.data[2]);
                    uint8_t subindex = getSDOsubIndex(msg.data[3]);
                    DEBUG_PRINT("Recv SDO init block download @ %d, idx:%x:%x\n", sdo_number, index, subindex);
                    auto free_line = GetSDOLineFree(who_am_i);
                    if(!free_line)
                    {
                        DEBUG_PRINT("SDO cs 6 error: can't get free line\n");
                        SendFailedSDO(sdo_number, who_am_i, index, subindex, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                        return;
                    }
                    InitSDOLine(*free_line.value(), sdo_number, index, subindex, SDOState::STATE_BLOCK_DOWNLOAD_IN_PROGRESS);
                    free_line.value()->rx_step = SDORXStep::RX_STEP_STARTED;
                    free_line.value()->peer_crc_support = (uint8_t)((msg.data[0] >> 2) & 1);
                    if(msg.data[0] & 2) /* if data set size is indicated */
                    {
                        free_line.value()->obj_size = (uint32_t)msg.data[4] + (uint32_t)msg.data[5] * 256 + (uint32_t)msg.data[6] * 256 * 256 + (uint32_t)msg.data[7] * 256 * 256 * 256;
                    }
                    data[0] = (5 << 5) | SDO_BSS_INITIATE_DOWNLOAD_RESPONSE;
                    data[1] = (uint8_t)(index & 0xFF);
                    data[2] = (uint8_t)((index >> 8) & 0xFF);
                    data[3] = subindex;
                    data[4] = SDO_BLOCK_SIZE;
                    DEBUG_PRINT("SDO cs 6: sending block dl init response @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                }
                else if(on_use_line.value()->rx_step == SDORXStep::RX_STEP_STARTED)
                {
                    DEBUG_PRINT("SDO cs 6: rcv block download segment @ %d\n", sdo_number);
                    ResetSDOTimer(*on_use_line.value());
                    uint8_t seq_no = msg.data[0] & 0x7F;
                    if(msg.data[0] & 0x80) // last segment?
                    {
                        if(seq_no == on_use_line.value()->sequence_number + 1)
                        {
                            on_use_line.value()->rx_step = SDORXStep::RX_STEP_END;
                            on_use_line.value()->sequence_number = seq_no;
                            /* Store the data temporary because we don't know yet how many bytes do not contain data */
                            memcpy(on_use_line.value()->temp_data, msg.data, 8);
                        }
                        data[0] = (5 << 5) | SDO_BSS_DOWNLOAD_RESPONSE;
                        data[1] = on_use_line.value()->sequence_number;
                        data[2] = SDO_BLOCK_SIZE;
                        DEBUG_PRINT("SDO cs 6: sending block download response @ %d\n", sdo_number);
                        SendSDO(sdo_number, who_am_i, data);
                        on_use_line.value()->sequence_number = 0;
                    }
                    else
                    {
                        if(seq_no == on_use_line.value()->sequence_number + 1)
                        {
                            on_use_line.value()->sequence_number = seq_no;
                            /* Store the data in the transfer structure. */
                            if(on_use_line.value()->extract_from_head(data + 1, 7) != FuncRetCode::OK)
                            {
                                SendFailedSDO(sdo_number, who_am_i, on_use_line.value()->target_index, on_use_line.value()->target_subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                                return;
                            }
                        }
                        if(seq_no == SDO_BLOCK_SIZE)
                        {
                            data[0] = (5 << 5) | SDO_BSS_DOWNLOAD_RESPONSE;
                            data[1] = on_use_line.value()->sequence_number;
                            data[2] = SDO_BLOCK_SIZE;
                            DEBUG_PRINT("SDO cs 6: sending block download response @ %d\n", sdo_number);
                            SendSDO(sdo_number, who_am_i, data);
                            on_use_line.value()->sequence_number = 0;
                        }
                    }
                }
                else if(on_use_line.value()->rx_step == SDORXStep::RX_STEP_END)
                {
                    DEBUG_PRINT("SDO cs 6: rcv SDO block dl END request @ %d\n", sdo_number);
                    if((msg.data[0] & 1) != SDO_BCS_END_DOWNLOAD_REQUEST)
                    {
                        DEBUG_PRINT("SDO cs 6: error block dl, recv wrong subcommand @ %d\n", sdo_number);
                        SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    ResetSDOTimer(*on_use_line.value());
                    uint8_t nbBytesNoData = (uint8_t)((msg.data[0] >> 2) & 0x07);
                    if(on_use_line.value()->extract_from_head(on_use_line.value()->temp_data + 1, 7 - nbBytesNoData) != FuncRetCode::OK)
                    {
                        SendFailedSDO(sdo_number, who_am_i, on_use_line.value()->target_index, on_use_line.value()->target_subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    if(on_use_line.value()->obj_size) /* If size was indicated in the initiate request */
                    {
                        if(on_use_line.value()->obj_size != on_use_line.value()->data.size())
                        {
                            DEBUG_PRINT("SDO cs 6: error block dl, sizes not match\n");
                            SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_LOCAL_CTRL_ERROR);
                            return;
                        }
                    }
                    data[0] = (5 << 5) | SDO_BSS_END_DOWNLOAD_RESPONSE;
                    DEBUG_PRINT("SDO cs 6: sending blk dl END response @ %d\n", sdo_number);
                    SendSDO(sdo_number, who_am_i, data);
                    /* Transfering line data to object dictionary. */
                    if(ParseSDOLineToOD(*on_use_line.value()) != FuncRetCode::OK)
                    {
                        DEBUG_PRINT("SDO cs 6: unable to copy data to OD\n");
                        SendFailedSDO(sdo_number, who_am_i, on_use_line.value()->target_index, on_use_line.value()->target_subindex, SDOAbortCode::SDOABT_GENERAL_ERROR);
                        return;
                    }
                    ResetSDOLine(*on_use_line.value());
                    DEBUG_PRINT("SDO cs 6: End of block @ %d\n", sdo_number);
                }
            }
            else
            {
                // TODO: Handle SDO Client
            }
            break;
        }
        default:
        {
            if(on_use_line) SendFailedSDO(sdo_number, who_am_i, on_use_line.value()->target_index, on_use_line.value()->target_subindex, SDOAbortCode::SDOABT_CS_NOT_VALID);
            else SendFailedSDO(sdo_number, who_am_i, 0, 0, SDOAbortCode::SDOABT_CS_NOT_VALID);
            DEBUG_PRINT("SDO: recv unknown cs: %d\n", cs);
            break;
        }
    }
}

// https://blog.csdn.net/qq_38156743/article/details/143882903
/*
 * Typical CANopen master LSS sequence:
 * 1) By sending correct Vendor ID and Product ID (predefined), with REV_LOW / HIGH, SERIAL_LOW / HIGH,
 *    we can get all devices' serial number and revision number (in most scenes, returned results are not in pairs with each other)
 */
void MiniCANOpen::ProcessLSSMessage(const DataType::Comm::CANMessage& msg)
{
#define GetLSSIdent(m) (((uint32_t)m.data[4] << 24) | ((uint32_t)m.data[3] << 16) | ((uint32_t)m.data[2] << 8) | ((uint32_t)m.data[1]))
    if(msg.len != 8)
    {
        DEBUG_PRINT("income LSS len != 8\n");
        return;
    }
    const auto msg_cs = msg.data[0];
    DEBUG_PRINT("Process Slave LSS: %d\n", msg_cs);
    switch(msg_cs)
    {
        case LSSServices::LSS_SM_GLOBAL:
        {
            if(msg.data[1] == lss_transfer.mode)
            {
                DEBUG_PRINT("Slave LSS already in mode: %d\n", msg.data[1]);
                break;
            }
            if(msg.data[1] == LSSMode::LSS_CONFIGURATION_MODE)
            {
                DEBUG_PRINT("Slave LSS switch to configuration mode\n");
                lss_transfer.mode = LSSMode::LSS_CONFIGURATION_MODE;
            }
            else if(msg.data[1] == LSSMode::LSS_WAITING_MODE)
            {
                DEBUG_PRINT("Slave LSS switch to operational mode\n");
                /* If the nodeID has changed update it and put the node state to Initialisation. */
                const auto current_id = GetNodeID();
                if(lss_transfer.node_id != current_id)
                {
                    if(current_id == 0xFF)
                    {
                        DEBUG_PRINT("LSS changed node_id to %d, reinitializing...\n", lss_transfer.node_id);
                        SetNodeID(lss_transfer.node_id);
                        SetNodeState(NodeState::Initialisation);
                    }
                    // else: The nodeID will be changed on NMT_Reset_Communication Request, see ProcessNMTMessage()
                }
                lss_transfer.mode = LSSMode::LSS_WAITING_MODE;
            }
            break;
        }
        case LSSServices::LSS_CONF_NODE_ID:
        {
            uint8_t error_code = 0, spec_error = 0;
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                if(msg.data[1] > 127 && msg.data[1] != 0xFF)
                {
                    DEBUG_PRINT("LSS_CONF_NODE_ID: node_id %d out of range\n", msg.data[1]);
                    error_code = 1;
                }
                else lss_transfer.node_id = msg.data[1];
            }
            else
            {
                DEBUG_PRINT("LSS_CONF_NODE_ID: not in config mode\n");
                break;
            }
            SendSlaveLSSMessage((LSSServices)msg_cs, &error_code, &spec_error);
            break;
        }
        case LSSServices::LSS_CONF_BIT_TIMING:
        {
            uint8_t error_code = 0, spec_error = 0;
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                switch(msg.data[2])
                {
                    case 0x00: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_1_MBPS; break;
                    case 0x01: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_800_KBPS; break;
                    case 0x02: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_500_KBPS; break;
                    case 0x03: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_250_KBPS; break;
                    case 0x04: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_125_KBPS; break;
                    case 0x05: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_100_KBPS; break;
                    case 0x06: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_50_KBPS; break;
                    case 0x07: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_20_KBPS; break;
                    case 0x08: lss_transfer.target_baudrate = DataType::Comm::CANBaudrate::BAUD_10_KBPS; break;
                    default:
                    {
                        DEBUG_PRINT("LSS_CONF_BIT_TIMING: unsupported baudrate: %d\n", msg.data[2]);
                        error_code = 0xFF;
                        break;
                    }
                }
            }
            else
            {
                DEBUG_PRINT("LSS_CONF_BIT_TIMING: not in config mode\n");
                break;
            }
            SendSlaveLSSMessage((LSSServices)msg_cs, &error_code, &spec_error);
            break;
        }
        case LSSServices::LSS_CONF_ACT_BIT_TIMING:
        {
            // TODO: switch bitrate
            break;
        }
        case LSSServices::LSS_CONF_STORE:
        {
            uint8_t error_code = 0, spec_error = 0;
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                if(lss_transfer.store_callback) lss_transfer.store_callback(error_code, spec_error);
                else
                {
                    DEBUG_PRINT("LSS_CONF_STORE: null store callback\n");
                    error_code = 0x01; /* store configuration is not supported */
                }
            }
            else
            {
                DEBUG_PRINT("LSS_CONF_STORE: not in config mode\n");
            }
            SendSlaveLSSMessage((LSSServices)msg_cs, &error_code, &spec_error);
            break;
        }
        case LSSServices::LSS_SM_SELECTIVE_VENDOR:
        case LSSServices::LSS_SM_SELECTIVE_PRODUCT:
        case LSSServices::LSS_SM_SELECTIVE_REVISION:
        case LSSServices::LSS_SM_SELECTIVE_SERIAL:
        {
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                DEBUG_PRINT("LSS_SM_SELECTIVE_XX: not in waiting mode\n");
                break;
            }
            // extract the received node info
            uint32_t received_node_info = GetLSSIdent(msg);
            uint32_t stored_node_info = 0;
            uint8_t actual_size = 0;
            uint8_t _1018_subindex = msg_cs - LSSServices::LSS_SM_SELECTIVE_VENDOR + 1; // 0x00: Number of Entries
            if(ReadODValue(0x1018, _1018_subindex, actual_size, &stored_node_info) != FuncRetCode::OK || actual_size != 4)
            {
                DEBUG_PRINT("LSS_SM_SELECTIVE_XX: read 0x1018:%d failed\n", _1018_subindex);
                break;
            }
            if(stored_node_info == received_node_info)
            {
                lss_transfer.addr_sel_match |= (0x01 << (msg_cs - LSSServices::LSS_SM_SELECTIVE_VENDOR));
                if(lss_transfer.addr_sel_match == 0x0F) // all fields have been set
                {
                    DEBUG_PRINT("LSS_SM_SELECTIVE_XX: all identify fields matched\n");
                    lss_transfer.addr_sel_match = 0;
                    lss_transfer.node_id = GetNodeID();
                    lss_transfer.mode = LSSMode::LSS_CONFIGURATION_MODE;
                    SendSlaveLSSMessage(LSSServices::LSS_SM_SELECTIVE_RESP, nullptr, nullptr);
                }
            }
            else
            {
                DEBUG_PRINT("LSS_SM_SELECTIVE_XX: identity field not match: %d:%d\n", msg_cs, received_node_info);
                lss_transfer.addr_sel_match = 0x00;
            }
            break;
        }
        case LSSServices::LSS_EXTRA_SM_SELECTIVE_SERIAL_ONLY:
        {
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE) break;
            uint32_t received_serial_number = GetLSSIdent(msg);
            uint32_t stored_serial_number = 0;
            uint8_t actual_size = 0;
            if(ReadODValue(0x1018, 0x04, actual_size, &stored_serial_number) != FuncRetCode::OK || actual_size != 4) break;
            if(stored_serial_number == received_serial_number)
            {
                DEBUG_PRINT("LSS_EXTRA_SM_SERIAL: serial matched\n");
                lss_transfer.addr_sel_match = 0;
                lss_transfer.node_id = GetNodeID();
                lss_transfer.mode = LSSMode::LSS_CONFIGURATION_MODE;
                SendSlaveLSSMessage(LSSServices::LSS_SM_SELECTIVE_RESP, nullptr, nullptr);
            }
            break;
        }
        case LSSServices::LSS_IDENT_REMOTE_VENDOR:
        case LSSServices::LSS_IDENT_REMOTE_PRODUCT:
        case LSSServices::LSS_IDENT_REMOTE_REV_LOW:
        case LSSServices::LSS_IDENT_REMOTE_REV_HIGH:
        case LSSServices::LSS_IDENT_REMOTE_SERIAL_LOW:
        case LSSServices::LSS_IDENT_REMOTE_SERIAL_HIGH:
        {
            uint32_t received_node_info = GetLSSIdent(msg);
            uint32_t stored_node_info = 0;
            uint8_t actual_size = 0;
            switch(msg_cs)
            {
                case LSSServices::LSS_IDENT_REMOTE_VENDOR:
                {
                    ReadODValue(0x1018, 0x01, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info == received_node_info) lss_transfer.addr_ident_match |= 0x01;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                case LSSServices::LSS_IDENT_REMOTE_PRODUCT:
                {
                    ReadODValue(0x1018, 0x02, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info == received_node_info) lss_transfer.addr_ident_match |= 0x02;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                case LSSServices::LSS_IDENT_REMOTE_REV_LOW:
                {
                    ReadODValue(0x1018, 0x03, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info >= received_node_info) lss_transfer.addr_ident_match |= 0x04;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                case LSSServices::LSS_IDENT_REMOTE_REV_HIGH:
                {
                    ReadODValue(0x1018, 0x03, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info <= received_node_info) lss_transfer.addr_ident_match |= 0x08;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                case LSSServices::LSS_IDENT_REMOTE_SERIAL_LOW:
                {
                    ReadODValue(0x1018, 0x04, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info >= received_node_info) lss_transfer.addr_ident_match |= 0x10;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                case LSSServices::LSS_IDENT_REMOTE_SERIAL_HIGH:
                {
                    ReadODValue(0x1018, 0x04, actual_size, &stored_node_info);
                    if(actual_size == 4 && stored_node_info <= received_node_info) lss_transfer.addr_ident_match |= 0x20;
                    else lss_transfer.addr_ident_match = 0x00;
                    break;
                }
                default: break;
            }
            if(lss_transfer.addr_ident_match == 0x3F)
            {
                DEBUG_PRINT("LSS_IDENT_REMOTE_XX: all identify fields matched\n");
                lss_transfer.addr_ident_match = 0x00;
                if(lss_transfer.callback) lss_transfer.callback(LSSServices::LSS_IDENT_SLAVE);
                SendSlaveLSSMessage(LSSServices::LSS_IDENT_SLAVE, nullptr, nullptr);
            }
            else if(lss_transfer.addr_ident_match == 0x00) DEBUG_PRINT("LSS_IDENT_REMOTE_XX: identity field not match: %d:%d\n", msg_cs, received_node_info);
            break;
        }
        case LSSServices::LSS_IDENT_REMOTE_NON_CONF:
        {
            if(GetNodeID() == 0xFF)
            {
                DEBUG_PRINT("LSS_IDENT_REMOTE_NON_CONF: non-configured\n");
                if(lss_transfer.callback) lss_transfer.callback(LSSServices::LSS_IDENT_NON_CONF_SLAVE);
                SendSlaveLSSMessage(LSSServices::LSS_IDENT_NON_CONF_SLAVE, nullptr, nullptr);
            }
            else DEBUG_PRINT("LSS_IDENT_REMOTE_NON_CONF: already configured, %d\n", GetNodeID());
            break;
        }
        case LSSServices::LSS_EXTRA_IDENT_REMOTE_SERIAL_ONLY:
        {
            uint32_t received_serial_number = GetLSSIdent(msg);
            uint32_t stored_serial_number = 0;
            uint8_t actual_size = 0;
            if(ReadODValue(0x1018, 0x04, actual_size, &stored_serial_number) == FuncRetCode::OK && actual_size == 4)
            {
                if(stored_serial_number == received_serial_number)
                {
                    DEBUG_PRINT("LSS_IDENT_REMOTE_XX: all identify fields matched\n");
                    lss_transfer.addr_ident_match = 0x00;
                    if(lss_transfer.callback) lss_transfer.callback(LSSServices::LSS_IDENT_SLAVE);
                    SendSlaveLSSMessage(LSSServices::LSS_IDENT_SLAVE, nullptr, nullptr);
                }
            }
            break;
        }
        case LSSServices::LSS_INQ_VENDOR_ID:
        case LSSServices::LSS_INQ_PRODUCT_CODE:
        case LSSServices::LSS_INQ_REV_NUMBER:
        case LSSServices::LSS_INQ_SERIAL_NUMBER:
        {
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                uint32_t stored_node_info = 0;
                uint8_t actual_size = 0;
                uint8_t _1018_subindex = msg_cs - LSSServices::LSS_INQ_VENDOR_ID + 1; // 0x00: Number of Entries
                if(ReadODValue(0x1018, _1018_subindex, actual_size, &stored_node_info) == FuncRetCode::OK && actual_size == 4)
                {
                    SendSlaveLSSMessage((LSSServices)msg_cs, &stored_node_info, nullptr);
                }
            }
            break;
        }
        case LSSServices::LSS_INQ_NODE_ID:
        {
            if(lss_transfer.mode == LSSMode::LSS_CONFIGURATION_MODE)
            {
                uint8_t node_id = GetNodeID();
                SendSlaveLSSMessage((LSSServices)msg_cs, &node_id, nullptr);
            }
            break;
        }
        case LSSServices::LSS_EXTRA_INQ_SERIAL_ONLY:
        {
            uint32_t stored_serial_number = 0;
            uint8_t actual_size = 0;
            if(ReadODValue(0x1018, 0x04, actual_size, &stored_serial_number) == FuncRetCode::OK && actual_size == 4)
            {
                SendSlaveLSSMessage((LSSServices)msg_cs, &stored_serial_number, nullptr);
            }
            break;
        }
        default: break;
    }
}

#define StartOrStop(CommType, FuncStart, FuncStop) \
    if(new_state.CommType && curr_handle_state.CommType == 0)\
    {\
        curr_handle_state.CommType = 1;\
        FuncStart;\
    }\
    else if(!new_state.CommType && curr_handle_state.CommType == 1)\
    {\
        curr_handle_state.CommType = 0;\
        FuncStop;\
    }
#define None

void MiniCANOpen::SwitchCurrentHandleState(const HandleState& new_state)
{
    StartOrStop(handle_lss, None, None);
    StartOrStop(handle_sdo, None, ResetAllSDO());
    StartOrStop(handle_sync, None, None); // startSYNC, stopSYNC
    StartOrStop(handle_lifeguard, LifeguardInit(), LifeguardStop()); // lifeGuardInit, lifeGuardStop
    StartOrStop(handle_emergency, None, None);
    StartOrStop(handle_pdo, None, None); // PDOInit, PDOStop
    StartOrStop(handle_bootup, None, SendBootUp());
}

void MiniCANOpen::SendSlaveLSSMessage(LSSServices command, void* data1, void* data2) const
{
    if(!curr_handle_state.handle_lss)
    {
        DEBUG_PRINT("cannot send LSS message, not in proper handle state: %d\n", m_NodeState);
        return;
    }
    DataType::Comm::CANMessage msg
    {
        .cob_id = SLSS_ADDRESS,
        .is_rtr = false,
        .len = 8
    };
    msg.data[0] = command;
    switch(command)
    {
        case LSSServices::LSS_INQ_NODE_ID:
        {
            if(!data1) // double check
            {
                DEBUG_PRINT("send slave LSS message, LSS_INQ_NODE_ID, data1 is null!\n");
                return;
            }
            msg.data[1] = *(uint8_t*)data1;
            break;
        }
        case LSSServices::LSS_CONF_NODE_ID:
        case LSSServices::LSS_CONF_BIT_TIMING:
        case LSSServices::LSS_CONF_STORE:
        {
            if(!data1 || !data2)
            {
                DEBUG_PRINT("send slave LSS message, LSS_CONF_xx, data1/2 is null!\n");
                return;
            }
            msg.data[1] = *(uint8_t*)data1;
            msg.data[2] = *(uint8_t*)data2;
            break;
        }
        case LSSServices::LSS_INQ_VENDOR_ID:
        case LSSServices::LSS_INQ_PRODUCT_CODE:
        case LSSServices::LSS_INQ_REV_NUMBER:
        case LSSServices::LSS_INQ_SERIAL_NUMBER:
        case LSSServices::LSS_EXTRA_INQ_SERIAL_ONLY:
        {
            if(!data1)
            {
                DEBUG_PRINT("send slave LSS message, LSS_INQ_xx, data1 is null!\n");
                return;
            }
            msg.data[1]=(uint8_t)(*(uint32_t*)data1 & 0xFF);
            msg.data[2]=(uint8_t)(*(uint32_t*)data1 >> 8 & 0xFF);
            msg.data[3]=(uint8_t)(*(uint32_t*)data1 >> 16 & 0xFF);
            msg.data[4]=(uint8_t)(*(uint32_t*)data1 >> 24 & 0xFF);
            break;
        }
        case LSSServices::LSS_SM_SELECTIVE_RESP:
        case LSSServices::LSS_IDENT_SLAVE:
        case LSSServices::LSS_IDENT_NON_CONF_SLAVE:
            break;
        default: return;
    }
    can_send(msg);
}

void MiniCANOpen::SendBootUp() const
{
    if(m_NodeId == 0xFF) return;
    DataType::Comm::CANMessage msg
    {
        .cob_id = (uint16_t)((uint16_t)DS301FuncCode::NODE_GUARD << 7 | m_NodeId),
        .is_rtr = false,
        .len = 1,
    };
    msg.data[0] = 0x00;
    can_send(msg);
}

void MiniCANOpen::ResetAllSDO()
{
    for(auto& transfer : sdo_transfers) ResetSDOLine(transfer);
}

void MiniCANOpen::SendSDO(uint8_t sdo_number, SDORole who_am_i, uint8_t* data)
{
    if(m_NodeState != NodeState::Operational && m_NodeState != NodeState::PreOperational)
    {
        return;
    }
    uint32_t stored_cob_id = 0;
    uint8_t actual_size = 0;
    if(who_am_i == SDORole::SDO_SERVER)
    {
        if(ReadODValue(0x1200 + sdo_number, 0x02, actual_size, &stored_cob_id) != FuncRetCode::OK || actual_size != 4)
        {
            return;
        }
    }
    else if(who_am_i == SDORole::SDO_CLIENT)
    {
        if(ReadODValue(0x1200 + sdo_number, 0x01, actual_size, &stored_cob_id) != FuncRetCode::OK || actual_size != 4)
        {
            return;
        }
    }
    else return; // SDORole::SDO_UNKNOWN
    DataType::Comm::CANMessage msg
    {
        .cob_id = (uint16_t)stored_cob_id,
        .is_rtr = false,
        .len = 8
    };
    memcpy(msg.data, data, 8);
    can_send(msg);
}

void MiniCANOpen::SendAbortSDO(uint8_t sdo_number, SDORole who_am_i, uint16_t target_index, uint8_t target_subindex,
                               SDOAbortCode abort_code)
{
    uint8_t data[8]{};
    data[0] = 0x80;
    // Index
    data[1] = (uint8_t)(target_index & 0xFF); // LSB
    data[2] = (uint8_t)((target_index >> 8) & 0xFF); // MSB
    // SubIndex
    data[3] = target_subindex;
    // Data
    data[4] = (uint8_t)(abort_code & 0xFF);
    data[5] = (uint8_t)((abort_code >> 8) & 0xFF);
    data[6] = (uint8_t)((abort_code >> 16) & 0xFF);
    data[7] = (uint8_t)((abort_code >> 24) & 0xFF);
    SendSDO(sdo_number, who_am_i, data);
}

void MiniCANOpen::SendFailedSDO(uint8_t sdo_number, SDORole who_am_i, uint16_t target_index, uint8_t target_subindex,
                                SDOAbortCode abort_code)
{
    if(auto line = GetSDOLineOnUse(sdo_number, who_am_i); line)
    {
        if(who_am_i == SDORole::SDO_SERVER) ResetSDOLine(*line.value());
        else if(who_am_i == SDORole::SDO_CLIENT)
        {
            StopSDOTimer(*line.value());
            line.value()->state = SDOState::STATE_ABORTED_INTERNAL;
            line.value()->abort_code = abort_code;
        }
    }
    SendAbortSDO(sdo_number, who_am_i, target_index, target_subindex, abort_code);
}

void MiniCANOpen::ResetSDOLine(SDOTransfer& transfer)
{
    InitSDOLine(transfer, 0, 0, 0, SDOState::STATE_RESET);
}

void MiniCANOpen::InitSDOLine(SDOTransfer& transfer, uint8_t sdo_number, uint16_t target_index, uint8_t target_subindex,
                              SDOState state)
{
    transfer.Reset();
    transfer.sdo_number = sdo_number;
    transfer.target_index = target_index;
    transfer.target_subindex = target_subindex;
    transfer.state = state;
    switch(state)
    {
        case SDOState::STATE_DOWNLOAD_IN_PROGRESS:
        case SDOState::STATE_UPLOAD_IN_PROGRESS:
        case SDOState::STATE_BLOCK_DOWNLOAD_IN_PROGRESS:
        case SDOState::STATE_BLOCK_UPLOAD_IN_PROGRESS:
            StartSDOTimer(transfer); break;
        default:
            StopSDOTimer(transfer); break;
    }
}

std::optional<MiniCANOpen::SDOTransfer*> MiniCANOpen::GetSDOLineOnUse(const uint8_t sdo_number, const SDORole who_am_i)
{
    auto it = std::find_if(sdo_transfers.begin(), sdo_transfers.end(), [=](const SDOTransfer& t)
    {
       return (t.state != SDOState::STATE_RESET && t.state != SDOState::STATE_ABORTED_INTERNAL && t.sdo_number == sdo_number && t.role == who_am_i);
    });
    if(it == sdo_transfers.end()) return std::nullopt;
    return &(*it);
}

std::optional<MiniCANOpen::SDOTransfer*> MiniCANOpen::GetSDOLineFree(SDORole expected_who_am_i)
{
    auto it = std::find_if(sdo_transfers.begin(), sdo_transfers.end(), [=](const SDOTransfer& t)
    {
       return t.state == SDOState::STATE_RESET;
    });
    if(it != sdo_transfers.end()) // we have existing free line, set the whoami param
    {
        it->role = expected_who_am_i;
        return &(*it);
    }
    // No free line existed, create new one
    if(sdo_transfers.size() >= SDO_MAX_SIMULTANEOUS_TRANSFERS) return std::nullopt; // cannot create new lines due to limitation
    sdo_transfers.emplace_back();
    sdo_transfers.back().role = expected_who_am_i;
    return &sdo_transfers.back();
}

FuncRetCode MiniCANOpen::ParseSDOLineToOD(const SDOTransfer& transfer)
{
    return WriteODValue(transfer.target_index, transfer.target_subindex, transfer.data.size(), transfer.data.data(), false);
}

FuncRetCode MiniCANOpen::ParseODToSDOLine(SDOTransfer& transfer)
{
    uint8_t actual_size = GetODValueSize(transfer.target_index, transfer.target_subindex);
    if(actual_size > SDOTransfer::SDO_MAX_TRANSFER_SIZE) return FuncRetCode::BUFFER_FULL;
    transfer.data.clear();
    transfer.data.resize(actual_size);
    return ReadODValue(transfer.target_index, transfer.target_subindex, actual_size, transfer.data.data(), false);
}

void MiniCANOpen::StartSDOTimer(SDOTransfer& transfer)
{
    if(!transfer.swtimer_started)
    {
        transfer.swtimer_id = alarm_timer.GetUnusedTimerID();
        transfer.swtimer_started = true;
        alarm_timer.AddTimer(transfer.swtimer_id, SDO_TIMEOUT_MS, std::bind(&MiniCANOpen::SDOTransferTimerCallback, this, std::placeholders::_1));
    }
}

void MiniCANOpen::StopSDOTimer(SDOTransfer& transfer)
{
    if(transfer.swtimer_started)
    {
        transfer.swtimer_started = false;
        alarm_timer.DelTimer(transfer.swtimer_id);
        transfer.swtimer_id = 0;
    }
}

void MiniCANOpen::ResetSDOTimer(const SDOTransfer& transfer)
{
    if(transfer.swtimer_started) alarm_timer.ResetTimer(transfer.swtimer_id);
}

void MiniCANOpen::SDOTransferTimerCallback(uint16_t id)
{
    for(auto& transfer : sdo_transfers)
    {
        if(transfer.swtimer_started && transfer.swtimer_id == id)
        {
            DEBUG_PRINT("SDO Timeout. SDO response not received: %x:%x\n", transfer.target_index, transfer.target_subindex);
            StopSDOTimer(transfer);
            transfer.state = SDOState::STATE_ABORTED_INTERNAL;
            SendAbortSDO(transfer.sdo_number, transfer.role, transfer.target_index, transfer.target_subindex, SDOAbortCode::SDOABT_TIMED_OUT);
            transfer.abort_code = SDOAbortCode::SDOABT_TIMED_OUT;
            ResetSDOLine(transfer);
        }
    }
}

void MiniCANOpen::LifeguardInit()
{
    if(m_NodeId == 0xFF) return;
    uint16_t heartbeat_time_ms = 0;
    uint8_t actual_size = 0;
    if(ReadODValue(0x1017, 0x00, actual_size, &heartbeat_time_ms) == FuncRetCode::OK && actual_size == 2 && heartbeat_time_ms > 0)
    {
        producer_heartbeat_swtimer_id = alarm_timer.GetUnusedTimerID();
        alarm_timer.AddTimer(producer_heartbeat_swtimer_id, heartbeat_time_ms, std::bind(&MiniCANOpen::ProducerHeartbeatAlarmCallback, this, std::placeholders::_1));
    }
}

void MiniCANOpen::LifeguardStop()
{
    if(producer_heartbeat_swtimer_id != 0xFFFF)
    {
        alarm_timer.DelTimer(producer_heartbeat_swtimer_id);
        producer_heartbeat_swtimer_id = 0xFFFF;
    }
}

void MiniCANOpen::ProducerHeartbeatAlarmCallback(uint16_t id)
{
    if(m_NodeId == 0xFF) return;
    uint16_t heartbeat_time_ms = 0;
    uint8_t actual_size = 0;
    if(ReadODValue(0x1017, 0x00, actual_size, &heartbeat_time_ms) == FuncRetCode::OK && actual_size == 2 && heartbeat_time_ms > 0)
    {
        DataType::Comm::CANMessage msg
        {
            .cob_id = (uint16_t)((uint16_t)GetNodeID() + 0x700),
            .is_rtr = false,
            .len = 1
        };
        msg.data[0] = to_underlying(m_NodeState);
        can_send(msg);
        alarm_timer.SetTimerInterval(producer_heartbeat_swtimer_id, heartbeat_time_ms);
    }
    else LifeguardStop();
}

MiniCANOpen::DS301FuncCode MiniCANOpen::GetFuncCodeFromCOB(uint16_t cob_id)
{
    return (DS301FuncCode)(cob_id >> 7);
}

uint8_t MiniCANOpen::GetNodeIDFromCOB(uint16_t cob_id)
{
    return (cob_id & 0x7F);
}
}
