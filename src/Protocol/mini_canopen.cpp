#include "mini_canopen.hpp"
#include "cpp_classes.hpp"

#define SLSS_ADDRESS 0x7E4
#define MLSS_ADDRESS 0x7E5

#define DEBUG_PRINT(...) uart_1.Print(true, __VA_ARGS__)

namespace iFOC
{
MiniCANOpen::MiniCANOpen(MiniCANOpen::CANSendFunc func)
{
    can_send = func;
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

FuncRetCode MiniCANOpen::WriteODValue(uint16_t index, uint8_t subindex, uint8_t value_size, const void* value)
{
    auto entry = FindODEntry(index);
    if(!entry || subindex >= entry.value()->subIndexes.size()) return FuncRetCode::PARAM_NOT_EXIST;
    auto& sub = entry.value()->subIndexes[subindex];
    // if(sub.accessType == ODAccessType::RO) return FuncRetCode::ACCESS_VIOLATION; // Disable access check
    if(value_size > sub.size) return FuncRetCode::PARAM_OUT_BOUND;
    memcpy(sub.pObject, value, value_size);
    if(entry.value()->rw_callback) entry.value()->rw_callback(ODRWType::WRITE, entry.value(), subindex);
    return FuncRetCode::OK;
}

FuncRetCode MiniCANOpen::ReadODValue(uint16_t index, uint8_t subindex, uint8_t& actual_size, void* dest)
{
    auto entry = FindODEntry(index);
    if(!entry || subindex >= entry.value()->subIndexes.size()) return FuncRetCode::PARAM_NOT_EXIST;
    auto& sub = entry.value()->subIndexes[subindex];
    // if(sub.accessType == ODAccessType::WO) return FuncRetCode::ACCESS_VIOLATION;
    actual_size = sub.size;
    memcpy(dest, sub.pObject, actual_size);
    if(entry.value()->rw_callback) entry.value()->rw_callback(ODRWType::READ, entry.value(), subindex);
    return FuncRetCode::OK;
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

void MiniCANOpen::ProcessEMCYMessage(const DataType::Comm::CANMessage& msg)
{

}

void MiniCANOpen::ProcessPDOMessage(const DataType::Comm::CANMessage& msg)
{

}

void MiniCANOpen::ProcessSDOMessage(const DataType::Comm::CANMessage& msg)
{

}

// https://blog.csdn.net/qq_38156743/article/details/143882903
/*
 * Typical CANopen master LSS sequence:
 * 1) By sending correct Vendor ID and Product ID (predefined), with REV_LOW / HIGH, SERIAL_LOW / HIGH,
 *    we can get all devices' serial number and revision number (in most scenes, they are not in pairs with each other)
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
    StartOrStop(handle_sdo, None, None); // None, resetSDO
    StartOrStop(handle_sync, None, None); // startSYNC, stopSYNC
    StartOrStop(handle_lifeguard, None, None); // lifeGuardInit, lifeGuartStop
    StartOrStop(handle_emergency, None, None); // emergencyInit, emergencyStop
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

MiniCANOpen::DS301FuncCode MiniCANOpen::GetFuncCodeFromCOB(uint16_t cob_id)
{
    return (DS301FuncCode)(cob_id >> 7);
}

uint8_t MiniCANOpen::GetNodeIDFromCOB(uint16_t cob_id)
{
    return (cob_id & 0x7F);
}
}
