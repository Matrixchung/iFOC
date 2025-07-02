#pragma once

#include <cstdint>
#include "foc_types.hpp"
#include "software_timer.hpp"
#include "can_message.h"
#include "can_baudrate.h"

namespace iFOC
{
class MiniCANOpen
{
    using allocator_type = Allocator<void>;
public:
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(MiniCANOpen);
    allocator_type allocator;
    enum class NodeState : uint8_t
    {
        Initialisation,
        PreOperational,
        Operational,
        Stopped
    };
    enum NMTCommand : uint8_t
    {
        Start_Node = 0x01,
        Stop_Node = 0x02,
        Enter_PreOperational = 0x80,
        Reset_Node = 0x81,
        Reset_Comm = 0x82
    };
    enum ODObjectType : uint8_t
    {
        OBJ_NULL = 0x00,       /* An object with no data fields */
        OBJ_DOMAIN = 0x02,     /* Large variable amount of data e.g. executable program code */
        OBJ_DEFTYPE = 0x05,    /* Denotes a type definition such as a BOOLEAN, UNSIGNED16, REAL32 and so on */
        OBJ_DEFSTRUCT = 0x06,  /* Defines a new record type e.g. the PDO mapping structure at 0x21 */
        OBJ_VAR = 0x07,        /* A single value such as an UNSIGNED8, BOOLEAN, REAL32, INTEGER16, etc. */
        OBJ_ARRAY = 0x08,      /* A multiple data field object where each field shares the SAME basic data type */
        OBJ_RECORD = 0x09,     /* A multiple data field object where the data fields may be any combination of simple variables. */
    };
    enum ODDataType : uint8_t
    {
        BOOLEAN = 0x01,
        INT8 = 0x02,
        INT16 = 0x03,
        INT32 = 0x04,
        UINT8 = 0x05,
        UINT16 = 0x06,
        UINT32 = 0x07,
        REAL32 = 0x08,
        VISIBLE_STRING = 0x09,
        OCTET_STRING = 0x0A,
        UNICODE_STRING = 0x0B,
        TIME_OF_DAY = 0x0C,
        TIME_DIFFERENCE = 0x0D,
        DOMAIN = 0x0F,
        INT24 = 0x10,
        REAL64 = 0x11,
        INT40 = 0x12,
        INT48 = 0x13,
        INT56 = 0x14,
        INT64 = 0x15,
        UINT24 = 0x16,
        UINT40 = 0x18,
        UINT48 = 0x19,
        UINT56 = 0x1A,
        UINT64 = 0x1B
    };
    enum ODAccessType : uint8_t
    {
        RW = 0x00,
        WO = 0x01,
        RO = 0x02
    };
    enum class ODRWType : uint8_t
    {
        READ = 0x00,
        WRITE = 0x01
    };
    struct ODSubIndex
    {
        ODAccessType accessType = RW;
        ODDataType dataType = BOOLEAN;      // Defines of what datatype the entry is
        uint8_t size = 0;                  // The size (in Byte) of the object variable
        void* pObject = nullptr;            // The pointer of the variable, allocated by CANOPEN_MEMALLOC(), deallocated by CANOPEN_MEMFREE()
    };
    struct ODEntry;
    using ODRWCallback = std::function<void(ODRWType type, const ODEntry *entry, uint8_t subindex)>;
    struct ODEntry
    {
        Vector<ODSubIndex> subIndexes{}; // subindexCount can be obtained through subIndexes.size()
        ODObjectType objectType = OBJ_NULL;
        uint16_t index = 0;
        ODRWCallback rw_callback = nullptr; // Callback function on Read/Write event
    };
    using CANSendFunc = std::function<FuncRetCode(DataType::Comm::CANMessage& msg)>;
    using NMTCallback = std::function<void(NMTCommand new_state)>;
    using NodeStateCallback = std::function<void(NodeState prev_state, NodeState new_state)>;
    using LSSCallback = std::function<void(uint8_t command)>;
    using LSSStoreCallback = std::function<void(uint8_t& error_code, uint8_t& spec_error)>;
    explicit MiniCANOpen(CANSendFunc func);
    ~MiniCANOpen();

    void OnRxMessage(const DataType::Comm::CANMessage& msg);

    void SetNodeState(NodeState new_state);
    void AddODEntry(const ODEntry& entry); // Add if not exist, else override
    FuncRetCode SetVendorID(uint32_t vendor_id);
    FuncRetCode SetProductCode(uint32_t product_code);
    FuncRetCode SetRevisionNumber(uint32_t revision_number);
    FuncRetCode SetSerialNumber(uint32_t serial_number);
    FuncRetCode WriteODValue(uint16_t index, uint8_t subindex, uint8_t value_size, const void* value);
    FuncRetCode ReadODValue(uint16_t index, uint8_t subindex, uint8_t& actual_size, void* dest);
    std::optional<ODEntry*> FindODEntry(uint16_t index);
    [[nodiscard]] uint8_t GetNodeID() const;
    void SetNodeID(uint8_t node_id);
    void RegisterNMTCallback(NMTCallback cb);
    void RegisterNodeStateCallback(NodeStateCallback cb);
    void RegisterLSSCallback(LSSCallback cb);
    void RegisterLSSStoreCallback(LSSStoreCallback cb);
private:
    enum DS301FuncCode : uint8_t
    {
        NMT = 0x00,
        SYNC = 0x01,
        TIMESTAMP = 0x02,
        TX_PDO_1 = 0x03,
        RX_PDO_1 = 0x04,
        TX_PDO_2 = 0x05,
        RX_PDO_2 = 0x06,
        TX_PDO_3 = 0x07,
        RX_PDO_3 = 0x08,
        TX_PDO_4 = 0x09,
        RX_PDO_4 = 0x0A,
        TX_SDO = 0x0B,
        RX_SDO = 0x0C,
        NODE_GUARD = 0x0E,
        LSS = 0x0F
    };
    enum LSSServices : uint8_t
    {
        LSS_SM_GLOBAL = 0x04,
        LSS_SM_SELECTIVE_VENDOR = 0x40,
        LSS_SM_SELECTIVE_PRODUCT = 0x41,
        LSS_SM_SELECTIVE_REVISION = 0x42,
        LSS_SM_SELECTIVE_SERIAL = 0x43,
        LSS_SM_SELECTIVE_RESP = 0x44,
        // iFOC Extra LSSService, allowing to select a device using matched serial number only.
        LSS_EXTRA_SM_SELECTIVE_SERIAL_ONLY = 0x45,
        LSS_CONF_NODE_ID = 0x11,
        LSS_CONF_BIT_TIMING = 0x13,
        LSS_CONF_ACT_BIT_TIMING = 0x15,
        LSS_CONF_STORE = 0x17,
        LSS_INQ_VENDOR_ID = 0x5A,
        LSS_INQ_PRODUCT_CODE = 0x5B,
        LSS_INQ_REV_NUMBER = 0x5C,
        LSS_INQ_SERIAL_NUMBER = 0x5D,
        LSS_INQ_NODE_ID = 0x5E,
        // iFOC Extra LSSService, allowing to respond serial number even in waiting mode.
        LSS_EXTRA_INQ_SERIAL_ONLY = 0x5F,
        LSS_IDENT_REMOTE_VENDOR = 0x46,
        LSS_IDENT_REMOTE_PRODUCT = 0x47,
        // fuzzy match (LOW recv <= stored, HIGH recv >= stored)
        LSS_IDENT_REMOTE_REV_LOW = 0x48,
        LSS_IDENT_REMOTE_REV_HIGH = 0x49,
        LSS_IDENT_REMOTE_SERIAL_LOW = 0x4A,
        LSS_IDENT_REMOTE_SERIAL_HIGH = 0x4B,
        LSS_IDENT_REMOTE_NON_CONF = 0x4C,
        // iFOC Extra LSSService, allowing to identify a device using matched serial number only.
        LSS_EXTRA_IDENT_REMOTE_SERIAL_ONLY = 0x4D,
        LSS_IDENT_SLAVE = 0x4F,
        LSS_IDENT_NON_CONF_SLAVE = 0x50,
        LSS_IDENT_FASTSCAN = 0x51
    };
    enum LSSMode : uint8_t
    {
        LSS_WAITING_MODE = 0,
        LSS_CONFIGURATION_MODE = 1
    };
    struct LSSTransfer
    {
        uint8_t mode = LSSMode::LSS_WAITING_MODE;
        uint8_t node_id = 0;
        uint8_t addr_sel_match = 0;
        uint8_t addr_ident_match = 0;
        DataType::Comm::CANBaudrate target_baudrate{DataType::Comm::CANBaudrate::BAUD_10_KBPS};
        // uint8_t switch_delay_state = 0;
        // uint16_t switch_delay = 0;
        LSSCallback callback = nullptr;
        LSSStoreCallback store_callback = nullptr;
    };
    struct HandleState
    {
        bool handle_bootup = false;
        bool handle_sdo = false;
        bool handle_emergency = false;
        bool handle_sync = false;
        bool handle_lifeguard = false;
        bool handle_pdo = false;
        bool handle_lss = false;
    };
    void ProcessNMTMessage(const DataType::Comm::CANMessage& msg);
    void ProcessSYNCMessage(const DataType::Comm::CANMessage& msg);
    void ProcessEMCYMessage(const DataType::Comm::CANMessage& msg);
    void ProcessPDOMessage(const DataType::Comm::CANMessage& msg);
    void ProcessSDOMessage(const DataType::Comm::CANMessage& msg);
    void ProcessLSSMessage(const DataType::Comm::CANMessage& msg);
    void SwitchCurrentHandleState(const HandleState& new_state);
    void SendSlaveLSSMessage(LSSServices command, void *data1, void *data2) const;
    void SendBootUp() const;
    static DS301FuncCode GetFuncCodeFromCOB(uint16_t cob_id);
    static uint8_t GetNodeIDFromCOB(uint16_t cob_id);
    uint8_t m_NodeId = 0xFF;
    NodeState m_NodeState = NodeState::Initialisation;
    HandleState curr_handle_state;
    HashMap<uint16_t, ODEntry> m_objectDict{};
    LSSTransfer lss_transfer;
    SoftwareTimer alarm_timer;
    CANSendFunc can_send = nullptr;
    NMTCallback nmt_callback = nullptr;
    NodeStateCallback node_state_callback = nullptr;
};
}