#pragma once

#include "protocol_base.hpp"
#include "../Common/Filter/sliding_filter.hpp"
#include "../DataType/Headers/Base/motion.hpp"
#include "../Common/foc_task.hpp"
#include "../Common/Interface/canfd_base.hpp"

namespace iFOC::Protocol
{
class CANFDProtocol final : public ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(CANFDProtocol);
public:
    explicit CANFDProtocol(HAL::CANFDBase* base);
    ~CANFDProtocol();
    void Init() override;
private:
    void HandleNodeAllocation(const uint8_t* payload, uint8_t len);
    void HandleGetNodeInfo(const uint8_t* payload, uint8_t len);
    void HandleExecuteOpcode(const uint8_t* payload, uint8_t len);
    void HandleParamGetSet(const uint8_t* payload, uint8_t len);
    void HandlePosControl(const uint8_t* payload, uint8_t len);
    void HandleVelTorqueControl(const uint8_t* payload, uint8_t len);
    void HandleFileDownload(const uint8_t* payload, uint8_t len);
    void ResetParamSession();
    bool ParseMotionHeader(const uint8_t header);
    void ProcessControlStruct(const bool latch_arrived);
    void SendOpcodeSession();
    void SendNodeStatus();
    void SendMiscFeedback();
    void SendRTFeedback();
    void PrepareNodeInfo();
    void TransmitPacket(uint8_t frame_id, const uint8_t payload_len, uint8_t* payload) const;
    void SetNodeID(uint8_t new_id);
    bool OnRxEvent(const DataType::Comm::CANFDMessage& msg);
    class PeriodicTask final : public Task
    {
        OVERRIDE_NEW();
        DELETE_COPY_CONSTRUCTOR(PeriodicTask);
    private:
        CANFDProtocol* parent;
        struct
        {
            TickType_t heartbeat = 0;
            TickType_t misc_feedback = 0;
        } next_send_tick;
        TickType_t xLastWakeTick = 0;
        TickType_t next_sample_cpu_usage = 0;
        TickType_t param_session_last_update_tick = 0;
        uint32_t observed_param_session_activity = 0;
        uint32_t last_runtime_counter = 0;
        uint32_t last_idle_runtime = 0;
    public:
        explicit PeriodicTask(CANFDProtocol* p);
        void InitNormal() override;
        void UpdateNormal() override;
        void UpdateMid(float Ts) override;
    };
    friend class PeriodicTask;
    HAL::CANFDBase* can;
    PeriodicTask periodic_task;
    TaskTimer response_timer;
    struct
    {
        Filter::SlidingFilter response_avg{100};
        Filter::SlidingFilter rt_task_avg{100};
        Filter::SlidingFilter mid_task_avg{100};
    } task_time_avg;
    struct
    {
        uint8_t data_len;
        uint8_t data[31];
    } node_info_packet{};
    struct
    {
        uint8_t session_id;
        uint8_t opcode;
        uint8_t status;
        uint8_t argument_len;
        uint8_t argument_echo_len;
        uint8_t argument[62];
        uint8_t argument_echo[61];
    } opcode_session{};
    struct
    {
        uint8_t state = 0; // IDLE(0), REQ_RX(1), RSP_READY(2)
        uint16_t session_id = 0;
        uint8_t flags = 0;
        uint16_t request_total_len = 0;
        uint16_t request_len = 0;
        uint16_t request_crc16 = 0;
        uint16_t response_len = 0;
        uint8_t response_complete = 0;
        uint8_t request_buffer[255]{};
        uint8_t response_buffer[255]{};
        uint32_t activity_seq = 0;
    } param_session{};
    struct
    {
        enum State : uint8_t
        {
            IDLE = 0,
            RECEIVING = 1,
            COMPLETE = 2,
        };
        State state = IDLE;
        uint8_t target = 0;
        uint16_t session_id = 0;
        uint32_t file_size = 0;
        uint32_t current_file_offset = 0;
        uint32_t last_data_offset = 0;
        uint64_t expected_crc64 = 0;
        uint8_t last_data_len = 0;
        uint8_t last_data_valid = 0;
        uint8_t last_data[56]{};
    } file_download{};
    struct
    {
        typedef enum CommandType : uint8_t
        {
            TORQUE = 0,
            VELOCITY = 1,
            POSITION = 2,
            MIT = 3,
            HYBRID = 4
        } CommandType;
        CommandType command = TORQUE;
        bool is_latched = false;
        bool is_relative = false;
        bool is_relative_curr_based = false;
        bool is_trajectory = false;
        bool is_trajectory_s_curve = false;
        Motion motion{};
        float mit_kp = 0.0f;
        float mit_kd = 0.0f;

        void Reset()
        {
            command = TORQUE;
            is_latched = false;
            is_relative = false;
            is_relative_curr_based = false;
            is_trajectory = false;
            is_trajectory_s_curve = false;
            motion.Reset();
            mit_kp = 0.0f;
            mit_kd = 0.0f;
        }
    } control_struct;
    uint16_t pos_command_seq = 0;
    uint8_t buffer[64]{};
    uint8_t node_id = 15;
    uint8_t vssc = 0;
    uint8_t rt_feedback_seq = 0;
    uint8_t cpu_usage_pct = 0;
};
}
