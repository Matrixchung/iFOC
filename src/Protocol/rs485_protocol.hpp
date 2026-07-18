#pragma once

#include "protocol_base.hpp"
#include "../DataType/Headers/Base/motion.hpp"
#include "../Common/foc_task.hpp"
#include "../Common/Interface/uart_hs_base.hpp"

namespace iFOC::Protocol
{
class RS485Protocol final : public ProtocolBase
{
    OVERRIDE_NEW();
    DELETE_COPY_CONSTRUCTOR(RS485Protocol);
public:
    explicit RS485Protocol(HAL::UARTHSBase* u);
    ~RS485Protocol();
    void Init() override;
    void SetScopeData(uint8_t ch, float data);
private:
    void IdleCallback(HAL::UARTHSBase* u);
    void ProcessRxPacket();
    void HandleNewNodeDiscovery();
    void HandleExecuteOpcode();
    void HandleParamGetSet();
    void HandlePosControl();
    void HandleVelTorqueControl();
    void HandleFileDownload();
    void HandleSerialScope();
    void ProcessControlStruct(bool latch_arrived);
    bool ParseMotionHeader(uint8_t header);
    void SendFileDownloadReply(uint8_t status, uint32_t info);
    void SendNodeStatus(uint8_t frame_id);
    void FillNodeDetailedInfo();
    void SendNodeDetailedInfo();
    void FillMiscFeedback();
    void SendMiscFeedback();
    void SendRTFeedback();
    void WriteTxPacket(); // Set header & calculate CRC16 & write to uart buffer
    class WorkerTask : public Task
    {
    public:
        explicit WorkerTask(RS485Protocol* p);
    protected:
        void UpdateMid(float Ts) override;
    private:
        RS485Protocol* parent = nullptr;
    } worker{this};
    friend class WorkerTask;
    std::array<float, 8> scope_data;
    HAL::UARTHSBase* uart;
    typedef union packet_info
    {
        uint8_t byte;
        struct
        {
            uint8_t id : 4;
            uint8_t frame_id : 4;
        } bit;
    } packet_info;
    struct
    {
        packet_info info;
        uint8_t data_len;
        uint8_t data[257]; // include CRC16
    } rx_packet;
    struct
    {
        uint8_t header_1;
        uint8_t header_2;
        packet_info info;
        uint8_t data_len;
        uint8_t data[255];
    } tx_packet;
    struct
    {
        uint8_t header_1;
        uint8_t header_2;
        packet_info info;
        uint8_t data_len;
        uint8_t data[30];
    } node_info_packet;
    struct
    {
        uint8_t header_1;
        uint8_t header_2;
        packet_info info;
        uint8_t data_len;
        uint8_t data[10];
    } misc_fb_packet;
    struct
    {
        uint8_t current_state = 0; // IDLE(0), REQ_RX(1), RSP_READY(2)
        uint8_t session_id = 0;
        uint8_t flags = 0;
        uint16_t request_total_len = 0;
        uint16_t request_len = 0;
        uint16_t request_crc16 = 0;
        uint16_t response_len = 0;
        uint8_t response_complete = 0;
        uint8_t request_buffer[255]{};
        uint8_t response_buffer[255]{};
        TickType_t last_update_tick = 0;
    } get_set_struct;
    struct
    {
        uint8_t current_state; // WAITING(0), BEGIN(1), DATA(2), FIN(3)
        uint8_t target;
        uint16_t session_id;
        uint32_t file_size;
        uint32_t current_file_offset; // already read, the offset is next empty byte
        // uint8_t data_buffer[255];
    } file_dl_struct;
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
    TaskTimer response_timer{};
    TickType_t control_frame_last_misc_send_tick = 0;
    uint8_t node_id = 15;
    uint8_t vssc = 0;
    uint8_t scope_channel_cnt = 0;
};
}
