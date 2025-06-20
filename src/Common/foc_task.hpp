#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "foc_types.hpp"
#include "foc_math.hpp"
#include "task_timer.hpp"
#include <bitset>

namespace iFOC
{
class Task
{
    OVERRIDE_NEW();
public:
    static constexpr size_t MAX_TASK_NAME_LEN = 13;
    Task() = delete;
    explicit Task(const char* taskName)
    {
        strncpy(config.name, taskName, sizeof(config.name));
    }
    [[nodiscard]] const char* GetName() const { return config.name; }
    [[nodiscard]] UBaseType_t GetRTOSPriority() const { return config.rtos_priority; }
    virtual ~Task()
    {
        ///! We can't delete a nullptr handle (in case of RTOS task has not been created)
        if(IsTaskRegistered(TaskType::NORMAL_TASK) && xHandle) vTaskDelete(xHandle);
    };
    struct TaskConfig
    {
        // bool register_rt_loop : 1 = false;
        // bool register_mid_loop : 1 = false;
        // bool register_normal_loop : 1 = false;
        uint8_t task_register_states = 0; // low 3 bits: 2[normal_loop, mid_loop, rt_loop]0
        char name[MAX_TASK_NAME_LEN] = "\0";
        uint16_t stack_depth = 128;
        // priority selection: https://www.cnblogs.com/yangguang-it/p/7156445.html
        UBaseType_t rtos_priority = tskIDLE_PRIORITY;
    };
    enum class TaskType : uint8_t
    {
        RT_TASK = 0,
        MID_TASK = 1,
        NORMAL_TASK = 2,
    };
    inline void Start()
    {
        stopCalled = false;
        xTaskCreate(Task::bootstrap,
                    config.name,
                    config.stack_depth,
                    this,
                    config.rtos_priority,
                    &xHandle);
    }
    inline void Stop()
    {
        this->stopCalled = true;
    };

    template<typename... Args>
    __fast_inline void RegisterTask(Args... args) { (..., _register_task(args)); }

    __fast_inline bool IsTaskRegistered(TaskType type) { return config.task_register_states & (1 << to_underlying(type)); }

    inline void SetBypass(bool bypass)
    {
        if(bypass)
        {
            // _task_bypassed_flags = config.task_register_states;
            config.task_register_states &= ~(1 << to_underlying(TaskType::RT_TASK));
            config.task_register_states &= ~(1 << to_underlying(TaskType::MID_TASK));
            if(IsTaskRegistered(TaskType::NORMAL_TASK) && xHandle) vTaskSuspend(xHandle);
        }
        else
        {
            config.task_register_states = _task_bypassed_flags;
            if(IsTaskRegistered(TaskType::NORMAL_TASK) && xHandle) vTaskResume(xHandle);
        }
    }

    __fast_inline TaskHandle_t GetHandle() { return xHandle; }

    bool operator==(const char* name) const noexcept
    {
        return strncasecmp(GetName(), name, sizeof(Task::config.name)) == 0;
    }
    bool operator!=(const char* name) const noexcept { return !(*this == name); };
    bool operator==(const Task& other) const noexcept { return (*this == other.GetName()); }
    bool operator!=(const Task& other) const noexcept { return !(*this == other); };
protected:
    friend class TaskProcessor;
    template <uint8_t> friend class MotorBase;
    static inline void sleep(int time_ms) { vTaskDelay(pdMS_TO_TICKS(time_ms)); }
    TaskHandle_t xHandle = nullptr;
    TaskConfig config;
    // std::optional<void*> motor;
    void* _motor;
    template<class T>
    T* GetMotor() { return reinterpret_cast<T*>(_motor); };
private:
    virtual void InitRT() {};
    virtual void UpdateRT(float Ts) {};
    virtual void InitMid() {};
    virtual void UpdateMid(float Ts) {};
    virtual void InitNormal() {};
    virtual void UpdateNormal() {};
    volatile uint8_t _task_bypassed_flags = 0;
    void _register_task(TaskType type)
    {
        config.task_register_states |= (1 << to_underlying(type));
        _task_bypassed_flags = config.task_register_states;
    }

    static void bootstrap(void* pvParameters)
    {
        auto* taskObject = reinterpret_cast<Task*>(pvParameters); // pvParameters == (*this)
        taskObject->InitNormal();
        while(!taskObject->stopCalled) taskObject->UpdateNormal();
        delete taskObject;
    }
    bool stopCalled = false;
};
}