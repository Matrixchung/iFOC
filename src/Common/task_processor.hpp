#pragma once

// Require: FreeRTOS

#include "foc_task.hpp"
#include "foc_types.hpp"
#include "func_ret_code.h"
#include <vector>
#include <list>
#include <deque>
/*
 *  Task  ->     Real-Time Task (RTTask)    |    Scheduled Task (NormalTask)
 *  Host   |       RTTaskScheduler()        |       FreeRTOS Scheduler
 *  Prio.  | Based on TaskNode.rt_priority  | Set priority when spawning task
 *  Sleep  |  Not implemented in RT tasks   |           vTaskDelay()
 * Preempt |             N/A                |             Support
 * Context |             N/A                |          Context saved
 */

namespace iFOC
{
class TaskProcessor
{
    OVERRIDE_NEW();
public:
    /// AppendTask() appends a new task to the end of task list.
    /// \param task: iFOC::Task*
    /// \brief before: head(task1) -> tail(task2) \n
    ///        after: head(task1) -> task2 -> \b tail(taskNew)
    /// \return FuncRetCode::OK:               the operation was completed successfully     \n
    ///         FuncRetCode::ACCESS_VIOLATION: the function cannot be run inside interrupts \n
    ///         FuncRetCode::INVALID_INPUT:    the input is not a valid task pointer        \n
    ///         FuncRetCode::PARAM_DUPLICATED: there's already an existing task which has
    ///                                        the same identifier with the new task
    FuncRetCode AppendTask(Task* task);

    /// PushFrontTask() appends a new task to the beginning of task list.
    /// \param task: iFOC::Task*
    /// \brief before: head(task1) -> tail(task2) \n
    ///        after: head(taskNew) -> task1 -> \b tail(task2)
    /// \return FuncRetCode::OK:               the operation was completed successfully     \n
    ///         FuncRetCode::ACCESS_VIOLATION: the function cannot be run inside interrupts \n
    ///         FuncRetCode::INVALID_INPUT:    the input is not a valid task pointer        \n
    ///         FuncRetCode::PARAM_DUPLICATED: there's already an existing task which has
    ///                                        the same identifier with the new task
    FuncRetCode PushFrontTask(Task* task);

    /// InsertTaskBeforeName() inserts a new task to the previous position of an existing task
    /// \param nextTaskName : const char*
    /// \param task : iFOC::Task*
    /// \brief before: head(task1) -> task2(nextTaskName == task2.config.name) -> tail(task3) \n
    ///        after: head(task1) -> \b taskNew -> task2 -> tail(task3)
    /// \return
    FuncRetCode InsertTaskBeforeName(const char* nextTaskName, Task* task);

    ///
    /// \param prevTaskName
    /// \param task
    /// \return
    FuncRetCode InsertTaskAfterName(const char* prevTaskName, Task* task);

    ///
    /// \param Ts
    __fast_inline void RTTaskScheduler(float Ts);

    ///
    /// \param Ts
    __fast_inline void MidTaskScheduler(float Ts);

    ///
    /// \param name
    /// \return
    [[nodiscard]] std::optional<Task*> GetTaskByName(const char* name);

    ///
    /// \param name
    /// \return
    FuncRetCode RemoveTaskByName(const char* name);

    template<typename... Args>
    void BypassTaskByName(Args... args);

    template<typename... Args>
    void UnbypassTaskByName(Args... args);

    ///
    /// \return
    // [[nodiscard]] Vector<Task::TaskConfig> GetAllTaskConfigs() const;

    [[nodiscard]] const List<Task*>& GetTaskList();

    ~TaskProcessor();
private:
    List<Task*> tasks;
    __fast_inline bool HasTask(const Task *task);
    static __fast_inline bool IsTaskValid(const Task *task);
    __fast_inline void InitializeTask(Task *task);
    void _set_bypass_task_by_name(const char* name, bool bypass);
};

__fast_inline bool TaskProcessor::HasTask(const Task *task)
{
    for(const auto* t : tasks)
    {
        if(*t == *task) return true;
    }
    return false;
}

__fast_inline bool TaskProcessor::IsTaskValid(const Task *task)
{
    return (task && strlen(task->config.name));
}

__fast_inline void TaskProcessor::InitializeTask(Task *task)
{
    if(task->IsTaskRegistered(Task::TaskType::RT_TASK)) task->InitRT();
    if(task->IsTaskRegistered(Task::TaskType::MID_TASK)) task->InitMid();
    if(task->IsTaskRegistered(Task::TaskType::NORMAL_TASK)) task->Start();
}

__fast_inline void TaskProcessor::RTTaskScheduler(float Ts)
{
    auto it = tasks.cbegin();
    while(it != tasks.cend())
    {
        if((*it)->IsTaskRegistered(Task::TaskType::RT_TASK)) (*it)->UpdateRT(Ts);
        ++it;
    }
}

__fast_inline void TaskProcessor::MidTaskScheduler(float Ts)
{
    auto it = tasks.cbegin();
    while(it != tasks.cend())
    {
        if((*it)->IsTaskRegistered(Task::TaskType::MID_TASK)) (*it)->UpdateMid(Ts);
        ++it;
    }
}

template<typename... Args>
void TaskProcessor::BypassTaskByName(Args... args)
{
    (..., _set_bypass_task_by_name(args, true));
}

template<typename... Args>
void TaskProcessor::UnbypassTaskByName(Args... args)
{
    (..., _set_bypass_task_by_name(args, false));
}

}
