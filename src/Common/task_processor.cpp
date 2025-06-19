#include "task_processor.hpp"

namespace iFOC
{
FuncRetCode TaskProcessor::AppendTask(Task *task)
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    if(!IsTaskValid(task)) return FuncRetCode::INVALID_INPUT;
    taskENTER_CRITICAL();
    if(HasTask(task))
    {
        taskEXIT_CRITICAL();
        return FuncRetCode::PARAM_DUPLICATED;
    }
    InitializeTask(task);
    tasks.push_back(task);
    taskEXIT_CRITICAL();
    return FuncRetCode::OK;
}

FuncRetCode TaskProcessor::PushFrontTask(Task *task)
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    if(!IsTaskValid(task)) return FuncRetCode::INVALID_INPUT;
    taskENTER_CRITICAL();
    if(HasTask(task))
    {
        taskEXIT_CRITICAL();
        return FuncRetCode::PARAM_DUPLICATED;
    }
    InitializeTask(task);
    tasks.push_front(task);
    taskEXIT_CRITICAL();
    return FuncRetCode::OK;
}

FuncRetCode TaskProcessor::InsertTaskBeforeName(const char *nextTaskName, Task *task)
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION;
    if(!IsTaskValid(task)) return FuncRetCode::INVALID_INPUT;
    taskENTER_CRITICAL();
    if(HasTask(task))
    {
        taskEXIT_CRITICAL();
        return FuncRetCode::PARAM_DUPLICATED;
    }
    auto it = tasks.cbegin();
    while(it != tasks.cend())
    {
        if(*(*it) == nextTaskName)
        {
            InitializeTask(task);
            tasks.insert(it, task);
            taskEXIT_CRITICAL();
            return FuncRetCode::OK;
        }
        ++it;
    }
    taskEXIT_CRITICAL();
    return FuncRetCode::PARAM_NOT_EXIST;
}

FuncRetCode TaskProcessor::InsertTaskAfterName(const char* prevTaskName, Task* task)
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION;
    if(!IsTaskValid(task)) return FuncRetCode::INVALID_INPUT;
    taskENTER_CRITICAL();
    if(HasTask(task))
    {
        taskEXIT_CRITICAL();
        return FuncRetCode::PARAM_DUPLICATED;
    }
    auto it = tasks.cbegin();
    while(it != tasks.cend())
    {
        if(*(*it) == prevTaskName)
        {
            ++it;
            InitializeTask(task);
            if(it == tasks.cend()) tasks.push_back(task);
            else tasks.insert(it, task);
            taskEXIT_CRITICAL();
            return FuncRetCode::OK;
        }
        ++it;
    }
    taskEXIT_CRITICAL();
    return FuncRetCode::PARAM_NOT_EXIST;
}

std::optional<Task*> TaskProcessor::GetTaskByName(const char *name)
{
    for(auto* task : tasks)
    {
        if(*task == name) return std::make_optional(task);
    }
    return std::nullopt;
}

FuncRetCode TaskProcessor::RemoveTaskByName(const char *name)
{
    if(xPortIsInsideInterrupt()) return FuncRetCode::ACCESS_VIOLATION; // can't be running inside isr
    taskENTER_CRITICAL();
    auto it = tasks.begin();
    while(it != tasks.end())
    {
        if(*(*it) == name)
        {
            (*it)->Stop();
            delete *it;
            tasks.erase(it);
            taskEXIT_CRITICAL();
            return FuncRetCode::OK;
        }
        ++it;
    }
    taskEXIT_CRITICAL();
    return FuncRetCode::PARAM_NOT_EXIST;
}

// Vector<Task::TaskConfig> TaskProcessor::GetAllTaskConfigs() const
// {
//     Vector<Task::TaskConfig> configs;
//     // taskENTER_CRITICAL();
//     for(const auto* task : tasks)
//     {
//         configs.emplace_back(task->config);
//     }
//     // taskEXIT_CRITICAL();
//     return configs;
// }

const List<Task*> &TaskProcessor::GetTaskList()
{
    return tasks;
}

void TaskProcessor::_set_bypass_task_by_name(const char* name, bool bypass)
{
    if(auto task = GetTaskByName(name))
    {
        task.value()->SetBypass(bypass);
    }
}

TaskProcessor::~TaskProcessor()
{
    taskENTER_CRITICAL();
    auto it = tasks.begin();
    while(it != tasks.end())
    {
        (*it)->Stop();
        delete *it;
        it = tasks.erase(it);
    }
    taskEXIT_CRITICAL();
}
}