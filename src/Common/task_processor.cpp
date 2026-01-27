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
    tasks.push_back(task);
    InitializeTask(task);
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
    tasks.push_front(task);
    InitializeTask(task);
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
            tasks.insert(it, task);
            InitializeTask(task);
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
            if(it == tasks.cend()) tasks.push_back(task);
            else tasks.insert(it, task);
            InitializeTask(task);
            taskEXIT_CRITICAL();
            return FuncRetCode::OK;
        }
        ++it;
    }
    taskEXIT_CRITICAL();
    return FuncRetCode::PARAM_NOT_EXIST;
}

Task* TaskProcessor::GetTaskByName(const char *name)
{
    for(auto* task : tasks)
    {
        if(*task == name) return task;
    }
    return nullptr;
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

bool TaskProcessor::HasTask(const Task *task) const
{
    for(const auto* t : tasks)
    {
        if(t && *t == *task) return true;
    }
    return false;
}

const List<Task*> &TaskProcessor::GetTaskList()
{
    return tasks;
}

void TaskProcessor::_set_bypass_task_by_name(const char* name, bool bypass)
{
    if(auto task = GetTaskByName(name))
    {
        task->SetBypass(bypass);
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