#ifndef _RTOS_TASK_H
#define _RTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"

class RTOSTask
{
public:
    virtual ~RTOSTask() {}
    inline void start(const char * const pcName, const uint16_t usStackDepth, UBaseType_t uxPriority) 
    {
        this->stopCalled = false;
        xTaskCreate(RTOSTask::bootstrap, pcName, usStackDepth, this, uxPriority, &this->xHandle);
    };
    inline void stop() 
    {
        this->stopCalled = true;
    };

protected:
    inline void sleep(int time_ms) 
    {
        vTaskDelay(pdMS_TO_TICKS(time_ms));
    };
    TaskHandle_t xHandle = nullptr;
private:
    virtual void setup() {}
    virtual void loop() {}
    virtual void cleanup() {}

    static void bootstrap(void* pvParameters) 
    {
        RTOSTask* taskObject = reinterpret_cast<RTOSTask*>(pvParameters);
        taskObject->setup();
        while (!taskObject->stopCalled) taskObject->loop();
        taskObject->cleanup();
        vTaskDelete(taskObject->xHandle);
    }
    bool stopCalled = false;
};

#endif