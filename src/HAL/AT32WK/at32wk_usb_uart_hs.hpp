#pragma once

#include "uart_hs_base.hpp"
#include "hal_const.h"

#if defined __has_include
#if __has_include("cdc_class.h") && __has_include("usbd_core.h")
#define USB_CDC_CLASS_PRESENT
#endif
#else
#define USB_CDC_CLASS_PRESENT
#endif

#if defined(AT32WK_ENV) && defined(USB_MODULE_ENABLED) && defined(USB_CDC_CLASS_PRESENT)

namespace iFOC::HAL
{
class USBUARTHS final : public UARTHSBase
{
public:
    USBUARTHS() = default;
    FuncRetCode Init(DataType::Comm::UARTBaudrate baud) override;
    void StartTransmit() override;
    void UpdateRxFIFO() override;
    void OnUSBIRQ();
private:
    static constexpr size_t USB_BUFFER_SIZE = (configTOTAL_HEAP_SIZE >= 16384 ? 1024 : 512);
    std::array<uint8_t, USB_BUFFER_SIZE> usb_buffer{};
    bool tx_pending = false;
};
}

#endif
