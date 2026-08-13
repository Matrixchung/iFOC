#include "at32wk_usb_uart_hs.hpp"

#if defined(AT32WK_ENV) && defined(USB_MODULE_ENABLED) && defined(USB_CDC_CLASS_PRESENT)

#include "cdc_class.h"
#include "cdc_desc.h"
#include "usbd_core.h"

namespace iFOC::HAL
{
#if defined(AT32F403Axx)
    FuncRetCode USBUARTHS::Init(DataType::Comm::UARTBaudrate baud)
    {
        (void)baud; // USB CDC does not use baudrate
        FuncRetCode ret = tx_fifo.init(USB_BUFFER_SIZE);
        if(ret != FuncRetCode::OK) return ret;
        ret = rx_fifo.init(USB_BUFFER_SIZE);
        if(ret != FuncRetCode::OK) return ret;
        crm_periph_clock_enable(CRM_USB_PERIPH_CLOCK, TRUE); // Enable USB clock
        /*
         * Note: from AT32F403Ax Reference Manual, when both USB & CAN peripheral activated,
         *       USB interrupts will be remapped to IRQ Line 73 & 74 (USBFS_MAPH/L)
         *       see: crm_usb_interrupt_remapping_set(CRM_USB_INT73_INT74);
         */
        usbd_core_init(&usb_core_dev, USB, &cdc_class_handler, &cdc_desc_handler, 0);
        usbd_connect(&usb_core_dev);
        return FuncRetCode::OK;
    }

    void USBUARTHS::StartTransmit()
    {
        if(tx_fifo.used() > 0) tx_pending = true;
    }

    void USBUARTHS::UpdateRxFIFO()
    {
        UARTHSBase::UpdateRxFIFO();
    }

    void USBUARTHS::OnUSBIRQ()
    {
        usbd_irq_handler(&usb_core_dev);
        const uint16_t recv_len = usb_vcp_get_rxdata(&usb_core_dev, usb_buffer.data());
        if(recv_len > 0)
        {
            rx_fifo.put(usb_buffer.data(), recv_len);
            if(idle_cb) idle_cb(this);
            if(tx_pending)
            {
                const uint16_t len = tx_fifo.peek(usb_buffer.data(), usb_buffer.max_size());
                if(len > 0)
                {
                    if(usb_vcp_send_data(&usb_core_dev, usb_buffer.data(), len) == SUCCESS)
                    {
                        tx_fifo.wipe_n(len);
                    }
                }
                if(tx_fifo.used() == 0) tx_pending = false;
            }
        }
    }

#endif
}

#endif
