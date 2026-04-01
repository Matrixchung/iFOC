#include "gd32_spi.hpp"

#if defined(GD32_ENV)

#include "hal_impl.hpp"
#include "foc_math.hpp"

namespace iFOC::HAL
{
SPI::SPI(const uint32_t spi_periph, GPIOBase& _cs) : hspi(spi_periph), cs(_cs)
{
    spi_struct_para_init(&spi_init_struct);
}

SPI::SPI(const uint32_t spi_periph, GPIOBase* _cs) : SPI(spi_periph, *_cs) {}

FuncRetCode SPI::Init()
{
    cs.ModeOutPP();
    cs.Set();
    rcu_periph_enum spi_clock_signal = RCU_SPI0;
#if defined(GD32G5X3)
    if(hspi == SPI0) spi_clock_signal = RCU_SPI0;
    else if(hspi == SPI1) spi_clock_signal = RCU_SPI1;
    else if(hspi == SPI2) spi_clock_signal = RCU_SPI2;
    else return FuncRetCode::PARAM_OUT_BOUND;
#endif
    spi_deinit(hspi);
    DelayUs(1);
    rcu_periph_clock_enable(spi_clock_signal);
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init(hspi, &spi_init_struct);
    spi_enable(hspi);
    return FuncRetCode::OK;
}

FuncRetCode SPI::WriteBytes(const uint8_t* data, const uint16_t size)
{
    uint32_t tickstart = xTaskGetTickCount();
    uint8_t* tx_address = (uint8_t*)data;
    uint16_t len = size;
    constexpr uint32_t TIMEOUT = 10;
    cs.Clear();
    (void)(SPI_DATA(hspi)); // UNUSED(hspi->dt)
    while(len--)
    {
        while(!(SPI_STAT(hspi) & SPI_FLAG_TBE))
        {
            if(xTaskGetTickCount() > tickstart + TIMEOUT)
            {
                cs.Set();
                (void)(SPI_DATA(hspi));
                return FuncRetCode::HARDWARE_ERROR;
            }
            tickstart = xTaskGetTickCount();
            spi_data_transmit(hspi, (uint8_t)*tx_address);
            tx_address++;
        }
    }
    cs.Set();
    (void)(SPI_DATA(hspi));
    return FuncRetCode::OK;
}

FuncRetCode SPI::ReadBytes(uint8_t* data, const uint16_t size)
{
    return WriteReadBytes(data, data, size);
}

FuncRetCode SPI::WriteReadBytes(const uint8_t *write_data, uint8_t *read_data, const uint16_t size)
{
    uint32_t tickstart = xTaskGetTickCount();
    uint8_t* tx_address = (uint8_t*)write_data;
    uint8_t* rx_address = (uint8_t*)read_data;
    uint16_t len = size;
    cs.Clear();
    (void)(SPI_DATA(hspi));
    while(len--)
    {
        constexpr uint32_t TIMEOUT = 10;
        while(!(SPI_STAT(hspi) & SPI_FLAG_TBE))
        {
            if(xTaskGetTickCount() - tickstart > TIMEOUT)
            {
                cs.Set();
                (void)(SPI_DATA(hspi));
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        tickstart = xTaskGetTickCount();
        spi_data_transmit(hspi, (uint8_t)*tx_address);
        tx_address++;
        while(!(SPI_STAT(hspi) & SPI_FLAG_RBNE))
        {
            if(xTaskGetTickCount() - tickstart > TIMEOUT)
            {
                cs.Set();
                (void)(SPI_DATA(hspi));
                return FuncRetCode::REMOTE_TIMEOUT;
            }
        }
        tickstart = xTaskGetTickCount();
        // *rx_address = (uint8_t)spi_i2s_data_receive(hspi);
        *rx_address = spi_data_receive(hspi);
        rx_address++;
    }
    cs.Set();
    return FuncRetCode::OK;
}

void SPI::SetDataWidth(const DataWidth w)
{
    if(w == DataWidth::BYTE) spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    else spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
}

void SPI::SetClock(const uint32_t clock)
{
    uint32_t spi_base_clock = rcu_clock_freq_get(CK_APB1);
#if defined(GD32G5X3)
    if(hspi == SPI0) spi_base_clock = rcu_clock_freq_get(CK_APB2);
#endif
    const uint16_t divider = spi_base_clock / clock;
    if(divider <= 2) spi_init_struct.prescale = SPI_PSC_2;
    else if(divider <= 4) spi_init_struct.prescale = SPI_PSC_4;
    else if(divider <= 8) spi_init_struct.prescale = SPI_PSC_8;
    else if(divider <= 16) spi_init_struct.prescale = SPI_PSC_16;
    else if(divider <= 32) spi_init_struct.prescale = SPI_PSC_32;
    else if(divider <= 64) spi_init_struct.prescale = SPI_PSC_64;
    else if(divider <= 128) spi_init_struct.prescale = SPI_PSC_128;
    else spi_init_struct.prescale = SPI_PSC_256;
}

void SPI::SetCPOLCPHA(const uint8_t cpol, const uint8_t cpha)
{
    if(cpol == 1)
    {
        if(cpha == 1) spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
        else spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
    }
    else
    {
        if(cpha == 1) spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
        else spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    }
}

void SPI::SetCS(const bool state)
{
    cs = state;
}
}

#endif