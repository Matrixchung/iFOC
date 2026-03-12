#include "at32wk_spi.hpp"

#if defined(AT32WK_ENV) && defined(SPI_MODULE_ENABLED)

#include "hal_impl.hpp"
#include "foc_math.hpp"

namespace iFOC::HAL
{
SPI::SPI(spi_type* _hspi, GPIOBase& _cs) : hspi(_hspi), cs(_cs)
{
    spi_default_para_init(&spi_init_struct);
};
SPI::SPI(spi_type* _hspi, GPIOBase* _cs) : hspi(_hspi), cs(*_cs)
{
    spi_default_para_init(&spi_init_struct);
};

FuncRetCode SPI::Init()
{
    cs.ModeOutPP();
    cs.Set();
    spi_enable(hspi, FALSE);
    DelayUs(1);
    spi_i2s_reset(hspi);
    DelayUs(1);
    spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
    spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
    spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
    spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
    spi_init(hspi, &spi_init_struct);
    spi_enable(hspi, TRUE);
    return FuncRetCode::OK;
}

FuncRetCode SPI::WriteBytes(const uint8_t* data, const uint16_t size)
{
    uint32_t tickstart = xTaskGetTickCount();
    uint8_t* tx_address = (uint8_t*)data;
    uint16_t len = size;
    constexpr uint32_t TIMEOUT = 10;
    cs.Clear();
    // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
    UNUSED(hspi->dt);
    while(len--)
    {
        while(!(hspi->sts & SPI_I2S_TDBE_FLAG))
        {
            if(xTaskGetTickCount() - tickstart > TIMEOUT)
            {
                // spi_enable(hspi, FALSE);
                cs.Set();
                // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
                UNUSED(hspi->dt);
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        tickstart = xTaskGetTickCount();
        spi_i2s_data_transmit(hspi, (uint8_t)*tx_address);
        tx_address++;
    }
    cs.Set();
    // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
    UNUSED(hspi->dt);
    return FuncRetCode::OK;
}

FuncRetCode SPI::ReadBytes(uint8_t *data, const uint16_t size)
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
    // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
    UNUSED(hspi->dt);
    while(len--)
    {
        constexpr uint32_t TIMEOUT = 10;
        // while(spi_i2s_flag_get(hspi, SPI_I2S_TDBE_FLAG) == RESET)
        while(!(hspi->sts & SPI_I2S_TDBE_FLAG))
        {
            if(xTaskGetTickCount() - tickstart > TIMEOUT)
            {
                cs.Set();
                // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
                UNUSED(hspi->dt);
                return FuncRetCode::HARDWARE_ERROR;
            }
        }
        tickstart = xTaskGetTickCount();
        // spi_i2s_data_transmit(hspi, (uint8_t)*tx_address);
        hspi->dt = *tx_address;
        tx_address++;
        // while(spi_i2s_flag_get(hspi, SPI_I2S_RDBF_FLAG) == RESET)
        while(!(hspi->sts & SPI_I2S_RDBF_FLAG))
        {
            if(xTaskGetTickCount() - tickstart > TIMEOUT)
            {
                cs.Set();
                // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
                UNUSED(hspi->dt);
                return FuncRetCode::REMOTE_TIMEOUT;
            }
        }
        // spi_i2s_flag_clear(hspi, SPI_I2S_RDBF_FLAG);
        UNUSED(hspi->dt);
        tickstart = xTaskGetTickCount();
        // *rx_address = (uint8_t)spi_i2s_data_receive(hspi);
        *rx_address = (uint8_t)hspi->dt;
        rx_address++;
    }
    cs.Set();
    return FuncRetCode::OK;
}

void SPI::SetDataWidth(SPIBase::DataWidth w)
{
    if(w == DataWidth::BYTE) spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
    else spi_init_struct.frame_bit_num = SPI_FRAME_16BIT;
}

void SPI::SetClock(const uint32_t clock)
{
    crm_clocks_freq_type freq;
    crm_clocks_freq_get(&freq);
    uint32_t spi_base_clock = freq.apb2_freq;
#if defined(AT32F403Axx)
    if(hspi == SPI2 || hspi == SPI3 || hspi == SPI4) spi_base_clock = freq.apb1_freq;
#endif
    const uint16_t divider = spi_base_clock / clock;
    if(divider <= 2) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_2;
    else if(divider <= 4) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_4;
    else if(divider <= 8) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
    else if(divider <= 16) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_16;
    else if(divider <= 32) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_32;
    else if(divider <= 64) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_64;
    else if(divider <= 128) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_128;
    else if(divider <= 256) spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_256;
    else spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_512;
}

void SPI::SetCPOLCPHA(const uint8_t cpol, const uint8_t cpha)
{
    if(cpol == 1) spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
    else spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_LOW;
    if(cpha == 1) spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
    else spi_init_struct.clock_phase = SPI_CLOCK_PHASE_1EDGE;
}

void SPI::SetCS(bool state)
{
    cs = state;
}
}

#endif