#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "deck_spi.h"
#include "config.h"
#include "cfassert.h"
#define DEBUG_MODULE "DECK_SPI"
#include "debug_cf.h"
#ifdef CONFIG_EXT_FLOW_TESTBOARD 
    #define SPI_SCK_PIN CONFIG_SPI_PIN_MOSI
    #define SPI_MOSI_PIN CONFIG_SPI_PIN_CLK
#else
    #define SPI_SCK_PIN CONFIG_SPI_PIN_CLK
    #define SPI_MOSI_PIN CONFIG_SPI_PIN_MOSI
#endif
#define SPI_MISO_PIN CONFIG_SPI_PIN_MISO
#define DUMMY_BYTE 0xA5
static bool isInit = false;
static SemaphoreHandle_t spiMutex;
static void spiConfigureWithSpeed(uint32_t baudRatePrescaler);
static spi_device_handle_t spi;
void spiBegin(void)
{
    if (isInit) {
        return;
    }
    spiMutex = xSemaphoreCreateMutex();
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    }; 
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_BAUDRATE_2MHZ, 
        .mode = 3,							 
        .spics_io_num = -1,					 
        .queue_size = 8,					 
    };
    spi_host_device_t host_id = SPI2_HOST;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0))
    ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
#else
    int dma_chan = host_id; 
    ret = spi_bus_initialize(host_id, &buscfg, dma_chan);
#endif
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(host_id, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    isInit = true;
}
static void spiConfigureWithSpeed(uint32_t baudRatePrescaler)
{
}
bool spiTest(void)
{
    return isInit;
}
bool spiExchange(size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx)
{
    if (isInit != true) {
        return false;
    }
    if (length == 0) {
        return true;    
    }
    esp_err_t ret;
    if (is_tx == true) {
        static spi_transaction_t t;
        memset(&t, 0, sizeof(t));					
        t.length = length * 8;						
        t.tx_buffer = data_tx;						
        ret = spi_device_polling_transmit(spi, &t); 
        assert(ret == ESP_OK);						
        return true;
    }
    static spi_transaction_t r;
    memset(&r, 0, sizeof(r));
    r.length = length * 8;
    r.flags = SPI_TRANS_USE_RXDATA;
    ret = spi_device_polling_transmit(spi, &r);
    assert(ret == ESP_OK);
    if (r.rxlength > 0) {
        memcpy(data_rx, r.rx_data, length);
    }
    return true;
}
void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    spiConfigureWithSpeed(baudRatePrescaler);
}
void spiEndTransaction()
{
    xSemaphoreGive(spiMutex);
}