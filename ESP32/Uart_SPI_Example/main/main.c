#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h"

#include "esp_log.h"

static const int BUF_SIZE = 512;
static QueueHandle_t uart0_queue;

#define TXD_PIN 	GPIO_NUM_32
#define RXD_PIN 	GPIO_NUM_34
#define UART_NUM	UART_NUM_1

#define SPI_DMA_CHAN	1
#define MOSI_PIN 		GPIO_NUM_19
#define MISO_PIN 		GPIO_NUM_18
#define SCLK_PIN 		GPIO_NUM_5
#define CS_PIN 			GPIO_NUM_21

static void initUart(void);
static void initSPI(void);
static void uart_event_task(void *pvParameters);
static void spi_task(void *pvParameters);

/**
 * @brief
 *
 */
void app_main(void)
{
	initUart();
	initSPI();
}


/**
 * @brief
 *
 */
static void initUart(void)
{
	const char *TAG = "init_uart";

	ESP_LOGI(TAG, "START");
    const uart_config_t uart_config = {
        .baud_rate 	= 115200,
        .data_bits 	= UART_DATA_8_BITS,
        .parity 	= UART_PARITY_DISABLE,
        .stop_bits 	= UART_STOP_BITS_1,
        .flow_ctrl 	= UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "END");
}

/**
 * @brief
 *
 */
static void initSPI(void)
{
	const char *TAG = "init_SPI";

	ESP_LOGI(TAG, "START");
    spi_bus_config_t bus_config={
        .mosi_io_num = MOSI_PIN,
        .miso_io_num = MISO_PIN,
        .sclk_io_num = SCLK_PIN
    };

    spi_slave_interface_config_t interface_config={
        .mode			= 0,
        .spics_io_num	= CS_PIN,
        .queue_size		= 3,
        .flags			= 0,
    };

    gpio_set_pull_mode(MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SCLK_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(CS_PIN, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(spi_slave_initialize(VSPI_HOST, &bus_config, &interface_config, SPI_DMA_CHAN));

    xTaskCreate(spi_task, "spi_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "END");
}

/**
 * @brief
 *
 * @param pvParameters
 */
static void uart_event_task(void *pvParameters)
{
	const char *TAG = "uart_events";

    uart_event_t event;
    uint8_t *pUartRxBuffer = (uint8_t*) malloc(BUF_SIZE);

    for(;;) {

        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {

            bzero(pUartRxBuffer, BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);

            switch(event.type) {

                case UART_DATA:
                {
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM, pUartRxBuffer, event.size, portMAX_DELAY);

                    printf("Data : %s \n", pUartRxBuffer);

                    char *pUartTxBuffer = "Uart message from esp32 to stm32...";
                    uart_write_bytes(UART_NUM, pUartTxBuffer, BUF_SIZE);

                }
                break;

                case UART_FIFO_OVF:
                {
                    ESP_LOGI(TAG, "hw fifo overflow");
                    xQueueReset(uart0_queue);
                }
                break;

                case UART_BUFFER_FULL:
                {
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart0_queue);
                }
                break;

                case UART_BREAK:
                {
                    ESP_LOGI(TAG, "uart rx break");
                }
                break;

                case UART_PARITY_ERR:
                {
                    ESP_LOGI(TAG, "uart parity error");
                }
                break;

                case UART_FRAME_ERR:
                {
                    ESP_LOGI(TAG, "uart frame error");
                }
                break;

                default:
                {
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                }
                break;
            }

        }

    }

    free(pUartRxBuffer);
    pUartRxBuffer = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief
 *
 * @param pvParameters
 */
static void spi_task(void *pvParameters)
{
	const char *TAG = "spi_task";

	ESP_LOGI(TAG, "start...");

    WORD_ALIGNED_ATTR char spiTxBuffer[] = "spi message from esp32 to stm32...";
    WORD_ALIGNED_ATTR char spiRxBuffer[34]="";


    spi_slave_transaction_t transsaction;
    memset(&transsaction, 0, sizeof(transsaction));


	for (;;) {

        memset(spiRxBuffer, 0, 34);

        transsaction.length		= 34 * 8;
        transsaction.tx_buffer	= spiTxBuffer;
        transsaction.rx_buffer	= spiRxBuffer;

        spi_slave_transmit(VSPI_HOST, &transsaction, portMAX_DELAY);

        printf("Received: %s \n" , spiRxBuffer);

	}

}




