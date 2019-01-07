/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>
#include <stdlib.h>


#define console_TXD  (UART_PIN_NO_CHANGE)
#define console_RXD  (UART_PIN_NO_CHANGE)
#define lidar_TXD      17
#define lidar_RXD      16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)
#define TEST_PIN       13

#define RMT_RX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_CARRIER_EN    0  /*!< Enable carrier for IR transmitter test with IR led */


#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  27     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  33    /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  100    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define BLINK_GPIO_0 26


#define BUF_SIZE (1024)
const int uart_num = UART_NUM_2;
const int uart_num_c = UART_NUM_0;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void lidar()
{
    

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    //int len = uart_read_bytes(uart_num, data, BUF_SIZE, 100/ portTICK_RATE_MS);
    uint8_t lidar_data[9];
    char* distp = (char *) malloc(6);
   
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, 20/ portTICK_RATE_MS);
        if (len > 0) {
            if((*data == 0x59) & (data[1] == 0x59)){
                for (int i = 0; i < 9; i++) {
                    lidar_data[i] = data[i];
                }
            } 
            else
                data = data +1;         
        }
        else 
            printf("no data \n");

        int low_dist = lidar_data[2];
        int high_dist = lidar_data[3];
        high_dist = high_dist << 8;
        float dist = low_dist + high_dist;
        dist = dist/100;
        /*
        if(dist< ){
            sprintf(distp,"%02.2f\r\n", dist);
        }
        else{
            sprintf(distp,"%d\r\n", dist);
        }*/
        sprintf(distp,"%02.2f\r\n", dist);
        uart_write_bytes(uart_num_c, distp, 6);
        uart_flush(uart_num);
    
}
static inline void nec_fill_item_level(rmt_item32_t* item, int high_us, int low_us)
{
    item->level0 = 1;
    item->duration0 = (high_us) / 10 * RMT_TICK_10_US;
    item->level1 = 0;
    item->duration1 = (low_us) / 10 * RMT_TICK_10_US;
}
/*
 * @brief RMT transmitter initialization
 */
static void nec_tx_init()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx.tx_config.idle_level = 0;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}


/*
 * @brief RMT receiver initialization
 */
static void nec_rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}
    
    static void check_efuse()
{  
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void ir_init(){
    check_efuse();

if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

}

static void ir(){

    //Continuously sample ADC1
    char* distp = (char *) malloc(6);
    
        //gpio set level
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float voltage = mV/1000.0;
        float power = powf(voltage,-1.18);
        float meters = (60.3*power); 
        meters = meters/100.0;
        sprintf(distp,"%02.2f\r\n", meters);
        uart_write_bytes(uart_num_c, distp, 6);

}
static void trigger(){
/**
 * @brief RMT transmitter demo, this task will periodically send NEC data. (100 * 32 bits each time.)
 *
 */
    nec_tx_init();
    int channel_rx = RMT_RX_CHANNEL;
    nec_rx_init();
    //esp_log_level_set(NEC_TAG, ESP_LOG_INFO);
    float dist;
    RingbufHandle_t rb = NULL;
    int channel_tx = RMT_TX_CHANNEL;
    int nec_tx_num = RMT_TX_DATA_NUM;
    size_t size = (sizeof(rmt_item32_t) * NEC_DATA_ITEM_NUM * nec_tx_num);
    int time = 10;
    rmt_item32_t* item_tx = (rmt_item32_t*) malloc(size);
    int item_num = NEC_DATA_ITEM_NUM * nec_tx_num;
       nec_fill_item_level(item_tx, time, time);
       uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
       char* distp = (char *) malloc(6);


    
    rmt_write_items(channel_tx, item_tx, item_num, false);

    vTaskDelay(10/ portTICK_RATE_MS);
    
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel_rx, &rb);
    rmt_rx_start(channel_rx, 1);
    if(rb){
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item)
        {
            vTaskDelay(10/ portTICK_RATE_MS);
            if(item->level0 == 1){
                dist = item->duration0 / 5800.0 ;
                sprintf(distp,"%2.2f\r\n", dist);
                uart_write_bytes(uart_num_c, distp, 6);
            }
            else if(item->level1 == 1){
                dist = item->duration1 / 5800.0;
                sprintf(distp,"%2.2f\r\n", dist);
                uart_write_bytes(uart_num_c, distp, 6);
            }
            //vRingbufferReturnItem(rb, (void*) item);
        }
            //uart_write_bytes(uart_num_c, distp, 6);
    //free(item);
    }

    rmt_rx_stop(channel_rx);
    rmt_memory_rw_rst(channel_rx);
    rmt_driver_uninstall(channel_tx);
    rmt_driver_uninstall(channel_rx);
    free(item_tx);
    
    

}


void app_main()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
        
    };
    uart_param_config(uart_num, &uart_config);
    uart_param_config(uart_num_c, &uart_config);
    uart_set_pin(uart_num, lidar_TXD, lidar_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_set_pin(uart_num_c, console_TXD, console_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(uart_num_c, BUF_SIZE * 2, 0, 0, NULL, 0);
    ir_init();
    while(1){
    lidar();
    vTaskDelay(100/portTICK_RATE_MS);
    trigger();
    vTaskDelay(100/portTICK_RATE_MS);
    ir();
    //vTaskDelay(300/portTICK_RATE_MS);
    
    vTaskDelay(700/portTICK_RATE_MS);
    }

    

}
