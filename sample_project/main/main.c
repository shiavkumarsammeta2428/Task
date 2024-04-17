

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"




#define GPIO_INPUT_IO_0     0   // A GPIO PIN for button 
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define BLINK_GPIO 0
#define ESP_INTR_FLAG_DEFAULT 0

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR         0b1101000                  /*  SLAVE ADDRESS OF ACCELEROMETER*/

#define TEMP_READ_LOW               0x42                       /*LOWER BYTE ADDRESS*/
#define TEMP_READ_HIGH              0x41                       /*UPPER BYTE ADDRESS*/


int cnt = 0;

void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}


/*Interrupt handle function*/
void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if(cnt == 0){

        cnt=1;
    }
    else{
        int var = 1;
        for(int i=0;i<2;i++){
            /* Set the GPIO level according to the state (LOW or HIGH)*/
            gpio_set_level(BLINK_GPIO,var); // LED ON
            var^=1;
            gpio_set_level(BLINK_GPIO,var); // LED OFF
            var^=1;
        }
    }
}


/*GPIO pin configuration  and interrupt*/
static void gpio_intr_init(void){

     //zero-initialize the config structure.
    gpio_config_t io_conf = {};

    //interrupt of any edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);


}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    return i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static void sensor_read_console_write(void* arg)
{
    uint8_t value=0;
    int16_t temp_value=0;
    float temp_degree=0;

    while(1)
    {
        if(cnt == 1){
            for(int i=1;i<=1000;i++)
            {
       
                mpu6050_register_read(TEMP_READ_HIGH,&value,1);
                temp_value |= (((int16_t)value) << 8);
                mpu6050_register_read(TEMP_READ_LOW,&value,1);
                temp_value |= ((int16_t)value);
                
                temp_degree = (float)((temp_value/340)+36.53); // calculation for obtaing temperature in degrees given in datasheet
                printf("%dth value of temperature: %f\n",i,temp_degree);
            }
            
            cnt = 0;
        }
    } 
}



void app_main(void)
{
    gpio_intr_init();
    configure_led();
    i2c_master_init();

    xTaskCreate(sensor_read_console_write, "sensor_read_console_write", 2048, NULL, 10, NULL);

}
