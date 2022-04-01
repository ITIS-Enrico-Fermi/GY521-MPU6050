#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include "esp_log.h"
#include <driver/i2c.h>
#include <math.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "MAIN"
#define ADDRESS 0x68

#define ACCEL_XOUT_H (0x3B)
#define ACCEL_XOUT_L (0x3C)
#define ACCEL_YOUT_H (0x3D)
#define ACCEL_YOUT_L (0x3E)
#define ACCEL_ZOUT_H (0x3F)
#define ACCEL_ZOUT_L (0x40)
#define GYRO_XOUT_H (0x43)
#define GYRO_XOUT_L (0x44)
#define GYRO_YOUT_H (0x45)
#define GYRO_YOUT_L (0x46)
#define GYRO_ZOUT_H (0x47)
#define GYRO_ZOUT_L (0x48)

int SDA_PIN = GPIO_NUM_21;
int SCL_PIN = GPIO_NUM_22;

i2c_cmd_handle_t i2c_handle;

struct accelerometer_data {
  int16_t x;
  int16_t y;
  int16_t z;
};
struct accelerometer_data accel_data;

struct gyroscope_data {
  int16_t x;
  int16_t y;
  int16_t z;
};
struct gyroscope_data gyro_data;

void i2c_master_init() {
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 400e3
    };
	i2c_param_config(I2C_NUM_1, &i2c_config);
  i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
}

uint8_t get_data(uint8_t reg_addr){
  uint8_t datum = 0;
  esp_err_t espErr;

  i2c_handle = i2c_cmd_link_create();
  i2c_master_start(i2c_handle);
  i2c_master_write_byte(i2c_handle, (ADDRESS<<1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(i2c_handle, reg_addr, true);
  i2c_master_stop(i2c_handle);
  espErr = i2c_master_cmd_begin(I2C_NUM_1, i2c_handle, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(i2c_handle);  

  if (espErr != ESP_OK) {
    ESP_LOGE(TAG, "Error read address %d: %d\n", reg_addr, espErr);
  }

  i2c_handle = i2c_cmd_link_create();
  i2c_master_start(i2c_handle);
  i2c_master_write_byte(i2c_handle, (ADDRESS<<1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(i2c_handle, &datum, I2C_MASTER_NACK);
  i2c_master_stop(i2c_handle);
  espErr = i2c_master_cmd_begin(I2C_NUM_1, i2c_handle, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(i2c_handle);

  if (espErr != ESP_OK) {
    ESP_LOGE(TAG, "Error read address %d: %d\n", reg_addr, espErr);
  }

  return datum;
}

int16_t get_parameter(uint8_t reg_addr1, uint8_t reg_addr2){
  uint8_t msb = 0, lsb = 0;
  int16_t value = 0; 
  msb = get_data(reg_addr1);
  lsb = get_data(reg_addr2);
  int8_t MSB = (int8_t)msb;
  value = ((MSB << 8) | lsb);
  return value;
}

void insert_data(struct accelerometer_data *ac_data, struct gyroscope_data *gy_data){
  //accelerometer data
  ac_data->x = get_parameter(ACCEL_XOUT_H, ACCEL_XOUT_L);
  ac_data->y = get_parameter(ACCEL_YOUT_H, ACCEL_YOUT_L);
  ac_data->z = get_parameter(ACCEL_ZOUT_H, ACCEL_ZOUT_L);

  //gyroscope data
  gy_data->x = get_parameter(GYRO_XOUT_H, GYRO_XOUT_L);
  gy_data->y = get_parameter(GYRO_YOUT_H, GYRO_YOUT_L);
  gy_data->z = get_parameter(GYRO_ZOUT_H, GYRO_ZOUT_L);
}

void app_main(){
    ESP_LOGI(TAG, "Initialization of I2C");
    i2c_master_init();

    while(true){
        insert_data(&accel_data, &gyro_data);

        ESP_LOGI(TAG, "acc x: %d\n", accel_data.x);
        ESP_LOGI(TAG, "acc y: %d\n", accel_data.y);
        ESP_LOGI(TAG, "acc z: %d\n", accel_data.z);

        ESP_LOGI(TAG, "gyr x: %d\n", gyro_data.x);
        ESP_LOGI(TAG, "gyr y: %d\n", gyro_data.y);
        ESP_LOGI(TAG, "gyr z: %d\n", gyro_data.z);

        vTaskDelay(500/portTICK_RATE_MS);
    } 
}