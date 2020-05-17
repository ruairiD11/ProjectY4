/*		Ruairi Doherty
 * 		26/01/2020
 * 		Final Year Project - Remote-Robot
 * 		Glove Module
 *
 * 		References:
 * 		-https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/ --> ESP-IDF API Reference
 * 		-"Kolban's Book on ESP32" - Neil Kolban, 2018
 * 		-https://www.youtube.com/playlist?list=PLB-czhEQLJbWMOl7Ew4QW1LpoUUE7QMOo --> Neil Kolban ESP32 Technical Tutorials
 * 		-https://www.arduino.cc/reference/en/language/functions/math/map/ --> map() function algorithm
 */

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include <esp_log.h>
#include <string.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include <driver/i2c.h>
#include "sdkconfig.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define PIN_SDA	21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define PITCH_FORWARD 4
#define PITCH_BACKWARD 5

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_CONFIG 0x1C

QueueHandle_t dataQueue = NULL;
EventGroupHandle_t pitchEventGroup;
TaskHandle_t flexTask, accelTask = NULL;

uint8_t forward_state, backward_state = 0;

uint8_t robotModuleAddress[] = { 0xb4, 0xe6, 0x2d, 0xe3, 0xd2, 0x6d };
uint8_t esp32CamAddress[] = { 0xC4, 0x4F, 0x33, 0x3A, 0x0C, 0x81 };

void IRAM_ATTR forward_isr_handler(void* arg)
{
	forward_state = 1;
}

void IRAM_ATTR backward_isr_handler(void* arg)
{
	backward_state = 1;
}

void on_data_sent(const uint8_t* mac_addr, esp_now_send_status_t status)
{
	printf("Last Packet Send Status: ");
	if (status == ESP_NOW_SEND_SUCCESS)
		printf("Delivery Success\n");
	else if (status == ESP_NOW_SEND_FAIL)
		printf("Delivery Fail\n");
}

// Arduino map() function
int16_t map(int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void wifi_init()
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_ERROR_CHECK(esp_now_init());
	esp_now_register_send_cb(on_data_sent);

	esp_now_peer_info_t robot_peerInfo;
	memcpy(robot_peerInfo.peer_addr, robotModuleAddress, 6);
	robot_peerInfo.ifidx = WIFI_IF_STA;
	robot_peerInfo.channel = 0;
	robot_peerInfo.encrypt = false;
	if(esp_now_add_peer(&robot_peerInfo) != ESP_OK)
		printf("Failed to add robot peer\n");

	esp_now_peer_info_t cam_peerInfo;
	memcpy(cam_peerInfo.peer_addr, esp32CamAddress, 6);
	cam_peerInfo.ifidx = WIFI_IF_STA;
	cam_peerInfo.channel = 1;
	cam_peerInfo.encrypt = false;
	if(esp_now_add_peer(&cam_peerInfo) != ESP_OK)
		printf("Failed to add cam peer\n");

	pitchEventGroup = xEventGroupCreate();
}

void gpio_interrupt_config()
{
	gpio_config_t io_conf_forward;
	io_conf_forward.intr_type = GPIO_INTR_HIGH_LEVEL;
	io_conf_forward.pin_bit_mask = (1<<PITCH_FORWARD);
	io_conf_forward.mode = GPIO_MODE_INPUT;
	io_conf_forward.pull_up_en = 1;
	io_conf_forward.pull_down_en = 0;
	gpio_config(&io_conf_forward);

	gpio_config_t io_conf_backward;
	io_conf_backward.intr_type = GPIO_INTR_HIGH_LEVEL;
	io_conf_backward.pin_bit_mask = (1<<PITCH_BACKWARD);
	io_conf_backward.mode = GPIO_MODE_INPUT;
	io_conf_backward.pull_up_en = 1;
	io_conf_backward.pull_down_en = 0;
	gpio_config(&io_conf_backward);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PITCH_FORWARD, forward_isr_handler, (void*) PITCH_FORWARD);
	gpio_isr_handler_add(PITCH_BACKWARD, backward_isr_handler, (void*) PITCH_BACKWARD);
}

void data_Tx(void* pvParameters)
{
	uint8_t dataTx[4];
	dataQueue = xQueueCreate(2, sizeof(dataTx));

	for(;;) {
		// Block waiting for sensor readings
		if(xQueueReceive(dataQueue, &dataTx, portMAX_DELAY)) {
			dataTx[2] = forward_state;
			dataTx[3] = backward_state;
			printf("%d,  %d,  %d,  %d\n", dataTx[0], dataTx[1], dataTx[2], dataTx[3]);

			// Transmit data
			ESP_ERROR_CHECK(esp_now_send(robotModuleAddress, dataTx, sizeof(dataTx)));
			ESP_ERROR_CHECK(esp_now_send(esp32CamAddress, dataTx, sizeof(dataTx)));
			forward_state = 0;
			backward_state = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void read_accelerometer(void* pvParameters)
{
	uint8_t data[14];
	uint8_t dataTx[2];
	int16_t accel_y, flex_avg;

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for(;;) {
		// Positioning internal pointer to MPU6050_ACCEL_XOUT_H register.
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
		i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1);

		// Read six accel registers
		i2c_master_read_byte(cmd, data,   0);
		i2c_master_read_byte(cmd, data+1, 0);
		i2c_master_read_byte(cmd, data+2, 0); // only concerned about
		i2c_master_read_byte(cmd, data+3, 0); // middle two bytes for y value
		i2c_master_read_byte(cmd, data+4, 0);
		i2c_master_read_byte(cmd, data+5, 1);

		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);

		//Wait forever for flex average value from flex task
		flex_avg = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		accel_y = (data[2] << 8) | data[3];

		dataTx[0] = map(accel_y, -17000, 17000, 0, 255);
		dataTx[1] = map(flex_avg, 2000, 3500, 0, 255);

		xQueueSend(dataQueue, &dataTx, 0);

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void read_flex(void* pvParameters)
{
	int16_t index = 2500;
	int16_t middle = 2500;
	int16_t third = 2500;
	int16_t old_index, old_middle, old_third;
	int16_t sum, avg;

	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db); //GPIO 36 - THIRD
	adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_11db); //GPIO 39 - MIDDLE
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_11db); //GPIO 34 - INDEX

	for(;;) {
		old_index = index;
		index = adc1_get_raw(ADC1_CHANNEL_6);
		if(index == 4095)
			index = 2500;

		old_middle = middle;
		middle = adc1_get_raw(ADC1_CHANNEL_3);
		if(middle == 4095)
			middle = 2200;

		old_third = third;
		third = adc1_get_raw(ADC1_CHANNEL_0);
		if(third == 4095)
			third = 3100;

		sum = index + middle + third + old_index + old_middle + old_third;
		avg = sum/6;

		avg = map(avg, 2100, 3200, 2000, 3500);

		xTaskNotify(accelTask, avg, eSetValueWithOverwrite);

		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void app_main(void)
{
	nvs_flash_init();
	wifi_init();
	gpio_interrupt_config();
	xTaskCreatePinnedToCore(&data_Tx, "data_Tx", 2048, NULL, 6, NULL, 0);
	xTaskCreatePinnedToCore(&read_accelerometer, "read_accelerometer", 2048, NULL, 5, &accelTask, 1);
	xTaskCreatePinnedToCore(&read_flex, "read_flex", 2048, NULL, 4, &flexTask, 1);
}
