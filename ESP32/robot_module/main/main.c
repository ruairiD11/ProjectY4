/*		Ruairi Doherty
 * 		26/01/2020
 * 		Final Year Project - Remote-Robot
 * 		Robot Module
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
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define PITCH_FORWARD (1<<0)
#define PITCH_BACKWARD (1<<1)

#define STEPPER_PIN_1	27
#define STEPPER_PIN_2	14
#define STEPPER_PIN_3	12
#define STEPPER_PIN_4	13

uint8_t gloveModuleAddress[] = { 0x84, 0x0D, 0x8E, 0xE6, 0x7C, 0x44 };

uint8_t forward_step[] = {  1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, 1, 0,
							0, 0, 0, 1 	};

uint8_t backward_step[] = { 0, 0, 0, 1,
							0, 0, 1, 0,
							0, 1, 0, 0,
							1, 0, 0, 0	};

QueueHandle_t dataQueue, dataRxQueue = NULL;
EventGroupHandle_t pitchEventGroup = NULL;

// Arduino map() function
int16_t map(int16_t value, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void on_data_receive(const uint8_t* mac_addr, const uint8_t* data, int len)
{
	// Receiving data as array of unsigned 8 bit integers
	uint8_t dataRx[4];
	memcpy(&dataRx, data, sizeof(dataRx));
	printf("Bytes received from Glove module: %d\n", len);
	xQueueSend(dataRxQueue, &dataRx, 0);
}

void process_data(void *pvParameters)
{
	uint8_t dataRx[4];
	int16_t dataRx_converted[4];

	for(;;) {
		if(xQueueReceive(dataRxQueue, &dataRx, portMAX_DELAY)) {
			// Casting received data to 16 bit signed integers
			dataRx_converted[0] = (int16_t)dataRx[0];
			dataRx_converted[1] = (int16_t)dataRx[1];
			dataRx_converted[2] = (int16_t)dataRx[2];
			dataRx_converted[3] = (int16_t)dataRx[3];

			xQueueSend(dataQueue, &dataRx_converted, 0);
		}
	}
	vTaskDelay(pdMS_TO_TICKS(50));
}

void servo_write(void* pvParameters)
{
	int16_t dataRx[4];
	int16_t rollValue, gripperValue = 0;
	uint8_t forward_state, backward_state = 0;

	/*---Gripper Servo Configuration---*/
	ledc_timer_config_t gripper_timer_conf;
	gripper_timer_conf.duty_resolution = LEDC_TIMER_15_BIT;
	gripper_timer_conf.freq_hz = 50;
	gripper_timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	gripper_timer_conf.timer_num = LEDC_TIMER_0;
	ledc_timer_config(&gripper_timer_conf);
	ledc_channel_config_t gripper_ledc_conf;
	gripper_ledc_conf.channel = LEDC_CHANNEL_0;
	gripper_ledc_conf.duty = gripperValue;
	gripper_ledc_conf.gpio_num = 16;
	gripper_ledc_conf.intr_type = LEDC_INTR_DISABLE;
	gripper_ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	gripper_ledc_conf.timer_sel = LEDC_TIMER_0;
	ledc_channel_config(&gripper_ledc_conf);

	/*---Roll Servo Configuration---*/
	ledc_timer_config_t roll_timer_conf;
	roll_timer_conf.duty_resolution = LEDC_TIMER_15_BIT;
	roll_timer_conf.freq_hz = 50;
	roll_timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	roll_timer_conf.timer_num = LEDC_TIMER_1;
	ledc_timer_config(&roll_timer_conf);
	ledc_channel_config_t roll_ledc_conf;
	roll_ledc_conf.channel = LEDC_CHANNEL_1;
	roll_ledc_conf.duty = gripperValue;
	roll_ledc_conf.gpio_num = 3;
	roll_ledc_conf.intr_type = LEDC_INTR_DISABLE;
	roll_ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
	roll_ledc_conf.timer_sel = LEDC_TIMER_1;
	ledc_channel_config(&roll_ledc_conf);

	for(;;) {
		//Wait forever for incoming data
		if(xQueueReceive(dataQueue, &dataRx, portMAX_DELAY)) {

			rollValue = map(dataRx[0], 0, 200, 1000, 4000);
			gripperValue = map(dataRx[1], 0, 200, 2000, 3500);
			forward_state = dataRx[2];
			backward_state = dataRx[3];

			if(forward_state == 1){
				xEventGroupSetBits(pitchEventGroup, PITCH_FORWARD);
			}
			else if(backward_state == 1)
				xEventGroupSetBits(pitchEventGroup, PITCH_BACKWARD);

			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, gripperValue);
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, rollValue);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void wifi_init()
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_ERROR_CHECK(esp_now_init());
	esp_now_peer_info_t glove_peerInfo;
	memcpy(glove_peerInfo.peer_addr, gloveModuleAddress, 6);
	glove_peerInfo.ifidx = WIFI_IF_STA;
	glove_peerInfo.channel = 0;
	glove_peerInfo.encrypt = false;
	if(esp_now_add_peer(&glove_peerInfo) != ESP_OK)
		printf("Failed to add glove peer\n");

	dataRxQueue = xQueueCreate(4, 4);
	dataQueue = xQueueCreate(4, 8);
	pitchEventGroup = xEventGroupCreate();

	esp_now_register_recv_cb(on_data_receive);
}

void stepper_write(void* pvParameters)
{
	EventBits_t bits;
	uint8_t i, j;
	gpio_pad_select_gpio(STEPPER_PIN_1);
	gpio_set_direction(STEPPER_PIN_1, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(STEPPER_PIN_2);
	gpio_set_direction(STEPPER_PIN_2, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(STEPPER_PIN_3);
	gpio_set_direction(STEPPER_PIN_3, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(STEPPER_PIN_4);
	gpio_set_direction(STEPPER_PIN_4, GPIO_MODE_OUTPUT);

	for(;;) {
		bits = xEventGroupWaitBits(pitchEventGroup, PITCH_FORWARD | PITCH_BACKWARD , pdTRUE, pdFALSE, portMAX_DELAY);
		if(bits & PITCH_FORWARD) {
			for(i = 0; i < 12; i++) {
				for(j = 0; j < 16; j+=4) {
					gpio_set_level(STEPPER_PIN_1, forward_step[j]);
					gpio_set_level(STEPPER_PIN_2, forward_step[j+1]);
					gpio_set_level(STEPPER_PIN_3, forward_step[j+2]);
					gpio_set_level(STEPPER_PIN_4, forward_step[j+3]);
					vTaskDelay(pdMS_TO_TICKS(10));
				}
			}
			// Set stepper pins back to 0
			gpio_set_level(STEPPER_PIN_1, 0);
			gpio_set_level(STEPPER_PIN_2, 0);
			gpio_set_level(STEPPER_PIN_3, 0);
			gpio_set_level(STEPPER_PIN_4, 0);
		}
		else if(bits & PITCH_BACKWARD) {
			for(i = 0; i < 12; i++) {
				for(j = 0; j < 16; j+=4) {
					gpio_set_level(STEPPER_PIN_1, backward_step[j]);
					gpio_set_level(STEPPER_PIN_2, backward_step[j+1]);
					gpio_set_level(STEPPER_PIN_3, backward_step[j+2]);
					gpio_set_level(STEPPER_PIN_4, backward_step[j+3]);
					vTaskDelay(pdMS_TO_TICKS(10));
				}
			}
			// Set stepper pins back to 0
			gpio_set_level(STEPPER_PIN_1, 0);
			gpio_set_level(STEPPER_PIN_2, 0);
			gpio_set_level(STEPPER_PIN_3, 0);
			gpio_set_level(STEPPER_PIN_4, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void app_main(void)
{
	nvs_flash_init();
	wifi_init();
	xTaskCreatePinnedToCore(&servo_write, "servo_write", 2048, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(&stepper_write, "stepper_write", 2048, NULL, 4, NULL, 1);
	xTaskCreatePinnedToCore(&process_data, "process_data", 2048, NULL, 6, NULL, 0);
}
