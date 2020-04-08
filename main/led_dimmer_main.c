

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"

#include "driver/pwm.h"


static const char *TAG = "led_dimmer";


#define PWM_CH0		   12
#define PWM_CH1		   13

#define BUTTON		   5
#define GPIO_INPUT_PIN_SEL ((1ULL<<BUTTON))

xQueueHandle gpio_evt_queue = NULL;
xQueueHandle mqtt_evt_queue = NULL;

// PWM period 1000us(1Khz), same as depth
#define PWM_PERIOD    (1000)


// pwm pin number
const uint32_t pin_num[2] = {
    PWM_CH0,
    PWM_CH1,
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[2] = {
    0, 0,
};

// phase table, delay = (phase[x]/360)*PERIOD
int16_t phase[2] = {
    0, 0, 
};

static void gpio_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	gpio_set_intr_type(BUTTON, GPIO_INTR_DISABLE);
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, 0);
	//gpio_set_intr_type(BUTTON, GPIO_INTR_DISABLE);
}


static void pwm_task(void *arg)
{
	//gpio_set_intr_type(BUTTON, GPIO_INTR_DISABLE);
	uint32_t io_num;
	uint32_t mqtt_message;
	uint8_t state = 0;
	pwm_init(PWM_PERIOD, duties, 2, pin_num);
	pwm_set_phases(phase);
	ESP_LOGI(TAG, "in pwm task\n");

	for (;;)
	{
		//if the button pressed
		if (xQueueReceive(gpio_evt_queue, &io_num, 100/portTICK_RATE_MS)){
			ESP_LOGI(TAG, "GPIO[%d] intr\n", io_num);

			//switch on
			if (!state){

				for(uint32_t i=1; i<=1000; i = i+5)
				{
					pwm_set_duty(0, i);
					pwm_start();
					vTaskDelay(10/portTICK_RATE_MS);
				}
				state = 1;
			}
			//switch off
			else{
				for(uint32_t k=1000; k>=1; k = k-5)
				{
					pwm_set_duty(0, k);
					pwm_start();
					vTaskDelay(10/portTICK_RATE_MS);

				}
				pwm_stop(0x0);
				state = 0;
			}
			gpio_set_intr_type(BUTTON, GPIO_INTR_POSEDGE);
		}

		//if receive the message from MQTT
		if (xQueueReceive(mqtt_evt_queue, &mqtt_message, 100/portTICK_RATE_MS)){

		}
	}
}	





void app_main()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE; //interrupt of rising edge
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	mqtt_evt_queue = xQueueCreate(5, sizeof(uint32_t));

	xTaskCreate(pwm_task, "pwm task", 2048, NULL, 10, NULL);
	
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUTTON, gpio_isr_handler, (void *) BUTTON);

    while (1) {

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

