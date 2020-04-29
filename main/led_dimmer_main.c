// led_dimmer project

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "mqtt_client.h"


#include "driver/gpio.h"
#include "driver/pwm.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "esp8266/gpio_register.h"
#include "esp8266/pin_mux_register.h"



#define ESP_WIFI_SSID		CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS		CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY	CONFIG_ESP_MAXIMUM_RETRY



static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "led_dimmer";
static const char *TAG_MQTT = "MQTT_CLI";
static const char *TAG_WIFI = "wifi station";

static int s_retry_num = 0;



#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT	   BIT1

#define PWM_CH0		   12
#define PWM_CH1		   13

#define BUTTON		   5
#define GPIO_INPUT_PIN_SEL ((1ULL<<BUTTON))

xQueueHandle gpio_evt_queue = NULL;
xQueueHandle mqtt_dim0_queue = NULL;
xQueueHandle mqtt_dim1_queue = NULL;

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

//MQTT topic subscribe
const char topic_dim_ch0_set[] = "/home/kitchen/light/dimmer/ch0/set";
const char topic_dim_ch1_set[] = "/home/kitchen/light/dimmer/ch1/set";

uint32_t duty;

 


static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
		if (s_retry_num < ESP_MAXIMUM_RETRY) {
            		esp_wifi_connect();
            		s_retry_num++;
            		ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        	}
		else {
            		xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        	}
        	ESP_LOGI(TAG_WIFI,"connect to the AP fail");
    	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        	ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        	ESP_LOGI(TAG_WIFI, "got ip:%s",
                ip4addr_ntoa(&event->ip_info.ip));
        	s_retry_num = 0;
        	xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    	}
}


void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}




static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	uint32_t xData;
	char duty[4];
	char *duty_cycle;
	duty_cycle = &duty[0];
	// your_context_t *context = event->context;
	switch (event->event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");

			msg_id = esp_mqtt_client_subscribe(client, topic_dim_ch0_set, 0);
			ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);

			msg_id = esp_mqtt_client_subscribe(client, topic_dim_ch1_set, 0);
			ESP_LOGI(TAG_MQTT, "sent subscribe successful, msg_id=%d", msg_id);
			break;

		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
			break;

		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			msg_id = esp_mqtt_client_publish(client, "/home/kitchen/light/dimmer/ch0/status", "500", 0, 0, 0);
			ESP_LOGI(TAG_MQTT, "sent publish successful, msg_id=%d", msg_id);
			break;

		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;

		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;

		case MQTT_EVENT_DATA:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DATA");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);

			if(strncmp(topic_dim_ch0_set, event->topic, event->topic_len)==0){
				ESP_LOGI(TAG_MQTT, "channel0");
				strncpy(duty_cycle, event->data, event->data_len);
				xData = atoi(duty_cycle);
				xQueueSendToBack(mqtt_dim0_queue, &xData, 100/portTICK_RATE_MS);		
			}
			else if(strncmp(topic_dim_ch1_set, event->topic, event->topic_len)==0){
				ESP_LOGI(TAG_MQTT, "channel1");
				strncpy(duty_cycle, event->data, event->data_len);
				xData = atoi(duty_cycle);
				xQueueSendToBack(mqtt_dim1_queue, &xData, 100/portTICK_RATE_MS);
			}
			else{
				ESP_LOGI(TAG_MQTT, "topic strings not equal");
			}	
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
			break;
			
	}
	return ESP_OK;
}



static void gpio_isr_handler(void *arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	gpio_set_intr_type(BUTTON, GPIO_INTR_DISABLE);
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, 0);
	
}


static void pwm_task(void *arg)
{
	uint8_t channel;
	channel=(uint8_t)arg;	
	uint32_t io_num;
	uint32_t xReceivedData;
	
	
	uint8_t state = 0;

	ESP_LOGI(TAG, "pwm task, channel = %d\n", channel);
	

	for (;;)
	{
		switch(state)
		{
			case 0:
				if(channel == 0){
					//ESP_LOGI(TAG, "ch0 - state = %d", state);
					if(xQueueReceive(gpio_evt_queue, &io_num, 100/portTICK_RATE_MS) && io_num == 5) 
					{
						state = 2;
						ESP_LOGI(TAG, "ch0 - Button pressed - GPIO[%d] intr, state = %d", io_num, state);
						io_num = 0;
					}
					else if(xQueueReceive(mqtt_dim0_queue, &xReceivedData, 100/portTICK_RATE_MS))
					{
						ESP_LOGI(TAG, "ch0 - recv duty cycle = %d, state = %d", xReceivedData, state);
						if(xReceivedData>4)
						{
							state = 1;
							pwm_set_duty(0, xReceivedData);
							pwm_start();
						}
						else if(xReceivedData<4)
						{
							state = 0;
							pwm_set_duty(0, xReceivedData);
							pwm_start();
						}
					}
				}
				else if(channel == 1){
					//ESP_LOGI(TAG, "ch1 - state = %d", state);
					if(xQueueReceive(mqtt_dim1_queue, &xReceivedData, 100/portTICK_RATE_MS)){	
						ESP_LOGI(TAG, "ch1 - recv duty cycle = %d, state = %d", xReceivedData, state);
						if(xReceivedData>4)
						{
							state = 1;
							pwm_set_duty(1, xReceivedData);
							pwm_start();
						}
						else if(xReceivedData<4)
						{
							state = 0;
							pwm_set_duty(1, xReceivedData);
							pwm_start();
						}
					}
				}
				break;

			case 2:
				if(channel == 0){
					ESP_LOGI(TAG, "ch0 - state = %d", state);	
					for(int32_t i=1; i<=1000; i=i+5)
					{
						pwm_set_duty(0, i);
						pwm_start();
						vTaskDelay(10/portTICK_RATE_MS);
					}
					state = 1;
					gpio_set_intr_type(BUTTON, GPIO_INTR_POSEDGE);
				}
				break;

			case 1:
				if(channel == 0){	
					//ESP_LOGI(TAG, "ch0 - state = %d", state);

					if(xQueueReceive(gpio_evt_queue, &io_num, 100/portTICK_RATE_MS) && io_num == 5)
					{
						state = 3;
						ESP_LOGI(TAG, "ch0 - Button pressed - GPIO[%d] intr, state = %d", io_num, state);
						io_num = 0;
					}				
					else if(xQueueReceive(mqtt_dim0_queue, &xReceivedData, 100/portTICK_RATE_MS))
					{
						ESP_LOGI(TAG, "ch0 - recv duty cycle = %d, state = %d", xReceivedData, state);
						if(xReceivedData<=4)
						{
							state = 0;
							pwm_set_duty(0, xReceivedData);
							pwm_start();
						}
						else if(xReceivedData>4)
						{
							state = 1;
							pwm_set_duty(0, xReceivedData);
							pwm_start();
						}
					}
				}
				if(channel == 1){
					//ESP_LOGI(TAG, "ch1 - state = %d", state);
					if(xQueueReceive(mqtt_dim1_queue, &xReceivedData, 100/portTICK_RATE_MS)){
						ESP_LOGI(TAG, "ch1 - recv duty cycle = %d, state = %d", xReceivedData, state);
						if(xReceivedData<=4)
						{
							state = 0;
							pwm_set_duty(1, xReceivedData);
							pwm_start();
						}
						else if(xReceivedData>4)
						{
							state = 1;
							pwm_set_duty(1, xReceivedData);
							pwm_start();
						}
					}
				}
				break;

			case 3:
				if(channel == 0){
					ESP_LOGI(TAG, "ch0 - state = %d", state);
					for(int32_t k=1000; k>=0; k=k-5)
					{
						pwm_set_duty(0, k);
						pwm_start();
						vTaskDelay(10/portTICK_RATE_MS);
					}
					state = 0;
					gpio_set_intr_type(BUTTON, GPIO_INTR_POSEDGE);
				}
				break;
		}
	}
}



static void mqtt_app_start(void)
{
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
		.event_handle = mqtt_event_handler,
		//.user_context = (void *)your_context
	};

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_start(client);
}



void app_main()
{
	

    	ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	//ESP_ERROR_CHECK(esp_event_loop_create_default());
	wifi_init_sta();


	//ESP_ERROR_CHECK(example_connect());

	ESP_LOGI(TAG_MQTT, "[APP] Startup ..");
	ESP_LOGI(TAG_MQTT, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	ESP_LOGI(TAG_MQTT, "[APP] IDF version: %s", esp_get_idf_version());

	esp_log_level_set("*", ESP_LOG_INFO);
	esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
	esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
	esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);



	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE; //interrupt of rising edge
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	io_conf.mode = GPIO_MODE_INPUT;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	mqtt_dim0_queue = xQueueCreate(5, sizeof(uint32_t));
	mqtt_dim1_queue = xQueueCreate(5, sizeof(uint32_t));
	
	pwm_init(PWM_PERIOD, duties, 2, pin_num);
	pwm_set_phases(phase);
	
	xTaskCreate(pwm_task, "pwm task for ch0", 2048, (void *) 0, 10, NULL);
	xTaskCreate(pwm_task, "pwm task for ch1", 2048, (void *) 1, 10, NULL);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUTTON, gpio_isr_handler, (void *) BUTTON);
	
	mqtt_app_start();

    while (1) {
        vTaskDelay(1000 / portTICK_RATE_MS);
   }
}
