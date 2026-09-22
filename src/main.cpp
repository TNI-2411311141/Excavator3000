#define CORE_DEBUG_LEVEL ARDUHAL_LOG_LEVEL_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define MCU_STATUS_PIN GPIO_NUM_2
#define MO_EN_PIN GPIO_NUM_14
#define MO_EN_ACTIVE LOW
#define MO_EN_INACTIVE HIGH
#define COLLISION_TRIGGER_PIN GPIO_NUM_35
#define COLLISION_STATUS_PIN GPIO_NUM_18
#define COLLISION_PROTECTION_BYPASS_BTN_PIN GPIO_NUM_13
#define COLLISION_PROTECTION_BYPASS_STATUS_PIN GPIO_NUM_15

#define WIFI_STA_SSID "pk-mtn-nx1"
#define WIFI_STA_PASS "9gpnnhahm4qbgyp"

#define MQTT_SERVER_IP "10.138.55.143"
#define MQTT_PORT 1883

DHT dht(GPIO_NUM_4, DHT11);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
WiFiClient espClient;
PubSubClient mqtt(espClient);

struct guarded_float {
	float value;
	SemaphoreHandle_t lock;
};

struct guarded_float dht11_temperature = {.value = 0, .lock = NULL};
struct guarded_float dht11_humidity = {.value = 0, .lock = NULL};

volatile bool collision_protection_bypass = 0;

struct mqtt_message {
	const char *topic;
	const char *message;
};

xQueueHandle mqtt_message_queue = xQueueCreate(10, sizeof(struct mqtt_message));
TaskHandle_t mqtt_sender_handler = NULL;
void mqtt_sender(void *) {
	struct mqtt_message msg;
	while (1) {
		if (xQueueReceive(mqtt_message_queue, &msg, portMAX_DELAY) !=
		    pdTRUE) {
			ESP_LOGW("mqtt_sender",
			         "timed out reached, no message received");
			continue;
		}
		ESP_LOGV("mqtt_sender", "received %s %s", msg.topic,
		         msg.message);
		if (mqtt.connected()) {
			if (mqtt.publish(msg.topic, msg.message))
				ESP_LOGI("mqtt_sender", "published %s %s",
				         msg.topic, msg.message);
			else
				ESP_LOGW("mqtt_sender", "cannot publish %s %s",
				         msg.topic, msg.message);
		} else
			ESP_LOGW("mqtt_sender",
			         "mqtt not connected, cannot publish %s %s",
			         msg.topic, msg.message);
	}
}

TaskHandle_t collision_check_routine_handler = NULL;
void collision_check_routine(void *) {
	static int is_free_prev = false;
	struct mqtt_message msg = {.topic = "esp32/collision"};
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(20));
		int is_free = digitalRead(COLLISION_TRIGGER_PIN);
		ESP_LOGV("collision_check_routine", "Read value: %hd", is_free);
		if (is_free == is_free_prev)
			continue;
		is_free_prev = is_free;
		ESP_LOGI("collision_check_routine", "%s",
		         is_free ? "free" : "collide");
		digitalWrite(MO_EN_PIN, !is_free);
		digitalWrite(COLLISION_STATUS_PIN, !is_free);
		msg.message = is_free ? "free" : "collide";
		xQueueSend(mqtt_message_queue, &msg, 0);
	}
}

void led_blink(void *) {
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(3000));
		digitalWrite(MCU_STATUS_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(20));
		digitalWrite(MCU_STATUS_PIN, 0);
	}
}

TaskHandle_t display_status_routine_handler = NULL;
void display_status_routine(void *) {
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		ESP_LOGD("display_status_routine", "Updating display status");
		display.clearDisplay();
		display.setTextColor(SSD1306_WHITE);
		display.setTextSize(1);
		display.setCursor(0, 0);
		xSemaphoreTake(dht11_temperature.lock, portMAX_DELAY);
		display.printf("Temp %3.2f C\n", dht11_temperature.value);
		xSemaphoreGive(dht11_temperature.lock);
		xSemaphoreTake(dht11_humidity.lock, portMAX_DELAY);
		display.printf("Humid %3.2f %%\n", dht11_humidity.value);
		xSemaphoreGive(dht11_humidity.lock);
		display.printf("MAC %s\n", WiFi.macAddress().c_str());
		display.printf("IP %s\n", WiFi.localIP().toString().c_str());
		display.printf("MQTT %s\n", mqtt.connected() ? "Connected"
		                                             : "Not connected");
		// display.printf("IPv6: %s\n",
		//                WiFi.localIPv6().toString().c_str());
		display.display();
	}
}

TaskHandle_t mqtt_report_routine_handler = NULL;
void mqtt_report_routine(void *) {
	char strbuf_temp[7];
	char strbuf_humid[7];
	struct mqtt_message msg;
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		xSemaphoreTake(dht11_temperature.lock, portMAX_DELAY);
		snprintf(strbuf_temp, sizeof(strbuf_temp), "%.2f",
		         dht11_temperature.value);
		xSemaphoreGive(dht11_temperature.lock);
		msg = (struct mqtt_message){.topic = "esp32/temperature",
		                            .message = strbuf_temp};
		xQueueSend(mqtt_message_queue, &msg, 0);

		xSemaphoreTake(dht11_humidity.lock, portMAX_DELAY);
		snprintf(strbuf_humid, sizeof(strbuf_humid), "%.2f",
		         dht11_humidity.value);
		xSemaphoreGive(dht11_humidity.lock);
		msg = (struct mqtt_message){.topic = "esp32/humidity",
		                            .message = strbuf_humid};
		xQueueSend(mqtt_message_queue, &msg, 0);
	}
}

TaskHandle_t mqtt_autoconnect_routine_handler = NULL;
void mqtt_autoconnect_routine(void *) {
	char client_id[32];
	struct mqtt_message msg = {.topic = "esp32/status",
	                           .message = "online"};
	snprintf(client_id, sizeof(client_id), "%llX", ESP.getEfuseMac());
	mqtt.setServer(MQTT_SERVER_IP, MQTT_PORT);
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(3000));
		if (mqtt.connected()) {
			mqtt.loop();
			xQueueSend(mqtt_message_queue, &msg, 0);
			continue;
		}
		if (!WiFi.isConnected()) {
			ESP_LOGW("mqtt_autoconnect_routine",
			         "Cannot connect to mqtt server, WiFi is not "
			         "connected");
			continue;
		}
		if (mqtt.connect(client_id)) {
			ESP_LOGI("mqtt_autoconnect_routine",
			         "Connected to mqtt server");
			vTaskResume(mqtt_report_routine_handler);
		} else {
			ESP_LOGW("mqtt_autoconnect_routine",
			         "Cannot connect to mqtt server, retrying");
			vTaskSuspend(mqtt_report_routine_handler);
		}
	}
}

TaskHandle_t dht11_update_routine_handler = NULL;
void dht11_update_routine(void *) {
	float read_buf;

	dht11_temperature.lock = xSemaphoreCreateMutex();
	dht11_humidity.lock = xSemaphoreCreateMutex();
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(3000));

		read_buf = dht.readTemperature();
		ESP_LOGV("dht11_update_routine", "Temperature: %.2f", read_buf);
		xSemaphoreTake(dht11_temperature.lock, portMAX_DELAY);
		dht11_temperature.value = read_buf;
		xSemaphoreGive(dht11_temperature.lock);

		read_buf = dht.readHumidity();
		ESP_LOGV("dht11_update_routine", "Humidity: %.2f", read_buf);
		xSemaphoreTake(dht11_humidity.lock, portMAX_DELAY);
		dht11_humidity.value = read_buf;
		xSemaphoreGive(dht11_humidity.lock);

		xTaskNotifyGive(mqtt_report_routine_handler);
		xTaskNotifyGive(display_status_routine_handler);
	}
}

IRAM_ATTR void collision_protection_bypass_trig_handler(void) {
	UBaseType_t collision_protection_bypass_trig_lock =
	    taskENTER_CRITICAL_FROM_ISR();
	static unsigned long last_trig = 0;
	bool is_free;
	unsigned long currtime = millis();
	struct mqtt_message msg = {.topic = "esp32/collision"};
	if (currtime - last_trig > 1000) {
		last_trig = currtime;
		collision_protection_bypass = !collision_protection_bypass;
		if (collision_protection_bypass) {
			vTaskSuspend(collision_check_routine_handler);
			digitalWrite(MO_EN_PIN, MO_EN_ACTIVE);
			digitalWrite(COLLISION_STATUS_PIN, LOW);
			digitalWrite(COLLISION_PROTECTION_BYPASS_STATUS_PIN,
			             HIGH);
			ESP_EARLY_LOGI(
			    "collision_protection_bypass_trig_handler",
			    "disable collision check");
			msg.message = "bypass";
			xQueueSendFromISR(mqtt_message_queue, &msg, NULL);
		} else {
			is_free = digitalRead(COLLISION_TRIGGER_PIN);
			digitalWrite(MO_EN_PIN, !is_free);
			digitalWrite(COLLISION_STATUS_PIN, !is_free);
			digitalWrite(COLLISION_PROTECTION_BYPASS_STATUS_PIN,
			             LOW);
			vTaskResume(collision_check_routine_handler);
			ESP_EARLY_LOGI(
			    "collision_protection_bypass_trig_handler",
			    "enable collision check");
			msg.message = is_free ? "free" : "collide";
			xQueueSendFromISR(mqtt_message_queue, &msg, NULL);
		}
	}
	taskEXIT_CRITICAL_FROM_ISR(collision_protection_bypass_trig_lock);
}

void setup(void) {
	WiFi.mode(WIFI_STA);
	WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
	WiFi.setAutoConnect(true);
	WiFi.setAutoReconnect(true);

	pinMode(MCU_STATUS_PIN, OUTPUT);
	pinMode(MO_EN_PIN, OUTPUT);
	pinMode(COLLISION_TRIGGER_PIN, INPUT);
	pinMode(COLLISION_STATUS_PIN, OUTPUT);
	pinMode(COLLISION_PROTECTION_BYPASS_BTN_PIN, INPUT_PULLDOWN);
	pinMode(COLLISION_PROTECTION_BYPASS_STATUS_PIN, OUTPUT);

	dht.begin();
	Wire.begin(GPIO_NUM_21, GPIO_NUM_22);

	while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
		ESP_LOGW("main", "Initialize SSD1306 Failed, retrying");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	display.setTextSize(1);
	display.setCursor(0, 0);

	dht11_temperature.lock = xSemaphoreCreateMutex();
	dht11_humidity.lock = xSemaphoreCreateMutex();

	xTaskCreate(&led_blink, "led_blink", 1024, NULL, 0, NULL);
	xTaskCreate(&mqtt_autoconnect_routine, "mqtt_autoconnect_routine", 2048,
	            NULL, 0, &mqtt_autoconnect_routine_handler);
	xTaskCreate(&mqtt_report_routine, "mqtt_report_routine", 8192, NULL, 0,
	            &mqtt_report_routine_handler);
	xTaskCreate(&collision_check_routine, "collision_check_routine", 4096,
	            NULL, 0, &collision_check_routine_handler);
	xTaskCreate(&dht11_update_routine, "dht11_update_routine", 4096, NULL,
	            0, &dht11_update_routine_handler);
	xTaskCreate(&display_status_routine, "display_status_routine", 4096,
	            NULL, 0, &display_status_routine_handler);
	xTaskCreate(&mqtt_sender, "mqtt_sender", 4096, NULL, 0,
	            &mqtt_sender_handler);

	attachInterrupt(COLLISION_PROTECTION_BYPASS_BTN_PIN,
	                &collision_protection_bypass_trig_handler, RISING);
}

void loop(void) { vTaskDelete(NULL); }
