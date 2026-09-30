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
#define MO_EN_PIN GPIO_NUM_25
#define MO_EN_ACTIVE HIGH
#define MO_EN_INACTIVE LOW
#define COLLISION_TRIGGER_PIN GPIO_NUM_33
#define COLLISION_STATUS_PIN GPIO_NUM_18
#define COLLISION_PROTECTION_BYPASS_BTN_PIN GPIO_NUM_32
#define COLLISION_PROTECTION_BYPASS_STATUS_PIN GPIO_NUM_15

#define WIFI_STA_SSID "pk-mtn-nx1"
#define WIFI_STA_PASS "9gpnnhahm4qbgyp"

#define MQTT_SERVER_IP "10.138.55.143"
#define MQTT_PORT 1883

DHT dht(GPIO_NUM_4, DHT11);
Adafruit_SSD1306 screen(128, 64, &Wire, -1);
WiFiClient espClient;
PubSubClient mqtt(espClient);

struct guarded_float {
	float value;
	SemaphoreHandle_t lock;
};

struct guarded_float weather_temperature = {.value = 0, .lock = NULL};
struct guarded_float weather_humidity = {.value = 0, .lock = NULL};

volatile bool collision_protection_bypass = 0;

// === Node-RED Remote Bypass Control ===
enum bypass_command : uint8_t { BYPASS_OFF, BYPASS_ON, BYPASS_TOGGLE };
QueueHandle_t bypass_command_queue = NULL;
SemaphoreHandle_t mqtt_lock = NULL;
portMUX_TYPE bypass_state_lock = portMUX_INITIALIZER_UNLOCKED;

bool get_collision_bypass() {
	portENTER_CRITICAL(&bypass_state_lock);
	bool bypass = collision_protection_bypass;
	portEXIT_CRITICAL(&bypass_state_lock);
	return bypass;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
	if (strcmp(topic, "esp32/bypass/set") != 0 || length != 1 ||
	    (payload[0] != '0' && payload[0] != '1'))
		return;
	// === Node-RED Remote Bypass Control: received command log ===
	ESP_LOGI("mqtt_receiver", "received %s %c (bypass %s)", topic,
	         payload[0], payload[0] == '1' ? "ON" : "OFF");
	bypass_command command = payload[0] == '1' ? BYPASS_ON : BYPASS_OFF;
	if (xQueueSend(bypass_command_queue, &command, 0) != pdTRUE)
		ESP_LOGW("bypass", "Command queue full; command not applied");
}

struct mqtt_message {
	const char *topic;
	const char *message;
};

xQueueHandle mqtt_message_queue = xQueueCreate(10, sizeof(struct mqtt_message));
TaskHandle_t mqtt_sender_handler = NULL;
void mqtt_sender(void *) {
	struct mqtt_message msg;
	// === Node-RED Remote Bypass Control ===
	// Coalesce changes and retry the actual current state until published.
	bool bypass_status_pending = true;

	while (1) {
		if (ulTaskNotifyTake(pdTRUE, 0))
			bypass_status_pending = true;
		if (bypass_status_pending) {
			xSemaphoreTake(mqtt_lock, portMAX_DELAY);
			if (mqtt.connected() &&
			    mqtt.publish("esp32/bypass/status",
			                 get_collision_bypass() ? "1" : "0",
			                 true))
				bypass_status_pending = false;
			xSemaphoreGive(mqtt_lock);
		}
		if (xQueueReceive(mqtt_message_queue, &msg,
		                  pdMS_TO_TICKS(100)) != pdTRUE)
			continue;
		ESP_LOGV("mqtt_sender", "received %s %s", msg.topic,
		         msg.message);

		// === Node-RED Remote Bypass Control: serialize MQTT access ===
		xSemaphoreTake(mqtt_lock, portMAX_DELAY);
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
		xSemaphoreGive(mqtt_lock);
	}
}

// === Node-RED Remote Bypass Control ===
// Only the collision task calls this, preventing competing sensor/GPIO writes.
void set_collision_bypass(bool bypass) {
	digitalWrite(COLLISION_PROTECTION_BYPASS_STATUS_PIN, bypass);
	bool is_free = digitalRead(COLLISION_TRIGGER_PIN);
	digitalWrite(MO_EN_PIN,
	             (bypass || is_free) ? MO_EN_ACTIVE : MO_EN_INACTIVE);
	digitalWrite(COLLISION_STATUS_PIN, bypass ? LOW : !is_free);
	portENTER_CRITICAL(&bypass_state_lock);
	collision_protection_bypass = bypass;
	portEXIT_CRITICAL(&bypass_state_lock);
	ESP_LOGI("bypass", "%s collision check", bypass ? "disable" : "enable");
	struct mqtt_message msg = {
	    .topic = "esp32/collision",
	    .message = bypass ? "bypass" : (is_free ? "free" : "collide")};
	xQueueSend(mqtt_message_queue, &msg, 0);
	xTaskNotifyGive(mqtt_sender_handler);
}

TaskHandle_t collision_check_routine_handler = NULL;
void collision_check_routine(void *) {
	static int is_free_prev = false;
	struct mqtt_message msg = {.topic = "esp32/collision"};

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(20));

		// === Node-RED Remote Bypass Control ===
		// Stay alive to receive commands; skip sensor handling while
		// bypassed.
		bypass_command command;
		for (unsigned int n = 0;
		     n < 16 &&
		     xQueueReceive(bypass_command_queue, &command, 0) == pdTRUE;
		     ++n) {
			set_collision_bypass(command == BYPASS_TOGGLE
			                         ? !collision_protection_bypass
			                         : command == BYPASS_ON);
		}
		if (collision_protection_bypass)
			continue;

		int is_free = digitalRead(COLLISION_TRIGGER_PIN);
		ESP_LOGV("collision_check_routine", "Read value: %hd", is_free);
		if (is_free == is_free_prev)
			continue;
		is_free_prev = is_free;
		ESP_LOGI("collision_check_routine", "%s",
		         is_free ? "free" : "collide");

		digitalWrite(MO_EN_PIN, is_free ? MO_EN_ACTIVE : MO_EN_INACTIVE);
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

TaskHandle_t screen_display_routine_handler = NULL;
void screen_display_routine(void *) {
	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		screen.clearDisplay();
		screen.setCursor(0, 0);

		xSemaphoreTake(weather_temperature.lock, portMAX_DELAY);
		screen.printf("Temp %3.2f C\n", weather_temperature.value);
		xSemaphoreGive(weather_temperature.lock);
		xSemaphoreTake(weather_humidity.lock, portMAX_DELAY);
		screen.printf("Humid %3.2f %%\n", weather_humidity.value);
		xSemaphoreGive(weather_humidity.lock);
		screen.printf("MAC %s\n", WiFi.macAddress().c_str());
		screen.printf("IP %s\n", WiFi.localIP().toString().c_str());
		// === Node-RED Remote Bypass Control: serialize MQTT access ===
		xSemaphoreTake(mqtt_lock, portMAX_DELAY);
		bool mqtt_connected = mqtt.connected();
		xSemaphoreGive(mqtt_lock);
		screen.printf("MQTT %s\n",
		              mqtt_connected ? "Connected" : "Not connected");

		screen.display();
		ESP_LOGD("screen_display_routine", "Displayed MCU status");
	}
}

TaskHandle_t mqtt_weather_report_routine_handler = NULL;
void mqtt_weather_report_routine(void *) {
	char strbuf_temp[7];
	char strbuf_humid[7];
	struct mqtt_message msg;

	while (1) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		xSemaphoreTake(weather_temperature.lock, portMAX_DELAY);
		snprintf(strbuf_temp, sizeof(strbuf_temp), "%.2f",
		         weather_temperature.value);
		xSemaphoreGive(weather_temperature.lock);

		msg = (struct mqtt_message){.topic = "esp32/temperature",
		                            .message = strbuf_temp};
		xQueueSend(mqtt_message_queue, &msg, 0);

		xSemaphoreTake(weather_humidity.lock, portMAX_DELAY);
		snprintf(strbuf_humid, sizeof(strbuf_humid), "%.2f",
		         weather_humidity.value);
		xSemaphoreGive(weather_humidity.lock);

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
	// Server and callback configured in setup before MQTT tasks start.

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(3000));
		// === Node-RED Remote Bypass Control ===
		xSemaphoreTake(mqtt_lock, portMAX_DELAY);
		if (mqtt.connected()) {
			mqtt.loop();
			xSemaphoreGive(mqtt_lock);
			xQueueSend(mqtt_message_queue, &msg, 0);
			continue;
		}

		if (!WiFi.isConnected()) {
			xSemaphoreGive(mqtt_lock);
			ESP_LOGW("mqtt_autoconnect_routine",
			         "Cannot connect to mqtt server, WiFi is not "
			         "connected");
			continue;
		}

		// === Node-RED Remote Bypass Control ===
		if (mqtt.connect(client_id) &&
		    mqtt.subscribe("esp32/bypass/set", 1)) {
			xTaskNotifyGive(mqtt_sender_handler);
			ESP_LOGI("mqtt_autoconnect_routine",
			         "Connected to mqtt server");
			vTaskResume(mqtt_weather_report_routine_handler);
		} else {
			// Retry connection/subscription together if subscribing
			// failed.
			mqtt.disconnect();
			ESP_LOGW("mqtt_autoconnect_routine",
			         "Cannot connect to mqtt server, retrying");
			vTaskSuspend(mqtt_weather_report_routine_handler);
		}
		xSemaphoreGive(mqtt_lock);
	}
}

TaskHandle_t weather_sensor_polling_routine_handler = NULL;
void weather_sensor_polling_routine(void *) {
	float read_buf;

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(3000));

		read_buf = dht.readTemperature();
		ESP_LOGV("weather_sensor_polling_routine", "Temperature: %.2f",
		         read_buf);
		xSemaphoreTake(weather_temperature.lock, portMAX_DELAY);
		weather_temperature.value = read_buf;
		xSemaphoreGive(weather_temperature.lock);

		read_buf = dht.readHumidity();
		ESP_LOGV("weather_sensor_polling_routine", "Humidity: %.2f",
		         read_buf);
		xSemaphoreTake(weather_humidity.lock, portMAX_DELAY);
		weather_humidity.value = read_buf;
		xSemaphoreGive(weather_humidity.lock);

		xTaskNotifyGive(mqtt_weather_report_routine_handler);
		xTaskNotifyGive(screen_display_routine_handler);
	}
}

IRAM_ATTR void collision_protection_bypass_trig_handler(void) {
	// === Node-RED Remote Bypass Control ===
	// Preserve one-second debounce; defer state/GPIO/MQTT work to tasks.
	static unsigned long last_trig = 0;
	unsigned long currtime = millis();
	if (currtime - last_trig > 1000) {
		bypass_command command = BYPASS_TOGGLE;
		BaseType_t higher_priority_task_woken = pdFALSE;
		if (xQueueSendFromISR(bypass_command_queue, &command,
		                      &higher_priority_task_woken) == pdTRUE)
			last_trig = currtime;
		if (higher_priority_task_woken)
			portYIELD_FROM_ISR();
	}
}

void setup(void) {
	weather_temperature.lock = xSemaphoreCreateMutex();
	weather_humidity.lock = xSemaphoreCreateMutex();
	// === Node-RED Remote Bypass Control ===
	bypass_command_queue = xQueueCreate(16, sizeof(bypass_command));
	mqtt_lock = xSemaphoreCreateMutex();
	configASSERT(bypass_command_queue && mqtt_lock);
	mqtt.setServer(MQTT_SERVER_IP, MQTT_PORT);
	mqtt.setCallback(mqtt_callback);

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
	// === Node-RED Remote Bypass Control: boot with protection active ===
	digitalWrite(COLLISION_PROTECTION_BYPASS_STATUS_PIN, LOW);
	bool is_free = digitalRead(COLLISION_TRIGGER_PIN);
	digitalWrite(MO_EN_PIN, is_free ? MO_EN_ACTIVE : MO_EN_INACTIVE);
	digitalWrite(COLLISION_STATUS_PIN, !is_free);

	dht.begin();
	Wire.begin(GPIO_NUM_21, GPIO_NUM_22);
	while (!screen.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
		ESP_LOGW("main", "Initialize SSD1306 Failed, retrying");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	screen.clearDisplay();
	screen.setTextColor(SSD1306_WHITE);
	screen.setTextSize(1);

	xTaskCreate(&led_blink, "led_blink", 1024, NULL, 0, NULL);
	xTaskCreate(&collision_check_routine, "collision_check_routine", 4096,
	            NULL, 0, &collision_check_routine_handler);
	xTaskCreate(&mqtt_autoconnect_routine, "mqtt_autoconnect_routine", 2048,
	            NULL, 0, &mqtt_autoconnect_routine_handler);
	xTaskCreate(&mqtt_sender, "mqtt_sender", 4096, NULL, 0,
	            &mqtt_sender_handler);
	xTaskCreate(&mqtt_weather_report_routine, "mqtt_weather_report_routine",
	            8192, NULL, 0, &mqtt_weather_report_routine_handler);
	xTaskCreate(&screen_display_routine, "screen_display_routine", 4096,
	            NULL, 0, &screen_display_routine_handler);
	xTaskCreate(&weather_sensor_polling_routine,
	            "weather_sensor_polling_routine", 4096, NULL, 0,
	            &weather_sensor_polling_routine_handler);

	attachInterrupt(COLLISION_PROTECTION_BYPASS_BTN_PIN,
	                &collision_protection_bypass_trig_handler, RISING);
}

void loop(void) { vTaskDelete(NULL); }
