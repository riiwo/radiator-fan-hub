#include "ha_link.h"

#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

#define CHECK_US (30 * 1000000LL)

static ha_link_config cfg;
static esp_mqtt_client_handle_t mqtt;
static volatile bool mqtt_up = false;
static char base[32];

// Reconnects WiFi and logs the IP.
static void wifi_event(void *, esp_event_base_t event_base, int32_t id, void *) {
  if (event_base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) esp_wifi_connect();
  else if (event_base == IP_EVENT) printf("wifi connected\n");
}

// Restarts the WiFi join when off the AP, and MQTT when WiFi is up but MQTT is not.
static void check_link(void *) {
  wifi_ap_record_t ap;
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
    printf("wifi down, reconnecting\n");
    esp_wifi_disconnect();                    // drop any stuck join attempt
    esp_wifi_connect();                       // start a fresh join
  } else if (!mqtt_up) {
    printf("mqtt down, reconnecting\n");
    esp_mqtt_client_reconnect(mqtt);          // force a broker reconnect
  }
}

// Starts WiFi as a station.
static void wifi_init() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&init));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
  wifi_config_t wifi = {};
  strcpy((char *) wifi.sta.ssid, cfg.ssid);
  strcpy((char *) wifi.sta.password, cfg.password);
  wifi.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
  ESP_ERROR_CHECK(esp_wifi_start());
}

// Publishes one HA discovery config.
static void discover(const char *component, const char *object, const char *name, const char *extra) {
  char topic[96], payload[640];
  snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", component, base, object);
  snprintf(payload, sizeof(payload),
           "{\"name\": \"%s\", \"has_entity_name\": true, \"unique_id\": \"%s_%s\", %s, "
           "\"availability_topic\": \"%s/status\", \"device\": {\"identifiers\": [\"%s\"], "
           "\"name\": \"Radiator hub %s\", \"model\": \"Radiator fan hub\"}}",
           name, base, object, extra, base, base, cfg.hub_id);
  esp_mqtt_client_publish(mqtt, topic, payload, 0, 1, 1);
}

// Subscribes to a topic under the hub.
static void subscribe(const char *object) {
  char topic[48];
  snprintf(topic, sizeof(topic), "%s/%s", base, object);
  esp_mqtt_client_subscribe(mqtt, topic, 1);
}

// Flashes the image at url and restarts.
static void ota(const char *url) {
  printf("ota from %s\n", url);
  esp_http_client_config_t http = {};
  http.url = url;
  http.timeout_ms = 30000;
  esp_http_client_handle_t client = esp_http_client_init(&http);
  ESP_ERROR_CHECK(esp_http_client_open(client, 0));
  esp_http_client_fetch_headers(client);
  const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
  esp_ota_handle_t handle;
  ESP_ERROR_CHECK(esp_ota_begin(slot, OTA_SIZE_UNKNOWN, &handle));
  static char buf[1024];
  int n;
  while ((n = esp_http_client_read(client, buf, sizeof(buf))) > 0) ESP_ERROR_CHECK(esp_ota_write(handle, buf, n));
  esp_http_client_cleanup(client);
  ESP_ERROR_CHECK(esp_ota_end(handle));
  ESP_ERROR_CHECK(esp_ota_set_boot_partition(slot));
  printf("ota done, restarting\n");
  esp_restart();
}

// Publishes discovery and subscribes to commands.
static void on_connected() {
  char extra[256];
  const char *temp = "\"device_class\": \"temperature\", \"unit_of_measurement\": \"\xC2\xB0" "C\", "
                     "\"state_class\": \"measurement\"";
  snprintf(extra, sizeof(extra), "\"state_topic\": \"%s/flow\", %s", base, temp);
  discover("sensor", "flow", "Flow", extra);
  snprintf(extra, sizeof(extra), "\"state_topic\": \"%s/return\", %s", base, temp);
  discover("sensor", "return", "Return", extra);
  snprintf(extra, sizeof(extra), "\"state_topic\": \"%s/step\", \"icon\": \"mdi:fan\"", base);
  discover("sensor", "step", "Step", extra);
  snprintf(extra, sizeof(extra),
           "\"state_topic\": \"%s/mode\", \"command_topic\": \"%s/mode/set\", "
           "\"options\": [\"auto\", \"low\", \"mid\", \"high\"]", base, base);
  discover("select", "mode", "Mode", extra);
  snprintf(extra, sizeof(extra), "\"command_topic\": \"%s/press\"", base);
  discover("button", "press", "Press", extra);
  mqtt_up = true;
  ha_link_publish("status", "online");
  subscribe("mode/set");
  subscribe("press");
  subscribe("ota");
  printf("mqtt connected as %s\n", base);
}

// Tracks the connection and routes commands.
static void mqtt_event(void *, esp_event_base_t, int32_t id, void *data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) data;
  if (id == MQTT_EVENT_CONNECTED) on_connected();
  else if (id == MQTT_EVENT_DISCONNECTED) mqtt_up = false;
  else if (id == MQTT_EVENT_DATA) {
    char topic[48], value[160];
    snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
    snprintf(value, sizeof(value), "%.*s", event->data_len, event->data);
    const char *object = topic + strlen(base) + 1;
    if (strcmp(object, "press") == 0) cfg.on_press();
    else if (strcmp(object, "ota") == 0) ota(value);
    else cfg.on_mode(value);
  }
}

void ha_link_start(const ha_link_config *config) {
  cfg = *config;
  snprintf(base, sizeof(base), "radiator_hub_%s", cfg.hub_id);
  wifi_init();

  static char will[48];
  snprintf(will, sizeof(will), "%s/status", base);
  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.broker.address.uri = cfg.broker_uri;
  mqtt_cfg.credentials.client_id = base;
  mqtt_cfg.session.last_will.topic = will;
  mqtt_cfg.session.last_will.msg = "offline";
  mqtt_cfg.session.last_will.retain = 1;
  mqtt = esp_mqtt_client_init(&mqtt_cfg);
  ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt, MQTT_EVENT_ANY, mqtt_event, NULL));
  ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt));

  esp_timer_create_args_t args = {};
  args.callback = check_link;
  args.name = "ha_link";
  esp_timer_handle_t timer;
  ESP_ERROR_CHECK(esp_timer_create(&args, &timer));        // timer for the link check
  ESP_ERROR_CHECK(esp_timer_start_periodic(timer, CHECK_US));  // run it every 30 s
}

void ha_link_publish(const char *object, const char *value) {
  if (!mqtt_up) return;
  char topic[48];
  snprintf(topic, sizeof(topic), "%s/%s", base, object);
  esp_mqtt_client_publish(mqtt, topic, value, 0, 1, 1);
}
