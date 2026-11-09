/*
 * Lab 3 - De 4: Giam sat moi truong be nuoi thuy hai san
 * ESP32 + HC-SR04 (muc nuoc) + DS18B20 (nhiet do) + MQTT + FreeRTOS
 *
 * Core 0: TaskTruyenData  (WiFi + MQTT)
 * Core 1: TaskDocData     (doc cam bien, in Serial, day vao dataQueue/alertQueue)
 * Core 1: TaskCanhBao     (dieu khien may bom + van thoat)
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- Chan ket noi ----------
#define TRIG_PIN   5
#define ECHO_PIN   18
#define ONEWIRE_PIN 4
#define PUMP_PIN   25   // LED/PWM mo phong may bom
#define VALVE_PIN  26   // LED mo phong van thoat

#define TANK_HEIGHT_CM 250.0f   // chieu cao be (cam bien dat o mieng be)

// Core 3.x: ledcWrite dung chan GPIO; core 2.x: dung kenh PWM (0)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PUMP_PWM PUMP_PIN
#else
  #define PUMP_PWM 0
#endif

// ---------- WiFi / MQTT ----------
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
#define T_LEVEL "nam_ptit/aqua/level"
#define T_TEMP  "nam_ptit/aqua/temp"
#define T_PUMP  "nam_ptit/aqua/pump"
#define T_VALVE "nam_ptit/aqua/valve"
#define T_CMD   "nam_ptit/aqua/cmd/#"   // cmd/pump , cmd/valve : "1" / "0"

WiFiClient espClient;
PubSubClient mqtt(espClient);
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds(&oneWire);

struct SensorData { float level_m; float temp_c; };
struct Command    { char dev; int on; };   // dev: 'P' bom, 'V' van

QueueHandle_t dataQueue;    // SensorData day du
QueueHandle_t alertQueue;   // muc nuoc (float)
QueueHandle_t cmdQueue;     // lenh manual tu web

volatile int pumpPercent = 0;
volatile int valveOn = 0;

// ---------- Doc muc nuoc ----------
float readLevelMeters() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1;
  float distCm = dur * 0.0343f / 2.0f;
  float level = (TANK_HEIGHT_CM - distCm) / 100.0f;   // m
  return level < 0 ? 0 : level;
}

// ---------- MQTT ----------
void onMqtt(char* topic, byte* payload, unsigned int len) {
  String t(topic);
  Command c;
  c.on = (len > 0 && payload[0] == '1') ? 1 : 0;
  if (t.endsWith("/pump"))       c.dev = 'P';
  else if (t.endsWith("/valve")) c.dev = 'V';
  else return;
  xQueueSend(cmdQueue, &c, 0);
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    String id = "esp32-aqua-" + String((uint32_t)esp_random(), HEX);
    if (mqtt.connect(id.c_str())) {
      mqtt.subscribe(T_CMD);
      Serial.println("[MQTT] connected");
    } else {
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

// ---------- Task truyen data (Core 0) ----------
void TaskTruyenData(void* pv) {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\n[WiFi] connected");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);

  SensorData d;
  for (;;) {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
    if (xQueueReceive(dataQueue, &d, pdMS_TO_TICKS(100)) == pdTRUE) {
      mqtt.publish(T_LEVEL, String(d.level_m, 2).c_str());
      mqtt.publish(T_TEMP,  String(d.temp_c, 1).c_str());
      mqtt.publish(T_PUMP,  String((int)pumpPercent).c_str());
      mqtt.publish(T_VALVE, String((int)valveOn).c_str());
    }
  }
}

// ---------- Task doc data (Core 1) ----------
void TaskDocData(void* pv) {
  SensorData d;
  for (;;) {
    ds.requestTemperatures();
    d.temp_c  = ds.getTempCByIndex(0);
    d.level_m = readLevelMeters();

    if (d.level_m >= 0) {
      Serial.printf("Muc nuoc: %.2f m | Nhiet do: %.1f C | Bom: %d%% | Van: %d\n",
                    d.level_m, d.temp_c, (int)pumpPercent, (int)valveOn);
      xQueueSend(dataQueue, &d, 0);
      xQueueSend(alertQueue, &d.level_m, 0);
    } else {
      Serial.println("Loi doc HC-SR04");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------- Task canh bao (Core 1) ----------
void TaskCanhBao(void* pv) {
  float level;
  Command c;
  bool manualPump = false, manualValve = false;
  int autoPct = 0;
  bool autoValve = false;
  uint32_t pumpEnd = 0, valveEnd = 0;

  for (;;) {
    uint32_t now = millis();

    if (xQueueReceive(alertQueue, &level, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool pumpBusy = autoPct > 0, valveBusy = autoValve;
      if (level < 0.5f) {                                  // < 0.5 m: mo van 10 s
        if (!valveBusy) { autoValve = true; valveEnd = now + 10000; }
      } else if (level >= 1.0f && level < 1.5f) {          // 1 - 1.5 m: bom 50% 10 s
        if (!pumpBusy) { autoPct = 50; pumpEnd = now + 10000; }
      } else if (level >= 1.5f && level <= 2.0f) {         // 1.5 - 2 m: bom 100% 20 s
        if (!pumpBusy) { autoPct = 100; pumpEnd = now + 20000; }
      }
    }

    while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) {      // che do manual
      if (c.dev == 'P') manualPump = c.on;
      else              manualValve = c.on;
    }

    if (autoPct > 0 && now >= pumpEnd) autoPct = 0;
    if (autoValve && now >= valveEnd)  autoValve = false;

    int pct = manualPump ? 100 : autoPct;
    pumpPercent = pct;
    valveOn = (manualValve || autoValve) ? 1 : 0;
    ledcWrite(PUMP_PWM, map(pct, 0, 100, 0, 255));
    digitalWrite(VALVE_PIN, valveOn);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(VALVE_PIN, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PUMP_PIN, 5000, 8);   // PWM cho may bom (core 3.x)
#else
  ledcSetup(PUMP_PWM, 5000, 8);    // core 2.x
  ledcAttachPin(PUMP_PIN, PUMP_PWM);
#endif
  ds.begin();

  dataQueue  = xQueueCreate(5, sizeof(SensorData));
  alertQueue = xQueueCreate(5, sizeof(float));
  cmdQueue   = xQueueCreate(5, sizeof(Command));

  xTaskCreatePinnedToCore(TaskTruyenData, "TruyenData", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskDocData,    "DocData",    6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskCanhBao,    "CanhBao",    4096, NULL, 3, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
