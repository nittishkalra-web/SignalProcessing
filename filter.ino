#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------- LORA CONFIG ----------
#define LORA_SS   5
#define LORA_RST  14
#define LORA_DIO0 2
#define LORA_BAND 433E6

// ---------- WIFI + MQTT CONFIG ----------
const char* ssid       = "realme C65 5G";
const char* password   = "aaaaaaaa";
const char* mqttServer = "172.30.42.25";
const int   mqttPort   = 1883;
const char* mqttTopic  = "lora/data";

const float A = -85;
const float n = 2.01;

// Global clients
WiFiClient espClient;
PubSubClient client(espClient);

// ------------------------------------------------------------
//  DISTANCE FROM RSSI  (KEEPING YOUR ORIGINAL FUNCTION)
// ------------------------------------------------------------
float estimateDistance(int rssi) {
  return pow(10, (A - rssi) / (10 * n));
}

// ------------------------------------------------------------
//  ----------  MEDIAN + EWMA + PARTICLE FILTER  ----------
// ------------------------------------------------------------

// ---------- MEDIAN FILTER (5 samples) ----------
int rssiBuffer[5] = {0,0,0,0,0};
int rssiIndex = 0;

int medianFilter(int value) {
  rssiBuffer[rssiIndex] = value;
  rssiIndex = (rssiIndex + 1) % 5;

  int temp[5];
  memcpy(temp, rssiBuffer, sizeof(temp));

  // sort
  for (int i = 0; i < 5; i++)
    for (int j = i + 1; j < 5; j++)
      if (temp[j] < temp[i]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }

  return temp[2]; // median
}

// ---------- EWMA FILTER ----------
float ewmaRSSI = -70;         // initial guess
const float ALPHA = 0.20;     // smoothing factor

float ewmaFilter(float x) {
  ewmaRSSI = ALPHA * x + (1 - ALPHA) * ewmaRSSI;
  return ewmaRSSI;
}

// ---------- PARTICLE FILTER ----------
const int N_PARTICLES = 120;
float particles[N_PARTICLES];
float weights[N_PARTICLES];

float d_min = 0.3;
float d_max = 20.0;

// initialize particles
void initParticles() {
  for (int i = 0; i < N_PARTICLES; i++)
    particles[i] = d_min + ((float)rand() / RAND_MAX) * (d_max - d_min);
}

// predict step
void predictParticles() {
  for (int i = 0; i < N_PARTICLES; i++) {
    float noise = random(-8, 9) / 1000.0;  // ±0.008m
    particles[i] += noise;

    if (particles[i] < d_min) particles[i] = d_min;
    if (particles[i] > d_max) particles[i] = d_max;
  }
}

// measurement update
float b = 1.4;  // Laplace noise scale

void updateWeights(float rssi) {
  float sum = 0;
  for (int i = 0; i < N_PARTICLES; i++) {
    float expectedRSSI = A - 10 * n * log10(particles[i]);
    float err = fabs(rssi - expectedRSSI);

    weights[i] = exp(-err / b);
    sum += weights[i];
  }

  for (int i = 0; i < N_PARTICLES; i++)
    weights[i] /= sum;
}

// resampling
void resampleParticles() {
  float newPart[N_PARTICLES];

  int index = rand() % N_PARTICLES;
  float beta = 0;
  float maxW = 0;

  for (int i = 0; i < N_PARTICLES; i++)
    if (weights[i] > maxW) maxW = weights[i];

  for (int i = 0; i < N_PARTICLES; i++) {
    beta += ((float)rand() / RAND_MAX) * 2.0 * maxW;

    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % N_PARTICLES;
    }
    newPart[i] = particles[index];
  }

  memcpy(particles, newPart, sizeof(particles));
}

// final estimated distance
float getPFdistance() {
  float sum = 0;
  for (int i = 0; i < N_PARTICLES; i++)
    sum += particles[i] * weights[i];
  return sum;
}

// ------------------------------------------------------------
//  MQTT RECONNECT
// ------------------------------------------------------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_Gateway")) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, state=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ------------------------------------------------------------
//  SETUP
// ------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("\n--- ESP32 LoRa Gateway (3-Stage Filter Active) ---");

  // init LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed.");
    while (1);
  }

  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  // init WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, mqttPort);
  reconnect();

  initParticles();   // start particle filter
}

// ------------------------------------------------------------
//  LOOP
// ------------------------------------------------------------
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    String cowId = "";
    while (LoRa.available()) cowId += (char)LoRa.read();
    cowId.trim();

    if (cowId.length() == 0) return;

    int raw_rssi = LoRa.packetRssi();

    // ---------- 3 FILTER STAGES ----------
    int med = medianFilter(raw_rssi);
    float rssi_smooth = ewmaFilter(med);

    predictParticles();
    updateWeights(rssi_smooth);
    resampleParticles();
    float distance = getPFdistance();   // FINAL distance from PF

    // ---------- OUTPUT ----------
    Serial.println("\n--------------------");
    Serial.println("ID: " + cowId);
    Serial.println("Raw RSSI: " + String(raw_rssi));
    Serial.println("Median RSSI: " + String(med));
    Serial.println("EWMA RSSI: " + String(rssi_smooth));
    Serial.println("Distance: " + String(distance, 2) + " m");

    // ---------- JSON SEND ----------
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["id"] = cowId;
    jsonDoc["distance"] = round(distance * 100.0) / 100.0;

    char payload[200];
    serializeJson(jsonDoc, payload);
    client.publish(mqttTopic, payload);
  }
}