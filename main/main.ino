#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "Secrets.h"
#include <sps30.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <DHT22.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// Definiciones para la pantalla LCD
#define SCREEN_ADDRESS 0x27
#define SCREEN_WIDTH 16
#define SCREEN_HEIGHT 2

// Definiciones para los sensores SPS30, MQ135, MQ7, DHT22
#define PIN_DHT22 27

// Definiciones para los pines de los sensores de gases
#define PIN_CO 34
#define PIN_NO2 35
#define PIN_SO2 32

// Valores promedio de R0 para cada gas (en kOhms)
float R0_CO = 2.179087;
float R0_NO2 = 2.480836;
float R0_SO2 = 0.502561;

// Definiciones para IoT core
#define AWS_IOT_PUBLISH_TOPIC   "ESP32_IOT_W_SENSORS/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ESP32_IOT_W_SENSORS/sub"

// Inicializar objetos
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
LiquidCrystal_I2C lcd(SCREEN_ADDRESS, SCREEN_WIDTH, SCREEN_HEIGHT);
DHT22 dht22(PIN_DHT22); 

// Declaración de objetos para NTP
WiFiUDP ntpUDP;
const long utcOffsetInSeconds = -5 * 3600; // Offset para Perú (UTC-5)
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds, 60000);

// Inicializar variables globales
int16_t ret;
struct sps30_measurement m;
uint16_t data_ready;
float pm2 = 0.0;
float pm10 = 0.0;
float t = 0.0;
float h = 0.0;
float concentracion_CO = 0.0;
float concentracion_NO2 = 0.0;
float concentracion_SO2 = 0.0;
char fecha[11];
char tiempo[9];

// Función para calcular la concentración en ppm para CO
float calcularConcentracionCO(float RS_CO) {
  float x = log(RS_CO / R0_CO);
  float y = exp(3.4829 * x - 0.843);
  return y;
}

// Función para calcular la concentración en ppm para NO2
float calcularConcentracionNO2(float RS_NO2) {
  float y = 6.3792 * RS_NO2 - 0.0115;
  return y;
}

// Función para calcular la concentración en ppm para SO2
float calcularConcentracionSO2(float RS_SO2) {
  float x = log(RS_SO2 / R0_SO2);
  float y = exp(2.8598 * x - 0.293);
  return y;
}

void setup() {
  Serial.begin(115200);
  connectAWS();

  while (true) {
    if (encender_lcd() && iniciar_sps30()) 
    {
      break;
    }
    delay(1000);  // Espera 1 segundo antes de volver a intentar
  }

  Serial.println("Calibration done!");

  // Inicializar el cliente NTP
  timeClient.begin();
  timeClient.update();  // Obtener la hora inicial
  getFormattedTime(tiempo);
  getFormattedDate(fecha);
  
  delay(1000);
  mostrarMensajeCalibracion();
}

void loop() {
  delay(20000);  // Espera 1 segundo entre mediciones
  timeClient.update();
  getFormattedTime(tiempo);
  getFormattedDate(fecha);
  Serial.println("Fecha: " + String(fecha));
  Serial.println("Hora: " + String(tiempo));

  leer_sensirion_sps30();
  imprimir_en_serial();
  leerSensoresGases();
  leer_dht22();
  imprimir_en_serial();
  actualizarPantalla();  // Actualiza la pantalla con las nuevas lecturas
  publishMessage();
  client.loop();
}

// Inicialización de la pantalla LCD
bool encender_lcd(){
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");
  return true;
}

// Inicialización del sensor SPS30
bool iniciar_sps30() {
    sensirion_i2c_init();
    while (sps30_probe() != 0) {
        Serial.print("SPS sensor probing failed\n");
        delay(500);  // Espera 500 ms antes de volver a intentar
    }
    int ret = sps30_start_measurement();
    if (ret < 0) {
        Serial.print("Error starting measurement\n");
        return false;  // Devuelve false si hay un error al iniciar la medición
    }
    return true;
}

// Mensaje de calibracion en pantalla LiquidCrystal 16x2
void mostrarMensajeCalibracion() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibracion");
  lcd.setCursor(0, 1);
  lcd.print("completada");
  delay(2000);  // Espera 2 segundos
}

void leer_sensirion_sps30() {
  ret = sps30_read_data_ready(&data_ready);  // Verifica si los datos están listos
  if (ret < 0) {
    Serial.print("Error reading data-ready flag: ");
    Serial.println(ret);
    return;  // Si hay un error, salta a la siguiente iteración del loop
  } else if (!data_ready) {
    Serial.print("Data not ready, no new measurement available\n");
    return;  // Si los datos no están listos, salta a la siguiente iteración del loop
  }

  ret = sps30_read_measurement(&m);  // Lee la medición
  if (ret < 0) {
    Serial.print("Error reading measurement: ");
    Serial.println(ret);
  } else {
    pm2 = m.mc_2p5;
    pm10 = m.mc_10p0;
  }
}

void leer_dht22() {
  t = dht22.getTemperature();
  h = dht22.getHumidity();
}

void leerSensoresGases() {
  // Leer las lecturas analógicas de CO, NO2 y SO2
  int valor_CO = analogRead(PIN_CO);
  int valor_NO2 = analogRead(PIN_NO2);
  int valor_SO2 = analogRead(PIN_SO2);

  // Calcular Rs (resistencia del sensor en kOhms)
  float RS_CO = (4095.0 / valor_CO - 1);
  float RS_NO2 = (4095.0 / valor_NO2 - 1);
  float RS_SO2 = (4095.0 / valor_SO2 - 1);

  // Calcular concentraciones en ppm
  concentracion_CO = calcularConcentracionCO(RS_CO);
  concentracion_NO2 = calcularConcentracionNO2(RS_NO2);
  concentracion_SO2 = calcularConcentracionSO2(RS_SO2);

}

void actualizarPantalla() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PM2.5:");
  lcd.print(pm2);
  lcd.setCursor(0, 1);
  lcd.print("PM10:");
  lcd.print(pm10);
  delay(2000); 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO:");
  lcd.print(concentracion_CO);
  lcd.setCursor(0, 1);
  lcd.print("NO2:");
  lcd.print(concentracion_NO2);
  delay(2000);  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SO2:");
  lcd.print(concentracion_SO2);
  lcd.setCursor(0, 1);
  lcd.print("Temperatura:");
  lcd.print(t);
  delay(2000);  // Espera 2 segundos
}

void connectAWS() {
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to Wi-Fi");

  for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
    WiFi.begin(WIFI_SSIDS[i], WIFI_PASSWORDS[i]);

    Serial.print("Trying to connect to ");
    Serial.println(WIFI_SSIDS[i]);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { // Intentar durante 10 segundos
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to Wi-Fi");
      Serial.println("");
      Serial.println(WIFI_SSIDS[i]);
      break;
    } else {
      Serial.println("Failed to connect to Wi-Fi");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to any Wi-Fi network");
    return;
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}

void publishMessage() {
  if (!client.connected()) {
    Serial.println("MQTT client not connected. Reconnecting...");
    connectAWS();
  }

  StaticJsonDocument<200> doc;
  doc["date"] = fecha;
  doc["time"] = tiempo;
  doc["pm2.5"] = pm2;
  doc["pm10"] = pm10;
  doc["co"] = concentracion_CO;
  doc["no2"] = concentracion_NO2;
  doc["so2"] = concentracion_SO2;
  doc["temperature"] = t;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  if (client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer)) {
    Serial.println("Message published successfully");
  } else {
    Serial.println("Message publishing failed");
  }
}

void getFormattedTime(char *timeBuffer) {
    // Obtener los componentes de hora
    time_t rawTime = (time_t)timeClient.getEpochTime();
    struct tm *ti;
    ti = localtime(&rawTime);

    snprintf(timeBuffer, 9, "%02d:%02d:%02d", ti->tm_hour, ti->tm_min, ti->tm_sec);
}

void getFormattedDate(char *dateBuffer) {
    // Obtener los componentes de fecha
    time_t rawTime = (time_t)timeClient.getEpochTime();
    struct tm *ti;
    ti = localtime(&rawTime);

    snprintf(dateBuffer, 11, "%04d-%02d-%02d", ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

void imprimir_en_serial(){
  Serial.print("PM 2.5: ");
  Serial.print(pm2);
  Serial.println(" ug/m3");
  Serial.print("PM 10.0: ");
  Serial.print(pm10);
  Serial.println(" ug/m3");
  Serial.print("CO: ");
  Serial.print(concentracion_CO);
  Serial.println(" ppm");
  Serial.print("NO2: ");
  Serial.print(concentracion_NO2);
  Serial.println(" ppm");
  Serial.print("SO2: ");
  Serial.print(concentracion_SO2);
  Serial.println(" ppm");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println("");

}
