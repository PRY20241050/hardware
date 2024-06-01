#include <MQ7.h>
#include <MQ135.h>
//#define LED_VERDE 25
//#define LED_AZUL 26
#define PIN_MQ135 35
#define PIN_MQ7 34
MQ135 mq135_sensor(PIN_MQ135);
MQ7 mq7_sensor(PIN_MQ7,5.0);
float temperature = 21.0; //ESTO SE PUEDE CAMBIAR EN BASE A UN DHT22
float humidity = 25.0; //ESTO SE PUEDE CAMBIAR EN BASE A UN DHT22
void setup() {
  Serial.begin(115200);
  //pinMode(LED_VERDE, OUTPUT);
  //pinMode(LED_AZUL, OUTPUT);
}

void loop() {
   //LECTURA DEL mq135
  float ppm_mq135 = mq135_sensor.getPPM();
  float correctedPPM_135 = mq135_sensor.getCorrectedPPM(temperature, humidity);
  Serial.print("\tPPM MQ135: ");
  Serial.print(ppm_mq135);
  Serial.print("\tCorrected PPM MQ135: ");
  Serial.print(correctedPPM_135);
  Serial.println(" ppm");

  // LECTURA DEL MQ7
  float ppm_mq7 = mq7_sensor.getPPM();
  Serial.print("\tPPM MQ7: ");
  Serial.print(ppm_mq7);
  Serial.println("\t");


  delay(1000);
}
