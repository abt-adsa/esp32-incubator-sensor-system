#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>

#define BME_SCK 25
#define BME_MISO 32
#define BME_MOSI 26
#define BME_CS 33
#define NUM_SAMPLES 10

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

float temperatureSamples[NUM_SAMPLES];
float humiditySamples[NUM_SAMPLES];
int sampleIndex = 0;

void checkAlarm(float avgTemp, float avgHumidity);
void displayValues(float temperature, float humidity, float avgTemp, float avgHumidity);

void setup() {
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delay(2000);
}

void loop() {
  temperatureSamples[sampleIndex] = bme.readTemperature();
  humiditySamples[sampleIndex] = bme.readHumidity();
  sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

  float avgTemp = 0, avgHumidity = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
      avgTemp += temperatureSamples[i];
      avgHumidity += humiditySamples[i];
  }

  avgTemp /= NUM_SAMPLES;
  avgHumidity /= NUM_SAMPLES;

  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  displayValues(temperature, humidity, avgTemp, avgHumidity);

  checkAlarm(avgTemp, avgHumidity);

  delay(1000);
}

void displayValues(float temperature, float humidity, float avgTemp, float avgHumidity) {
  Serial.print("Current Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Current Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Average Temperature = ");
  Serial.print(avgTemp);
  Serial.println(" *C");

  Serial.print("Average Humidity = ");
  Serial.print(avgHumidity);
  Serial.println(" %");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("   T: ");
  display.print(temperature);
  display.println(" *C");

  display.print("  RH: ");
  display.print(humidity);
  display.println(" %");

  display.print(" AvT: ");
  display.print(avgTemp);
  display.println(" *C");

  display.print("AvRH: ");
  display.print(avgHumidity);
  display.println(" %");

  display.display();
}

void checkAlarm(float avgTemp, float avgHumidity) {
    String alarmMessage = "(!) ";

    bool alarmTriggered = false;

    if (avgTemp < 22 || avgTemp > 26) {
        alarmMessage += "T, ";
        alarmTriggered = true;
    }

    if (avgHumidity < 30 || avgHumidity > 60) {
        alarmMessage += "RH, ";
        alarmTriggered = true;
    }

    if (alarmTriggered) {
        alarmMessage.remove(alarmMessage.length() - 2);
        Serial.println(alarmMessage);
        
        display.setCursor(0, OLED_HEIGHT - 10);
        display.print(alarmMessage);
        display.display();
    }
}
