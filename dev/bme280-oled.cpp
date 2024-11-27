#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>

constexpr uint8_t BME_SCK = 25;
constexpr uint8_t BME_MISO = 32;
constexpr uint8_t BME_MOSI = 26;
constexpr uint8_t BME_CS = 33;
constexpr uint8_t NUM_SAMPLES = 10;

constexpr uint16_t OLED_WIDTH = 128;
constexpr uint16_t OLED_HEIGHT = 64;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

float temperatureSamples[NUM_SAMPLES];
float humiditySamples[NUM_SAMPLES];
int sampleIndex = 0;

void initializeDisplay();
void initializeBME();
void checkAlarm(float temperatureAvg, float humidityAvg);
void displayValues(float temperature, float humidity, float temperatureAvg, float humidityAvg);
void sampleEnvironmentalData(float &temperature, float &humidity, float &temperatureAvg, float &humidityAvg);

void setup() {
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  initializeDisplay();
  initializeBME();

  delay(2000);
}

void loop() {
  float temperature = 0.0f, humidity = 0.0f, temperatureAvg = 0.0f, humidityAvg = 0.0f;
  sampleEnvironmentalData(temperature, humidity, temperatureAvg, humidityAvg);

  displayValues(temperature, humidity, temperatureAvg, humidityAvg);

  checkAlarm(temperatureAvg, humidityAvg);

  delay(1000);
}

void initializeDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        while (1);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
}

void initializeBME() {
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void displayValues(float temperature, float humidity, float temperatureAvg, float humidityAvg) {
  Serial.print("Current Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Current Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Average Temperature = ");
  Serial.print(temperatureAvg);
  Serial.println(" *C");

  Serial.print("Average Humidity = ");
  Serial.print(humidityAvg);
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
  display.print(temperatureAvg);
  display.println(" *C");

  display.print("AvRH: ");
  display.print(humidityAvg);
  display.println(" %");

  display.display();
}

void checkAlarm(float temperatureAvg, float humidityAvg) {
    String alarmMessage = "(!) ";

    bool alarmTriggered = false;

    if (temperatureAvg < 22 || temperatureAvg > 26) {
        alarmMessage += "T, ";
        alarmTriggered = true;
    }

    if (humidityAvg < 30 || humidityAvg > 60) {
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

void sampleEnvironmentalData(float &temperature, float &humidity, float &temperatureAvg, float &humidityAvg) {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();

    temperatureSamples[sampleIndex] = temperature;
    humiditySamples[sampleIndex] = humidity;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

    temperatureAvg = 0;
    humidityAvg = 0;

    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        temperatureAvg += temperatureSamples[i];
        humidityAvg += humiditySamples[i];
    }

    temperatureAvg /= NUM_SAMPLES;
    humidityAvg /= NUM_SAMPLES;
}