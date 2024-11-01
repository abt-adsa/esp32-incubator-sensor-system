#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>

#define BME_SCK 25
#define BME_MISO 32
#define BME_MOSI 26
#define BME_CS 33
#define NUM_SAMPLES 10 // Number of samples for averaging

unsigned long delayTime;
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
float temperatureSamples[NUM_SAMPLES];
float humiditySamples[NUM_SAMPLES];
int sampleIndex = 0;

void printValues();
void checkAlarm(float avgTemp, float avgHumidity);

void setup() {
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    bool status = bme.begin();
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.println("-- Default Test --");
    delayTime = 1000;
}

void loop() { 
    temperatureSamples[sampleIndex] = bme.readTemperature();
    humiditySamples[sampleIndex] = bme.readHumidity();
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

    // Calculate averages
    float avgTemp = 0, avgHumidity = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        avgTemp += temperatureSamples[i];
        avgHumidity += humiditySamples[i];
    }
    avgTemp /= NUM_SAMPLES;
    avgHumidity /= NUM_SAMPLES;

    printValues();
    checkAlarm(avgTemp, avgHumidity);
    delay(delayTime);
}

void printValues() {
    Serial.print("Current Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Current Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void checkAlarm(float avgTemp, float avgHumidity) {

    if (avgTemp < 22 || avgTemp > 26) {
        Serial.println("Alarm: Temperature out of range!");
    }
    if (avgHumidity < 30 || avgHumidity > 60) {
        Serial.println("Alarm: Humidity out of range!");
    }

    Serial.print("Average Temperature = ");
    Serial.print(avgTemp);
    Serial.println(" *C");

    Serial.print("Average Humidity = ");
    Serial.print(avgHumidity);
    Serial.println(" %");

    Serial.println();
}
