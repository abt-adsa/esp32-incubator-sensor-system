#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include "MAX30105.h"
#include "heartRate.h"

#define BME_SCK 25
#define BME_MISO 32
#define BME_MOSI 26
#define BME_CS 33
#define NUM_SAMPLES 10

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// Objects for sensors
MAX30105 particleSensor;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Variables for heart rate sensor
const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// Variables for temperature and humidity sensor
float temperatureSamples[NUM_SAMPLES];
float humiditySamples[NUM_SAMPLES];
int sampleIndex = 0;

// Timing control variables
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 1000; // Display interval for both sensors

// Function declarations
void checkAlarm(float avgTemp, float avgHumidity, int avgBpm);
void displayAllValues(long irValue, float bpm, int avgBpm, float temperature, float humidity, float avgTemp, float avgHumidity);

void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing sensors..."));

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    // Initialize BME280 sensor
    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    // Initialize MAX30105 sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 was not found. Please check wiring/power.");
        while (1);
    }
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x7F);
    particleSensor.setPulseAmplitudeGreen(0);

    Serial.println("Place your index finger on the heart rate sensor with steady pressure.");
}

void loop() {
    // Heart rate sensor sampling
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++) {
                beatAvg += rates[x];
            }
            beatAvg /= RATE_SIZE;
        }
    }

    // Temperature and humidity sampling every 1000 ms
    if (millis() - lastDisplayUpdate >= displayInterval) {
        lastDisplayUpdate = millis();

        // Collect temperature and humidity samples
        temperatureSamples[sampleIndex] = bme.readTemperature();
        humiditySamples[sampleIndex] = bme.readHumidity();
        sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

        // Calculate average temperature and humidity
        float avgTemp = 0, avgHumidity = 0;
        for (int i = 0; i < NUM_SAMPLES; i++) {
            avgTemp += temperatureSamples[i];
            avgHumidity += humiditySamples[i];
        }
        avgTemp /= NUM_SAMPLES;
        avgHumidity /= NUM_SAMPLES;

        // Get current temperature and humidity values
        float temperature = bme.readTemperature();
        float humidity = bme.readHumidity();

        // Display all values on OLED
        displayAllValues(irValue, beatsPerMinute, beatAvg, temperature, humidity, avgTemp, avgHumidity);

        // Check alarm for environmental conditions and heart rate
        checkAlarm(avgTemp, avgHumidity, beatAvg);
    }
}

void displayAllValues(long irValue, float bpm, int avgBpm, float temperature, float humidity, float avgTemp, float avgHumidity) {
    display.clearDisplay();
    
    // Heart Rate Display
    display.setCursor(0, 0);
    display.print("  BPM: ");
    display.println(bpm);
    display.print("AvBPM: ");
    display.println(avgBpm);
    
    display.print("    T: ");
    display.print(temperature);
    display.println(" *C");

    display.print("   RH: ");
    display.print(humidity);
    display.println(" %");

    display.print("  AvT: ");
    display.print(avgTemp);
    display.println(" *C");

    display.print(" AvRH: ");
    display.print(avgHumidity);
    display.println(" %");

    display.display();
}

void checkAlarm(float avgTemp, float avgHumidity, int avgBpm) {
    String alarmMessage = "(!) ";
    bool alarmTriggered = false;

    // Check environmental conditions
    if (avgTemp < 22 || avgTemp > 26) {
        alarmMessage += "T, ";
        alarmTriggered = true;
    }
    if (avgHumidity < 30 || avgHumidity > 60) {
        alarmMessage += "RH, ";
        alarmTriggered = true;
    }

    // Check heart rate conditions
    if (avgBpm < 70 || avgBpm > 190) {
        alarmMessage += "HR, ";
        alarmTriggered = true;
    }

    // Display the alarm message if triggered
    if (alarmTriggered) {
        alarmMessage.remove(alarmMessage.length() - 2); // Remove last comma and space
        Serial.println(alarmMessage);
        display.setCursor(0, OLED_HEIGHT - 10);
        display.print(alarmMessage);
        display.display();
    }
}
