// --- Library Includes ---
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <deque>
#include <algorithm>
#include <vector>

// --- Configuration Constants ---

// Pin configuration for sensors
constexpr uint8_t BME_SCK = 25;
constexpr uint8_t BME_MISO = 32;
constexpr uint8_t BME_MOSI = 26;
constexpr uint8_t BME_CS = 33;
constexpr uint8_t MIC_PIN = 14;

// Timing constants (in milliseconds)
constexpr uint32_t LOOP_TIME = 500;             // Sampling period
constexpr uint32_t HOUR_IN_MS = 3600000;
constexpr uint32_t DISPLAY_INTERVAL = 1000;     // Display update interval

// Sound measurement constants
constexpr int16_t BIAS = 1920;                  // Bias
constexpr float SPL_REF = 94.0f;                // Reference sound pressure level (dB)
constexpr float PEAK_REF = 0.00631f;            // Reference amplitude (Pa)
constexpr float AMP_GAIN = 80.0f;               // Amplifier gain for the mic
constexpr size_t BUFFER_SIZE = HOUR_IN_MS / LOOP_TIME; // 1-hour rolling window size

// Display settings
constexpr uint16_t OLED_WIDTH = 128;
constexpr uint16_t OLED_HEIGHT = 64;

// Sampling constants
constexpr uint8_t RATE_SIZE = 10;   // Sample size for heart rate averaging
constexpr uint8_t NUM_SAMPLES = 10; // Sample size for environmental averaging

// --- Sensor Objects ---
MAX30105 particleSensor;                     // Heart rate sensor object
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);  // OLED display object
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);      // Environmental sensor object

// --- Monitoring Variables ---

// Heart rate monitoring variables
uint8_t rates[RATE_SIZE];           // Heart rate sample array
uint8_t rateSpot = 0;               // Index for storing new heart rate samples
uint32_t lastBeat = 0;              // Last detected beat timestamp
float beatsPerMinute = 0.0f;
float beatAvg = 0.0f;

// Environmental monitoring variables
float temperature = 0.0f;
float humidity = 0.0f;
float temperatureSamples[NUM_SAMPLES];  // Temperature sample array
float humiditySamples[NUM_SAMPLES];     // Humidity sample array
uint8_t sampleIndex = 0;

// Sound level monitoring variables
std::deque<float> SPLBuffer;     // Rolling buffer for sound pressure level samples
float Leq = 0.0f;                // Equivalent continuous sound level
float Lmax = 0.0f;               // Maximum sound level
float L10 = 0.0f;                // 90th percentile sound level

// Timing control variables
uint32_t lastDisplayUpdate = 0;  // Last display update timestamp

// --- Function Declarations ---
void initializeDisplay();
void initializeBME280();
void initializeMAX30105();
void sampleHeartRate();
void sampleEnvironmentalData(float &avgTemp, float &avgHumidity);
void sampleSoundLevels();
void calculateLeq();
void calculateLmax();
void calculateL10();
void updateDisplay(long irValue, float bpm, int avgBpm, float temperature, float humidity, 
                   float avgTemp, float avgHumidity, float Leq, float Lmax, float L10);
void checkAlarm(float temperature, float humidity, float avgTemp, float avgHumidity, float bpm,
                int avgBpm, float Leq, float Lmax, float L10);

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing peripherals..."));

    initializeDisplay();
    initializeBME280();
    initializeMAX30105();
}

// --- Main Loop ---
void loop() {
    sampleHeartRate();

    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        lastDisplayUpdate = millis();

        float avgTemp = 0, avgHumidity = 0;
        sampleEnvironmentalData(avgTemp, avgHumidity);
        sampleSoundLevels();

        updateDisplay(0, beatsPerMinute, beatAvg, temperature, humidity, avgTemp, avgHumidity,
                      Leq, Lmax, L10);
        checkAlarm(temperature, humidity, avgTemp, avgHumidity, beatsPerMinute, beatAvg, Leq, Lmax, L10);
    }
}

// --- Initialization Functions ---

void initializeDisplay() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        while (1);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
}

void initializeBME280() {
    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}

void initializeMAX30105() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 was not found. Please check wiring/power.");
        while (1);
    }

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x7F);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("Place your index finger on the heart rate sensor with steady pressure.");
}

// --- Sampling Functions ---

// Heart rate sampling and averaging
void sampleHeartRate() {
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (uint8_t)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (uint8_t x = 0; x < RATE_SIZE; x++) {
                beatAvg += rates[x];
            }

            beatAvg /= RATE_SIZE;
        }
    }
}

// Environmental data sampling and averaging
void sampleEnvironmentalData(float &avgTemp, float &avgHumidity) {
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();

    temperatureSamples[sampleIndex] = temperature;
    humiditySamples[sampleIndex] = humidity;
    sampleIndex = (sampleIndex + 1) % NUM_SAMPLES;

    avgTemp = 0;
    avgHumidity = 0;

    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        avgTemp += temperatureSamples[i];
        avgHumidity += humiditySamples[i];
    }

    avgTemp /= NUM_SAMPLES;
    avgHumidity /= NUM_SAMPLES;
}

// Sound level sampling and calculation of metrics
void sampleSoundLevels() {
    int analogValue = abs(analogRead(MIC_PIN) - BIAS);
    static uint32_t lastSoundUpdate = 0;

    if (millis() - lastSoundUpdate >= LOOP_TIME) {
        lastSoundUpdate = millis();

        float dBSPL = SPL_REF + 20 * log10((float)analogValue / (PEAK_REF * AMP_GAIN)) - 80;
        SPLBuffer.push_back(dBSPL);

        if (SPLBuffer.size() > BUFFER_SIZE) {
            SPLBuffer.pop_front();
        }

        calculateLeq();
        calculateLmax();
        calculateL10();
    }
}

// --- Sound Metric Calculations ---

void calculateLeq() {
    float sumSquaredAmplitudes = 0.0f;
    for (float sample : SPLBuffer) {
        sumSquaredAmplitudes += pow(10, sample / 10.0f);
    }

    Leq = 10 * log10(sumSquaredAmplitudes / SPLBuffer.size());
}

void calculateLmax() {
    Lmax = *std::max_element(SPLBuffer.begin(), SPLBuffer.end());
}

void calculateL10() {
    std::vector<float> sortedBuffer(SPLBuffer.begin(), SPLBuffer.end());
    std::sort(sortedBuffer.begin(), sortedBuffer.end());
    uint32_t indexL10 = (uint32_t)(0.9 * sortedBuffer.size());
    L10 = sortedBuffer[indexL10];
}

// --- Display Output Functions ---

void updateDisplay(long irValue, float bpm, int avgBpm, float temperature, float humidity, 
                   float avgTemp, float avgHumidity, float Leq, float Lmax, float L10) {
    display.clearDisplay();
    
    display.setCursor(0, 0);
    display.print("HR/Av: ");
    display.print(bpm);
    display.print("/");
    display.print(avgBpm);
    display.println(" BPM");

    display.print(" T/Av: ");
    display.print(temperature);
    display.print("/");
    display.print(avgTemp);
    display.println(" C");

    display.print("RH/Av: ");
    display.print(humidity);
    display.print("/");
    display.print(avgHumidity);
    display.println(" %");

    // NICU Sound Levels Display
    display.println("Leq/Lmax/L10: ");
    display.print(Leq);
    display.print("/");
    display.print(Lmax);
    display.print("/");
    display.print(L10);
    display.println(" dB");

    display.display();
}

void checkAlarm(float temperature, float humidity, float avgTemp, float avgHumidity, float bpm,
                int avgBpm, float Leq, float Lmax, float L10) {
    String alarmMessage = "(!) ";
    bool alarmTriggered = false;

    if ((avgTemp < 22 || avgTemp > 26) && (temperature < 22 || temperature > 26)) {
        alarmMessage += "T, ";
        alarmTriggered = true;
    }
    if ((avgHumidity < 30 || avgHumidity > 60) && (humidity < 30 || humidity > 30)) {
        alarmMessage += "RH, ";
        alarmTriggered = true;
    }
    if ((avgBpm < 70 || avgBpm > 190) && (bpm < 70 || bpm > 190)) {
        alarmMessage += "HR, ";
        alarmTriggered = true;
    }
    if ((Leq > 45) || (Lmax > 60) || (L10 > 50)) {
        alarmMessage += "L, ";
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
