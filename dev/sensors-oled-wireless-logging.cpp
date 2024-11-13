// --- Library Includes ---
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Wifi.h>
#include <WiFiUdp.h>
#include <ThingSpeak.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <FS.h>
#include <SD.h>

#include <NTPClient.h>
#include <TimeLib.h>

#include <deque>
#include <algorithm>
#include <vector>

// --- Configuration Constants ---

// Wireless credentials
constexpr const char* SSID = "SSID";
constexpr const char* PASSWORD = "PASSWORD*"; 
constexpr uint32_t CHANNEL_ID = 0;
constexpr const char* WRITE_API_KEY = "WRITE API KEY";

// SPI pin configurations
constexpr uint8_t SPI_SCK = 25;
constexpr uint8_t SPI_MISO = 32;
constexpr uint8_t SPI_MOSI = 26;

// Pin configuration for sensors
constexpr uint8_t BME_CS = 33;
constexpr uint8_t MIC_PIN = 35;

// Pin configuration for SD card
constexpr uint8_t SD_CS = 4;

// Timing constants (in milliseconds)
constexpr uint32_t SAMPLING_INTERVAL = 500;
constexpr uint32_t HOUR_IN_MS = 3600000;
constexpr uint32_t DISPLAY_UPDATE_INTERVAL = 1000; 
constexpr uint32_t UPLOAD_INTERVAL = 20000;

// Sound measurement constants
constexpr int16_t BIAS = 1920;
constexpr float SPL_REF = 94.0f;                               // Reference sound pressure level (dB)
constexpr float PEAK_REF = 0.00631f;                           // Reference amplitude (Pa)
constexpr float AMP_GAIN = 80.0f;                              // Amplifier gain for the mic
constexpr size_t BUFFER_SIZE = HOUR_IN_MS / SAMPLING_INTERVAL; // 1-hour rolling window size

// Display settings
constexpr uint16_t OLED_WIDTH = 128;
constexpr uint16_t OLED_HEIGHT = 64;

// Sampling constants
constexpr uint8_t RATE_SIZE = 10;   // Sample size for heart rate averaging
constexpr uint8_t NUM_SAMPLES = 10; // Sample size for environmental averaging

// --- Sensor & Functionality Objects ---
MAX30105 particleSensor;                                      // Heart rate sensor object
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1); // OLED display object
Adafruit_BME280 bme(BME_CS, SPI_MOSI, SPI_MISO, SPI_SCK);     // Environmental sensor object
WiFiClient wifiClient;                                        // WiFi object
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);       // NTP object
WiFiUDP ntpUDP;                                               // NTP update object

// --- Variables ---

// Data logging variables
File dataFile;

// Heart rate monitoring variables
uint8_t rates[RATE_SIZE]; // Heart rate sample array
uint8_t rateSpot = 0;     // Index for storing new heart rate samples
uint32_t lastBeat = 0;    // Last detected beat timestamp
int bpm = 0;
float bpmAvg = 0.0f;

// Environmental monitoring variables
float temperatureSamples[NUM_SAMPLES]; // Temperature sample array
float humiditySamples[NUM_SAMPLES];    // Humidity sample array
uint8_t sampleIndex = 0;                

// Sound level monitoring variables
std::deque<float> splBuffer; // Rolling buffer for sound pressure level samples

// Timing control variables
uint32_t lastUpload = 0;
uint32_t lastDisplayUpdate = 0;

// --- Function Declarations ---
void initializeWireless();
void initializeRTC();
void initializeSD();
void initializeDisplay();
void initializeBME280();
void initializeMAX30105();
void sampleHeartRate();
void sampleEnvironmentalData(float &temperature, float &humidity, float &temperatureAvg, float &humidityAvg);
void sampleSoundLevels(float &spl, float &Leq, float &Lmax, float &L10);
void calculateLeq(float &Leq);
void calculateLmax(float &Lmax);
void calculateL10(float &L10);
void updateDisplay(int bpm, float bpmAvg, float temperature, float humidity, float temperatureAvg,
                   float humidityAvg, float Leq, float Lmax, float L10);
void checkAlarm(int bpm, float bpmAvg, float temperature, float humidity, float temperatureAvg,
                float humidityAvg, float Leq, float Lmax, float L10);
void dataUpload(int bpm, float temperature, float humidity, float spl);
void logData(int bpm, float temperature, float humidity, float spl);
String getCurrentDate();
String getCurrentTime();

// --- Setup ---
void setup() {
    Serial.begin(115200);

    initializeWireless();
    initializeRTC();
    initializeSD();

    Serial.println(F("Initializing peripherals..."));

    initializeDisplay();
    initializeBME280();
    initializeMAX30105();
}

// --- Main Loop ---
void loop() {
    sampleHeartRate();

    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        lastDisplayUpdate = millis();

        float temperature = 0.0f, humidity = 0.0f, temperatureAvg = 0.0f, humidityAvg = 0.0f;
        sampleEnvironmentalData(temperature, humidity, temperatureAvg, humidityAvg);

        float spl = 0.0f, Leq = 0.0f, Lmax = 0.0f, L10 = 0.0f;
        sampleSoundLevels(spl, Leq, Lmax, L10);

        updateDisplay(bpm, bpmAvg, temperature, humidity, temperatureAvg, humidityAvg, Leq, Lmax, L10);
        checkAlarm(bpm, bpmAvg, temperature, humidity, temperatureAvg, humidityAvg, Leq, Lmax, L10);

        logData(bpm, temperature, humidity, spl);

        if (millis() - lastUpload >= UPLOAD_INTERVAL) {
            lastUpload = millis();

            dataUpload(bpm, temperature, humidity, spl);
        }
    }
}

// --- Initialization Functions ---

void initializeWireless() {
    Serial.println(F("Initializing wireless connections..."));

    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED) { 
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to Wi-Fi!");

    ThingSpeak.begin(wifiClient);
}

void initializeRTC() {    
    timeClient.begin();
    timeClient.update();
    setTime(timeClient.getEpochTime());
}

void initializeSD() { 
    if (!SD.begin(SD_CS)) {
        Serial.println("SD Card initialization failed!");
        return;
    }

    String filename = "/data_" + getCurrentDate() + ".txt";
    dataFile = SD.open(filename.c_str(), FILE_APPEND);
    
    if (!dataFile) {
        Serial.println("Failed to open file for logging.");
    } else {
        Serial.printf("Logging to %s\n", filename.c_str());
    }
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

        bpm = 60 / (delta / 1000.0);

        if (bpm < 255 && bpm > 20) {
            rates[rateSpot++] = (uint8_t)bpm;
            rateSpot %= RATE_SIZE;

            bpmAvg = 0;
            for (uint8_t x = 0; x < RATE_SIZE; x++) {
                bpmAvg += rates[x];
            }

            bpmAvg /= RATE_SIZE;
        }
    }
}

// Environmental data sampling and averaging
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

// Sound level sampling and calculation of metrics
void sampleSoundLevels(float &spl, float &Leq, float &Lmax, float &L10) {
    int analogValue = abs(analogRead(MIC_PIN) - BIAS);
    static uint32_t lastSoundUpdate = 0;

    if (millis() - lastSoundUpdate >= SAMPLING_INTERVAL) {
        lastSoundUpdate = millis();

        spl = SPL_REF + 20 * log10((float)analogValue / (PEAK_REF * AMP_GAIN)) - 80;
        splBuffer.push_back(spl);

        if (splBuffer.size() > BUFFER_SIZE) {
            splBuffer.pop_front();
        }

        calculateLeq(Leq);
        calculateLmax(Lmax);
        calculateL10(L10);
    }
}

// --- Sound Metric Calculations ---

void calculateLeq(float &Leq) {
    float sumSquaredAmplitudes = 0.0f;

    for (float sample : splBuffer) {
        sumSquaredAmplitudes += pow(10, sample / 10.0f);
    }

    Leq = 10 * log10(sumSquaredAmplitudes / splBuffer.size());
}

void calculateLmax(float &Lmax) {
    Lmax = *std::max_element(splBuffer.begin(), splBuffer.end());
}

void calculateL10(float &L10) {
    std::vector<float> sortedBuffer(splBuffer.begin(), splBuffer.end());
    std::sort(sortedBuffer.begin(), sortedBuffer.end());
    
    uint32_t indexL10 = (uint32_t)(0.9 * sortedBuffer.size());

    L10 = sortedBuffer[indexL10];
}

// --- Display Output Functions ---

void updateDisplay(int bpm, float bpmAvg, float temperature, float humidity, float temperatureAvg,
                   float humidityAvg, float Leq, float Lmax, float L10) {
    display.clearDisplay();
    
    display.setCursor(0, 0);
    display.print("HR/Av: ");
    display.print(bpm);
    display.print("/");
    display.print(bpmAvg);
    display.println(" BPM");

    display.print(" T/Av: ");
    display.print(temperature);
    display.print("/");
    display.print(temperatureAvg);
    display.println(" C");

    display.print("RH/Av: ");
    display.print(humidity);
    display.print("/");
    display.print(humidityAvg);
    display.println(" %");

    display.println("Leq/Lmax/L10: ");
    display.print(Leq);
    display.print("/");
    display.print(Lmax);
    display.print("/");
    display.print(L10);
    display.println(" dB");

    display.display();
}

void checkAlarm(int bpm, float bpmAvg, float temperature, float humidity, float temperatureAvg,
                float humidityAvg, float Leq, float Lmax, float L10) {
    String alarmMessage = "(!) ";
    bool alarmTriggered = false;

    if ((bpm < 70 || bpm > 190) && (bpmAvg < 70 || bpmAvg > 190)) {
        alarmMessage += "HR, ";
        alarmTriggered = true;
    }
    if ((temperature < 22 || temperature > 26) && (temperatureAvg < 22 || temperatureAvg > 26)) {
        alarmMessage += "T, ";
        alarmTriggered = true;
    }
    if ((humidity < 30 || humidity > 30) && (humidityAvg < 30 || humidityAvg > 60)) {
        alarmMessage += "RH, ";
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

// --- Data Upload Function ---
void dataUpload(int bpm, float temperature, float humidity, float spl) {

    ThingSpeak.setField(1, bpm);
    ThingSpeak.setField(2, temperature);
    ThingSpeak.setField(3, humidity);
    ThingSpeak.setField(4, spl);

    int httpCode = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);

    if (httpCode == 200) {
        Serial.println("Data uploaded successfully");
    } else {
        Serial.println("Failed to upload data: " + String(httpCode));
    }
}

// --- SD card data logging functions ---

void logData(int bpm, float temperature, float humidity, float spl) {
  if (dataFile) {

    String data = getCurrentTime() + ", " + 
                  String(bpm, 2) + ", " +      
                  String(temperature, 2) + ", " +
                  String(humidity, 2) + ", " + 
                  String(spl, 2); 

    dataFile.println(data);
    dataFile.flush();
  }
}

// --- RTC Functions ---

String getCurrentDate() {
  time_t now = timeClient.getEpochTime();
  char dateBuffer[9];
  snprintf(dateBuffer, sizeof(dateBuffer), "%04d%02d%02d", year(now), month(now), day(now));

  return String(dateBuffer);
}

String getCurrentTime() {
  time_t now = timeClient.getEpochTime();
  char timeBuffer[9];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", hour(now), minute(now), second(now));

  return String(timeBuffer);
}