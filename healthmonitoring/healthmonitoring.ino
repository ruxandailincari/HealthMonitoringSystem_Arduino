#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

const int ECG_SENSOR_PIN = A0;
const int LO_POS_PIN = 8;
const int LO_NEG_PIN = 9;

MAX30105 particleSensor;

uint32_t irBuffer[100];  
uint32_t redBuffer[100]; 
int32_t spo2;           
int8_t validSPO2;      
int32_t heartRate;     
int8_t validHeartRate;

unsigned long lastReport = 0;

#define REPORTING_PERIOD_MS 1000
#define VIBRATION_MOTOR_PIN 12

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Wire.setClock(400000);

  Serial.println("Initializare senzori");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 nu a fost gasit!");
    while (1);
  }

  byte ledBrightness = 60; 
  byte sampleAverage = 4;  
  byte ledMode = 2;        
  int sampleRate = 100;    
  int pulseWidth = 411;    
  int adcRange = 4096;     
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();

  pinMode(LO_POS_PIN, INPUT);
  pinMode(LO_NEG_PIN, INPUT);
  pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);

  Serial.println("Asteptare deget pentru calibrare initiala (4 secunde)");
  for (byte i = 0; i < 100; i++) {
    while (particleSensor.available() == false){ 
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
    if(i % 25 == 0) Serial.print(".");
  }
  
  maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void loop() {

  particleSensor.check();
  while (particleSensor.available()) {
    for (byte i = 1; i < 100; i++) {
      redBuffer[i - 1] = redBuffer[i];
      irBuffer[i - 1] = irBuffer[i];
    }
    redBuffer[99] = particleSensor.getRed();
    irBuffer[99] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  if (millis() - lastReport >= REPORTING_PERIOD_MS) {
    lastReport = millis();

    maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (!validHeartRate || heartRate == 0) {
      digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
    } else {
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
    }

    float temperature = particleSensor.readTemperature();

    Serial.print("{");

    Serial.print("\"hr\":");
    if (validHeartRate){ 
      Serial.print(heartRate);
    }
    else { 
      Serial.print(-1);
    }

    Serial.print(",\"spo2\":");
    if (validSPO2) { 
      Serial.print(spo2);
    }
    else {
       Serial.print(-1);
    }

    Serial.print(",\"temp\":");
    Serial.print(temperature, 2);

    Serial.print(",\"ecg\":");
    Serial.print((digitalRead(LO_POS_PIN) == 1 || digitalRead(LO_NEG_PIN) == 1) 
                  ? -1 
                  : analogRead(ECG_SENSOR_PIN));

    Serial.print(",\"leadsOff\":");
    Serial.print((digitalRead(LO_POS_PIN) == 1 || digitalRead(LO_NEG_PIN) == 1));

    Serial.println("}");
    }
}