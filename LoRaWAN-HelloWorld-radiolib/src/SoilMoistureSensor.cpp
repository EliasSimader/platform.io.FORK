#include <SoilMoistureSensor.h>

SoilMoistureSensor::SoilMoistureSensor(int sensorPin) {
    this->sensorPin = sensorPin;
}

void SoilMoistureSensor::begin() {
    pinMode(this->sensorPin, INPUT);
}

int SoilMoistureSensor::readValue() {
    return analogRead(this->sensorPin);
}