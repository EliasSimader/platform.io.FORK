#ifndef SOILMOISTURESENSOR_H
#define SOILMOISTURESENSOR_H

#include <Arduino.h>

class SoilMoistureSensor {
    private:
        int sensorPin;
    public:
        SoilMoistureSensor(int sensorPin);
        
        void begin();

        int readValue();
};

#endif