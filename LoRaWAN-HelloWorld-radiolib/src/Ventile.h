#ifndef Ventile_h
#define Ventile_h

#include <Arduino.h>

class Ventile {
    private:
        int ventilPin;
    public:
        Ventile(int ventilPin);
        void initialize();
        void open(unsigned long duration);
};

#endif