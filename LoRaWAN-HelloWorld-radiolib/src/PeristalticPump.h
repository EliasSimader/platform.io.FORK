#ifndef PeristalticPump_h
#define PeristalticPump_h

#include <Arduino.h>

class PeristalticPump {
    private:
        int pumpPin;
    public:
        PeristalticPump(int pumpPin);
        void initialize();
        void run(unsigned long duration);
};

#endif