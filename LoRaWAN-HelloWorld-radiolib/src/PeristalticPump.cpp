#include <PeristalticPump.h>

PeristalticPump::PeristalticPump(int pumpPin) {
    this->pumpPin = pumpPin;
}   

void PeristalticPump::initialize() {
    pinMode(this->pumpPin, OUTPUT);
}

void PeristalticPump::run(unsigned long duration) {
    digitalWrite(this->pumpPin, HIGH); 
    delay(duration); 
    digitalWrite(this->pumpPin, LOW); 
}