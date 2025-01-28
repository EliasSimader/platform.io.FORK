#include <Ventile.h>

Ventile::Ventile(int ventilPin) {
    this->ventilPin = ventilPin;
}   

void Ventile::initialize() {
    pinMode(this->ventilPin, OUTPUT);
}

void Ventile::open(unsigned long duration) {
    digitalWrite(this->ventilPin, HIGH); 
    delay(duration); 
    digitalWrite(this->ventilPin, LOW); 
}