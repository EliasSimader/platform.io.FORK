/*
This demonstrates how to save the join information in to permanent memory
so that if the power fails, batteries run out or are changed, the rejoin
is more efficient & happens sooner due to the way that LoRaWAN secures
the join process - see the wiki for more details.

This is typically useful for devices that need more power than a battery
driven sensor - something like a air quality monitor or GPS based device that
is likely to use up it's power source resulting in loss of the session.

The relevant code is flagged with a ##### comment

Saving the entire session is possible but not demonstrated here - it has
implications for flash wearing and complications with which parts of the
session may have changed after an uplink. So it is assumed that the device
is going in to deep-sleep, as below, between normal uplinks.

Once you understand what happens, feel free to delete the comments and
Serial.prints - we promise the final result isn't that many lines.

*/

#if !defined(ESP32)
#pragma error("This is not the example your device is looking for - ESP32 only")
#endif

#include <Preferences.h>

RTC_DATA_ATTR uint16_t bootCount = 0;

#include "GPS.h"
#include "LoRaWAN.hpp"
#include "SoilMoistureSensor.h"
#include "PeristalticPump.h"
#include "Ventile.h"

static GAIT::LoRaWAN<RADIOLIB_LORA_MODULE> loRaWAN(RADIOLIB_LORA_REGION,
                                                   RADIOLIB_LORAWAN_JOIN_EUI,
                                                   RADIOLIB_LORAWAN_DEV_EUI,
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_APP_KEY},
#ifdef RADIOLIB_LORAWAN_NWK_KEY
                                                   (uint8_t[16]) {RADIOLIB_LORAWAN_NWK_KEY},
#else
                                                   nullptr,
#endif
                                                   RADIOLIB_LORA_MODULE_BITMAP);

static GAIT::GPS gps(GPS_SERIAL_PORT, GPS_SERIAL_BAUD_RATE, GPS_SERIAL_CONFIG, GPS_SERIAL_RX_PIN, GPS_SERIAL_TX_PIN);

SoilMoistureSensor soilSensor1(32);
SoilMoistureSensor soilSensor2(33);
SoilMoistureSensor soilSensor3(34);
SoilMoistureSensor soilSensor4(35);
PeristalticPump pump(22);
Ventile ventil1(21);
Ventile ventil2(22);
Ventile ventil3(25);
Ventile ventil4(26);

// abbreviated version from the Arduino-ESP32 package, see
// https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/deepsleep.html
// for the complete set of options
void print_wakeup_reason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("Wake from sleep"));
    } else {
        Serial.print(F("Wake not caused by deep sleep: "));
        Serial.println(wakeup_reason);
    }

    Serial.print(F("Boot count: "));
    Serial.println(++bootCount); // increment before printing
}

void gotoSleep(uint32_t seconds) {
    loRaWAN.goToSleep();
    gps.goToSleep();

    Serial.println("[APP] Go to sleep");
    Serial.println();

    esp_sleep_enable_timer_wakeup(seconds * 1000UL * 1000UL); // function uses uS
    esp_deep_sleep_start();

    Serial.println(F("\n\n### Sleep failed, delay of 5 minutes & then restart ###\n"));
    delay(5UL * 60UL * 1000UL);
    ESP.restart();
}

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;        // wait for serial to be initalised
    delay(2000); // give time to switch to the serial monitor

    print_wakeup_reason();

    Serial.println(F("Setup"));
    loRaWAN.setup(bootCount);

    loRaWAN.setDownlinkCB([](uint8_t fPort, uint8_t* downlinkPayload, std::size_t downlinkSize) {
        Serial.print(F("[APP] Payload: fPort="));
        Serial.print(fPort);
        Serial.print(", ");
        GAIT::arrayDump(downlinkPayload, downlinkSize);
    });
    Serial.println(F("[APP] Aquire data and construct LoRaWAN uplink"));

    std::string uplinkPayload = RADIOLIB_LORAWAN_PAYLOAD;
    uint8_t fPort = 221;

    //GPS
    gps.setup();
    if (gps.isValid()) {
        fPort = 1;          //1 is location
        uplinkPayload = std::to_string(gps.getLatitude()) + "," + std::to_string(gps.getLongitude()) + "," +
        std::to_string(gps.getAltitude()) + "," + std::to_string(gps.getHdop());
    }
    else {
        fPort = 223;         //223 are errors (read in the upload formatter)
        Serial.println(F("[ERROR] GPS-Daten ungültig."));
        uplinkPayload = "GPS_ERROR"; 
    }

    //Pumpe initialisieren
    pump.initialize();

    //Ventile initialisieren
    ventil1.initialize();
    ventil2.initialize();
    ventil3.initialize();
    ventil4.initialize();

    // Feuchtigkeitssensoren
    soilSensor1.begin();
    soilSensor2.begin();
    soilSensor3.begin();
    soilSensor4.begin();

    if(soilSensor1.readValue() >= 0 || soilSensor2.readValue() >= 0 || soilSensor3.readValue() >= 0 || soilSensor4.readValue() >= 0) {
        fPort=2;    // Port 2 is für die Feuchtigkeit

        // Sensorwerte
        float sensorValue1 = soilSensor1.readValue();
        float sensorValue2 = soilSensor2.readValue();
        float sensorValue3 = soilSensor3.readValue();
        float sensorValue4 = soilSensor4.readValue();

        // Feuchtigkeitswerte
        float moisture1 = map(sensorValue1, 0, 4095, 100, 0);
        float moisture2 = map(sensorValue2, 0, 4095, 100, 0);
        float moisture3 = map(sensorValue3, 0, 4095, 100, 0);
        float moisture4 = map(sensorValue4, 0, 4095, 100, 0);

        // Durchschnitt der Feuchtigkeitswerte
        float averageMoisture = (moisture1 + moisture2 + moisture3 + moisture4) / 4;

        // Payload zusammensetzen
        std::string moisturePayload =   "F1:" + std::to_string((int)moisture1) + 
                                        "F2:" + std::to_string((int)moisture2) + 
                                        "F3:" + std::to_string((int)moisture3) + 
                                        "F4:" + std::to_string((int)moisture4) + 
                                        "AVGM:" + std::to_string((int)averageMoisture)
                                        ;

        // Wenn Feuchtigkeit unter 50% -> Pumpenkreislauf wird aktiviert
        if(moisture1 < 50 || moisture2 < 50 || moisture3 < 50 || moisture4 < 50) {
            pump.run(5000);
            Serial.println("Pumpe wurde aktiviert");

            // Ventile öffnen sich, wenn Feuchtigkeit unter 50% fällt
            if (moisture1 < 50) {
                ventil1.open(5000); 
                Serial.println("Ventil 1 geöffnet");
            }
            if (moisture2 < 50) {
                ventil2.open(5000);
                Serial.println("Ventil 2 geöffnet");
            }
            if (moisture3 < 50) {
                ventil3.open(5000);
                Serial.println("Ventil 3 geöffnet");
            }
            if (moisture4 < 50) {
                ventil4.open(5000);
                Serial.println("Ventil 4 geöffnet");
            }

            Serial.println("Der Kreislauf wurde für 5 Sekunden bewässert!");
            moisturePayload = "P1, " + moisturePayload;  // P1 = Pump ON
        } else {
            moisturePayload = "P0, " + moisturePayload;  // P0 = Pump OFF
        }

        //Serial.println("Payload length: " + String(moisturePayload.length()));
        uplinkPayload = moisturePayload;
    } else {
        Serial.println("[ERROR] Feuchtigkeitssensor-Daten ungültig.");
        fPort = 223;         //223 are errors
        uplinkPayload = "FEUCHTIGKEITSSENSOR_ERROR";
    }

    loRaWAN.setUplinkPayload(fPort, uplinkPayload);
    Serial.println("Payload gesetzt, warte auf Übertragung...");
}

void loop() {
    loRaWAN.loop();
}