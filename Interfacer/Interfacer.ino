#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <IRremoteESP8266.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>

#include "config.h"

WiFiUDP client;

const uint8_t  AtmoBufferSize = 5 + 3*AtmoLeds; // 0xC0FFEE + Cmd + OrbID + RGB
uint8_t buffer[AtmoBufferSize];

enum statuus {
    OTA_START,
    OTA_DONE,
    OTA_ERROR,
    AP_CONFIG
};

/*  ******************************************************************
    ** SETUP *********************************************************
    ******************************************************************  */

void setup() {
    WiFiManager wifiManager;

    wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
            showStatus(AP_CONFIG);
    });

    wifiManager.autoConnect(AP_NAME, AP_PASS);

    client.beginMulticast(WiFi.localIP(), multicastIP, SERVER_PORT);
    // client.begin(SERVER_PORT);

    Serial.begin(115200);

    irrecv.enableIRIn();

    ArduinoOTA.onStart([]() {
            showStatus(OTA_START);
            });
    ArduinoOTA.onEnd([]() {
            showStatus(OTA_DONE);
            });
    ArduinoOTA.onError([](ota_error_t error) {
            showStatus(OTA_ERROR);
            });
    // ArduinoOTA.setPassword("fastled");
    ArduinoOTA.begin();

}

/*  ******************************************************************
    ** LOOP **********************************************************
    ******************************************************************  */

void loop() {
    decode_results IRcode;

    ArduinoOTA.handle();

    if (irrecv.decode(&IRcode))
        handleIR(&IRcode);

    delay(10);
}

/*  ******************************************************************
    ** FUNCTIONS *****************************************************
    ******************************************************************  */

void handleIR(decode_results *results) {
    // previous code, to ignore repetitions
    static uint16_t lastcode = 0xFFFF;
    static byte lastrepeat = false;

    uint32_t code = results->value;

    if (IR_type == 0) {
        // Martin's IR
        bool repeatbit = ((code & 0x0800) == 0x0800);
        code &= ~0x0800; // cut "repeat" bit (0x0800)

        bool repeat = false;
        if (code == lastcode && repeatbit == lastrepeat) {
            repeat = true;
        } else {
            lastcode = code;
            lastrepeat = repeatbit;
        }

        bool shift = ( (code & 0x0140) == 0x0140);
        code &= ~0x0140; // cut "shift" bits

        if (code <=9 && ! repeat && ! shift) {
            pattern = code;
        }

    } else if (IR_type == 1) {
        // Alex's IR
        if (code == 0x807FF10E || code == 0x38D5F9F5) {
            // Flash
            pattern = 1;
        } else if (code == 0x807FE916 || code == 0xADEB659D) {
            // Strobe
            pattern = 2;
        } else if (code == 0x807FD926 || code == 0x92EBB33D) {
            // Fade
            pattern = 3;
        } else if (code == 0x807FC936 || code == 0x7B758561) {
            // Smooth
            pattern = 4;
        } else if (code == 0x9AF57A5F) {
            // off
            pattern = 0;
            setColor(0, 0, 0);
        } else if (code == 0x807FE11E || code == 0x7E087B19) {
            // on
            pattern = 0;
            setColor(100, 100, 100);
        }
    }

    irrecv.resume();
}

void showStatus(enum statuus s) {
    // simple way to show current status (OTA Update, AP config,...) with LEDs

    // Serial.print("Status ");
    // Serial.println(s);

    fill_solid(leds, PixelCount, CRGB::Black);

    switch (s) {
        case OTA_START:
            leds[61] = CRGB::Yellow;
            break;
        case OTA_ERROR:
            leds[62] = CRGB::Red;
            break;
        case OTA_DONE:
            leds[63] = CRGB::Green;
            break;
        case AP_CONFIG:
            leds[64] = CRGB::Blue;
            break;
    }

    reorderedShow();
    delay(0.04*PixelCount);
}

