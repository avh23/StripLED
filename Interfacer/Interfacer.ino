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

IRrecv irrecv(PIN_IR);

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

    Serial.begin(serialBaud);

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
            sendCmd('p', code);
        }

    } else if (IR_type == 1) {
        // Alex's IR
        if (code == 0x807FF10E || code == 0x38D5F9F5) {
            // Flash
            sendCmd('p', 1);
            pattern = 1;
        } else if (code == 0x807FE916 || code == 0xADEB659D) {
            // Strobe
            sendCmd('p', 2);
            pattern = 2;
        } else if (code == 0x807FD926 || code == 0x92EBB33D) {
            // Fade
            sendCmd('p', 3);
            pattern = 3;
        } else if (code == 0x807FC936 || code == 0x7B758561) {
            // Smooth
            sendCmd('p', 4);
            pattern = 4;
        } else if (code == 0x9AF57A5F) {
            // off
            sendCmd('r', 0);
            sendCmd('b', 1);
        } else if (code == 0x807FE11E || code == 0x7E087B19) {
            // on
            sendCmd('r', 1);
        }
    }

    irrecv.resume();
}


void sendCmd(char cmd, uint8_t param) {
    /*
       Send command to other ESP
       Available commands:
       b: blank (black)
       r 0/1: stop/run
       p x: set pattern x
       s x: set color
       d l c: directly set color c of led l
     */
    Serial.print('!');
    Serial.print(cmd);
    Serial.print(param);
}

void sendColor(uint32_t param) {
    Serial.print( (param & (0x00ff0000)) >> 16);
    Serial.print( (param & (0x0000ff00)) >>  8);
    Serial.print( (param & (0x000000ff))      );
}


void showStatus(enum statuus s) {
    // simple way to show current status (OTA Update, AP config,...) with LEDs

    sendCmd('r', 0);
    sendCmd('b', 1);

    switch (s) {
        case OTA_START:
            sendCmd('d', 61);
            sendColor(0xffff00);
            break;
        case OTA_ERROR:
            sendCmd('d', 62);
            sendColor(0xff0000);
            break;
        case OTA_DONE:
            sendCmd('d', 63);
            sendColor(0x00ff00);
            break;
        case AP_CONFIG:
            sendCmd('d', 64);
            sendColor(0x0000ff);
            break;
    }

}

