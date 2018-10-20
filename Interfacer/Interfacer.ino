#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <FS.h>
#include <EEPROM.h>
#include <FastLED.h>
FASTLED_USING_NAMESPACE

#include "config.h"

WiFiUDP client;

IRrecv irrecv(PIN_IR);

IPAddress multicastIP(239, 15, 18, 2);
const uint8_t  AtmoBufferSize = 5 + 3*AtmoLeds; // 0xC0FFEE + Cmd + OrbID + RGB
uint8_t buffer[AtmoBufferSize];

ESP8266WebServer server(80);

enum statuus {
    OTA_START,
    OTA_DONE,
    OTA_ERROR,
    AP_CONFIG
};

typedef void (*Pattern)();
typedef struct {
  uint8_t number;
  String name;
} PatternAndName;
typedef PatternAndName PatternAndNameList[];

// List of patterns to cycle through
const uint8_t patternCount = 10;
PatternAndNameList patterns = {
  { 1, "Strobe" },
  { 2, "Color Strobe" },
  { 3, "Moving Rainbow" },
  { 5, "Flag" },
  { 4, "Points" },
  { 6, "Fire2012" },
  { 7, "Random Colors" },
  { 8, "Test" },
  { 9, "Travelling Point" },
  { 10, "Solid Color" },
};

const uint8_t brightnessCount = 5;
uint8_t brightnessMap[brightnessCount] = { 16, 32, 64, 128, 255 };
int brightnessIndex = 0;
uint8_t brightness = brightnessMap[brightnessIndex];

uint8_t power = 1;

uint8_t currentPatternIndex = 0;
uint8_t currentPaletteIndex = 0;
CRGB solidColor = CRGB::Blue;


/*  ******************************************************************
    ** SETUP *********************************************************
    ******************************************************************  */

void setup() {
    WiFiManager wifiManager;

    Serial.begin(serialBaud);

    WiFi.begin("ssid", "pw");

    SPIFFS.begin();

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());

    // wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
    //         showStatus(AP_CONFIG);
    // });

    // wifiManager.autoConnect(AP_NAME, AP_PASS);

    client.beginMulticast(WiFi.localIP(), multicastIP, SERVER_PORT);
    // client.begin(SERVER_PORT);

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

    //EEPROM.begin(512);
    //loadSettings();

    server.on("/all", HTTP_GET, []() {
            sendAll();
            });

    server.on("/power", HTTP_GET, []() {
            sendPower();
            });

    server.on("/power", HTTP_POST, []() {
            String value = server.arg("value");
            setPower(value.toInt());
            sendPower();
            });

    server.on("/solidColor", HTTP_GET, []() {
            sendSolidColor();
            });

    server.on("/solidColor", HTTP_POST, []() {
            String r = server.arg("r");
            String g = server.arg("g");
            String b = server.arg("b");
            setSolidColor(r.toInt(), g.toInt(), b.toInt());
            sendSolidColor();
            });

    server.on("/pattern", HTTP_GET, []() {
            sendPattern();
            });

    server.on("/pattern", HTTP_POST, []() {
            String value = server.arg("value");
            setPattern(value.toInt());
            sendPattern();
            });

    server.on("/patternUp", HTTP_POST, []() {
            adjustPattern(true);
            sendPattern();
            });

    server.on("/patternDown", HTTP_POST, []() {
            adjustPattern(false);
            sendPattern();
            });

    server.on("/brightness", HTTP_GET, []() {
            sendBrightness();
            });

    server.on("/brightness", HTTP_POST, []() {
            String value = server.arg("value");
            setBrightness(value.toInt());
            sendBrightness();
            });

    server.on("/brightnessUp", HTTP_POST, []() {
            adjustBrightness(true);
            sendBrightness();
            });

    server.on("/brightnessDown", HTTP_POST, []() {
            adjustBrightness(false);
            sendBrightness();
            });

    server.on("/palette", HTTP_GET, []() {
            sendPalette();
            });

    server.on("/palette", HTTP_POST, []() {
            String value = server.arg("value");
            //setPalette(value.toInt());
            //sendPalette();
            });

    server.serveStatic("/index.htm", SPIFFS, "/index.htm");
    server.serveStatic("/fonts", SPIFFS, "/fonts", "max-age=86400");
    server.serveStatic("/js", SPIFFS, "/js");
    server.serveStatic("/css", SPIFFS, "/css", "max-age=86400");
    server.serveStatic("/images", SPIFFS, "/images", "max-age=86400");
    server.serveStatic("/", SPIFFS, "/index.htm");

    server.begin();

}

/*  ******************************************************************
    ** LOOP **********************************************************
    ******************************************************************  */

void loop() {
    decode_results IRcode;

    ArduinoOTA.handle();

    server.handleClient();

    if (irrecv.decode(&IRcode))
        handleIR(&IRcode);

    handleAtmo();

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
            sendCmd('b', 1);
        } else if (code == 0x807FE11E || code == 0x7E087B19) {
            // on
            sendCmd('r', 1);
        }
    }

    irrecv.resume();
}


void handleAtmo() {
    // AtmoOrb
    // https://github.com/ambilight-4-mediaportal/AtmoOrb/blob/master/Particle/Photon/AtmoOrb_UDP.ino
    int packetSize = client.parsePacket();

    if (packetSize == AtmoBufferSize) {
        client.read(buffer, AtmoBufferSize);
        //client.flush();
        unsigned int i = 0;

        // Look for 0xC0FFEE
        if(buffer[i++] == 0xC0 && buffer[i++] == 0xFF && buffer[i++] == 0xEE)
        {
            byte commandOptions = buffer[i++];
            byte rcvOrbID = buffer[i++];

            // Command options
            // 1 = force off
            // 2 = use lamp smoothing and validate by Orb ID
            // 4 = validate by Orb ID
            // 8 = discovery
            if(commandOptions == 1)
            {
                // Orb ID 0 = turn off all lights
                // Otherwise turn off selectively
                if(rcvOrbID == 0 || rcvOrbID == orbID)
                {
                    sendCmd('b', 1);
                }

                return;
            }
            else if(commandOptions == 2 or commandOptions == 4)
            {
                if(rcvOrbID != orbID)
                    return;

                byte red =  buffer[i++];
                byte green =  buffer[i++];
                byte blue =  buffer[i++];

                sendCmd('s', 0);
                sendColor(((uint32_t)red << 16) + ((uint32_t)green << 8) + blue);
            }
            else if(commandOptions == 8)
            {
                // Respond to remote IP address with Orb ID
                IPAddress remoteIP = client.remoteIP();

                client.beginPacket(remoteIP, client.remotePort());
                client.write(orbID);
                client.endPacket();

                return;
            }
        }
    }
}

void sendCmd(char cmd, uint8_t param) {
    /*
       Send command to other ESP
       Available commands:
       b 0: blank (black)
       r 0/1: run/stop
       p x: set pattern x
       s 0 c: set solid color
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

void sendColor(uint8_t r, uint8_t g, uint8_t b) {
    Serial.print((char)r);
    Serial.print((char)g);
    Serial.print((char)b);
}


void showStatus(enum statuus s) {
    // simple way to show current status (OTA Update, AP config,...) with LEDs

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


void sendAll()
{
  String json = "{";

  json += "\"power\":" + String(power) + ",";
  json += "\"brightness\":" + String(brightness) + ",";

  json += "\"currentPattern\":{";
  json += "\"index\":" + String(currentPatternIndex);
  json += ",\"name\":\"" + patterns[currentPatternIndex].name + "\"}";

  json += ",\"currentPalette\":{";
  json += "\"index\":" + String(currentPaletteIndex);
  json += ",\"name\":\"" + String("not supported") + "\"}";

  json += ",\"solidColor\":{";
  json += "\"r\":" + String(solidColor.r);
  json += ",\"g\":" + String(solidColor.g);
  json += ",\"b\":" + String(solidColor.b);
  json += "}";

  json += ",\"patterns\":[";
  for (uint8_t i = 0; i < patternCount; i++)
  {
    json += "\"" + patterns[i].name + "\"";
    if (i < patternCount - 1)
      json += ",";
  }
  json += "]";

  json += ",\"palettes\":[";
  json += "]";

  json += "}";

  server.send(200, "text/json", json);
  json = String();
}

void sendPower()
{
  String json = String(power);
  server.send(200, "text/json", json);
  json = String();
}

void sendPattern()
{
  String json = "{";
  json += "\"index\":" + String(currentPatternIndex);
  json += ",\"name\":\"" + patterns[currentPatternIndex].name + "\"";
  json += "}";
  server.send(200, "text/json", json);
  json = String();
}

void sendPalette()
{
  String json = "{";
  json += "\"index\":" + String(currentPaletteIndex);
  json += ",\"name\":\"" + String("not supported") + "\"";
  json += "}";
  server.send(200, "text/json", json);
  json = String();
}

void sendBrightness()
{
  String json = String(brightness);
  server.send(200, "text/json", json);
  json = String();
}

void sendSolidColor()
{
  String json = "{";
  json += "\"r\":" + String(solidColor.r);
  json += ",\"g\":" + String(solidColor.g);
  json += ",\"b\":" + String(solidColor.b);
  json += "}";
  server.send(200, "text/json", json);
  json = String();
}

void setPower(uint8_t value)
{
  power = value == 0 ? 0 : 1;
  if (power == 0) {
      sendCmd('r', 0);
      sendCmd('b', 1);
  } else {
      sendCmd('r', 1);
  }
}

void setSolidColor(CRGB color)
{
  setSolidColor(color.r, color.g, color.b);
}

void setSolidColor(uint8_t r, uint8_t g, uint8_t b)
{
    solidColor = CRGB(r, g, b);
    sendCmd('s', 0);
    sendColor(r, g, b);
}

// increase or decrease the current pattern number, and wrap around at the ends
void adjustPattern(bool up)
{
  if (up)
    currentPatternIndex++;
  else
    currentPatternIndex--;

  // wrap around at the ends
  if (currentPatternIndex < 0)
    currentPatternIndex = patternCount - 1;
  if (currentPatternIndex >= patternCount)
    currentPatternIndex = 0;
}

void setPattern(int value)
{
  // don't wrap around at the ends
  if (value < 0)
    value = 0;
  else if (value >= patternCount)
    value = patternCount - 1;

  currentPatternIndex = value;

  sendCmd('p', patterns[currentPatternIndex].number);
  sendCmd('r', 1);
}

// adjust the brightness, and wrap around at the ends
void adjustBrightness(bool up)
{
  if (up)
    brightnessIndex++;
  else
    brightnessIndex--;

  // wrap around at the ends
  if (brightnessIndex < 0)
    brightnessIndex = brightnessCount - 1;
  else if (brightnessIndex >= brightnessCount)
    brightnessIndex = 0;

  brightness = brightnessMap[brightnessIndex];

  FastLED.setBrightness(brightness);
}

void setBrightness(int value)
{
  // don't wrap around at the ends
  if (value > 255)
    value = 255;
  else if (value < 0) value = 0;

  brightness = value;

  FastLED.setBrightness(brightness);
}

void showSolidColor() {
    sendCmd('s', 0);
    sendColor(solidColor.r, solidColor.g, solidColor.b);
}

