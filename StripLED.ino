// SYSTEM_THREAD(ENABLED);
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <IRremoteESP8266.h>

#define SERVER_PORT 49692
#define DISCOVERY_PORT 49692
IPAddress multicastIP(239, 15, 18, 2);
unsigned int orbID = 1;

const uint16_t PixelCount = 30;

// Each pixel needs ca 1mA even when black
// also, the power-calculation of FastLED seems to be off by a factor of ~0.6
const uint16_t maxMilliAmp = (600 - 0.9*PixelCount ) / 0.6;

IRrecv irrecv(5); // GPIO5 = D3

WiFiUDP client;

int pattern = 4;

const uint8_t PointCount = 3;   // for pattern "Points"
const uint8_t PointsDim = 0;  // 0 = no trail, 256 = infinite trail
const uint8_t PointsMaxSpeed = 15;
const float   PointsAge = 2000;
int16_t pointpos[PointCount];
int16_t pointspd[PointCount];
CRGB pointcol[PointCount];

CRGB leds[PixelCount];

// Settings for AtmoOrb
const uint8_t  AtmoLeds = 1;
const uint16_t AtmoFirstLed = 3;
const uint8_t  AtmoLedsPerLed = 5;
const uint8_t  AtmoBufferSize = 5 + 3*AtmoLeds; // 0xC0FFEE + Cmd + OrbID + RGB
#define TIMEOUT_MS   500
uint8_t buffer[AtmoBufferSize];
// SMOOTHING SETTINGS
#define SMOOTH_STEPS 50 // Steps to take for smoothing colors
#define SMOOTH_DELAY 4 // Delay between smoothing steps
#define SMOOTH_BLOCK 0 // Block incoming colors while smoothing

byte nextColor[3];
byte prevColor[3];
byte currentColor[3];
byte smoothStep = SMOOTH_STEPS;
unsigned long smoothMillis;

void SetRandomSeed()
{
    uint32_t seed;

    // random works best with a seed that can use 31 bits
    // analogRead on a unconnected pin tends toward less than four bits
    seed = analogRead(0);
    delay(1);

    for (int shifts = 3; shifts < 31; shifts += 3)
    {
        seed ^= analogRead(0) << shifts;
        delay(1);
    }

    // Serial.println(seed);
    randomSeed(seed);
}

void setup() {
    WiFi.mode(WIFI_STA);
    WiFi.config({10,1,0,123}, {10,1,0,1}, {255,255,255,0});
    WiFi.begin("tabr.org", NULL);
    WiFi.waitForConnectResult();

    client.beginMulticast(WiFi.localIP(), multicastIP, SERVER_PORT);
    // client.begin(SERVER_PORT);

    Serial.begin(115200);

    irrecv.enableIRIn();

    ArduinoOTA.onStart([]() {
            // Serial.println("Start");
            // clear strip
            for (int i=0; i<PixelCount; i++) {
                leds[i] = CRGB::Black;
            }
            leds[1] = CRGB::Yellow;
            FastLED.show();
            delay(0.04*PixelCount);
            });
    ArduinoOTA.onEnd([]() {
            // Serial.println("\nEnd");
            leds[1] = CRGB::Black;
            leds[2] = CRGB::Black;
            leds[3] = CRGB::Green;
            FastLED.show();
            delay(0.04*PixelCount);
            });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            // int newprogress = progress / (total / 255);
            // if ( newprogress != oldprogress ) {
            //     strip.SetPixelColor(2, RgbColor((int)newprogress * 0.1));
            //     strip.Show();
            //     delay(0.04*PixelCount);
            //     oldprogress = newprogress;
            // }

            });
    ArduinoOTA.onError([](ota_error_t error) {
            leds[1] = CRGB::Black;
            leds[2] = CRGB::Red;
            leds[3] = CRGB::Black;
            FastLED.show();
            delay(0.04*PixelCount);

            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });
    // ArduinoOTA.setPassword("fastled");
    ArduinoOTA.begin();

    FastLED.addLeds<NEOPIXEL, 3>(leds, PixelCount);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, maxMilliAmp);
    FastLED.setCorrection(TypicalSMD5050);
    FastLED.setTemperature(DirectSunlight);

    SetRandomSeed();

    for (int i=0; i<PointCount; i++) {
        initPixel(i);
    }
}

void initPixel (int i) {
    pointpos[i] = random(PixelCount << 5);

    // Ensure the point is not standing still
    pointspd[i] = 0;
    while (pointspd[i] == 0)
        pointspd[i] = (int)random(2*PointsMaxSpeed+1) - PointsMaxSpeed;

    pointcol[i] = CHSV(random(256), 128 + random(128), 150);
}

uint8_t h = 0;

void loop() {
    decode_results IRcode;

    ArduinoOTA.handle();

    if (irrecv.decode(&IRcode))
        handleIR(&IRcode);

    if (pattern == 1) {
        // Strobe
        fill_solid(leds, PixelCount, CRGB::White);
        FastLED.show();
        delay(2);
        fill_solid(leds, PixelCount, CRGB::Black);
        FastLED.show();
        delay(48);

    } else if (pattern == 2) {
        // Color Strobe
        fill_solid(leds, PixelCount, CHSV(random(256), 128, 255));
        FastLED.show();
        delay(2);
        fill_solid(leds, PixelCount, CRGB::Black);
        FastLED.show();
        delay(48);

    } else if (pattern == 3) {
        // Moving Rainbow
        fill_rainbow(leds, PixelCount, h, -2);
        FastLED.show();
        delay(40);
        h++;

    } else if (pattern == 4) {
        // Points
        if (PointsDim == 0) {
            fill_solid(leds, PixelCount, CRGB::Black);
        } else {
            for (int i=0; i<PixelCount; i++) {
                leds[i].nscale8(PointsDim);
            }
        }
        for (int i=0; i<PointCount; i++) {
            uint16_t curpos = pointpos[i] >> 5;

            CRGB curcolor = CRGB::Black;
            for (int j=0; j<PointCount; j++) {
                if (curpos == (pointpos[j] >> 5)) {
                    curcolor += pointcol[j];
                }
            }
            leds[curpos] = curcolor;

            if (random(PointsAge) < 1) {
                initPixel(i);
            } else {
                pointpos[i] += pointspd[i];
                if (pointpos[i] < 0)
                    pointpos[i] += PixelCount << 5;
                else if (pointpos[i] >= PixelCount << 5)
                    pointpos[i] -= PixelCount << 5;
            }
        }
        FastLED.show();
        delay(36);

    } else if (pattern == 0) {
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
                        smoothStep = SMOOTH_STEPS;
                        forceLedsOFF();
                    }

                    return;
                }
                else if(commandOptions == 2)
                {
                    if(rcvOrbID != orbID)
                        return;

                    byte red =  buffer[i++];
                    byte green =  buffer[i++];
                    byte blue =  buffer[i++];

                    setSmoothColor(red, green, blue);
                }
                else if(commandOptions == 4)
                {
                    if(rcvOrbID != orbID)
                        return;

                    byte red =  buffer[i++];
                    byte green =  buffer[i++];
                    byte blue =  buffer[i++];

                    smoothStep = SMOOTH_STEPS;
                    setColor(red, green, blue);
                    setSmoothColor(red, green, blue);

                    return;
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

        if (smoothStep < SMOOTH_STEPS && millis() >= (smoothMillis + (SMOOTH_DELAY * (smoothStep + 1))))
        {
            smoothColor();
        }

        // delay(20);
    }

}

void setColor(byte red, byte green, byte blue)
{
    fill_solid(leds, PixelCount, CRGB(red, green, blue));
    FastLED.show();
}


// Set a new color to smooth to
void setSmoothColor(byte red, byte green, byte blue)
{
    if (smoothStep == SMOOTH_STEPS || SMOOTH_BLOCK == 0)
    {
        if (nextColor[0] == red && nextColor[1] == green && nextColor[2] == blue)
        {
            return;
        }

        prevColor[0] = currentColor[0];
        prevColor[1] = currentColor[1];
        prevColor[2] = currentColor[2];

        nextColor[0] = red;
        nextColor[1] = green;
        nextColor[2] = blue;

        smoothMillis = millis();
        smoothStep = 0;
    }
}

// Display one step to the next color
void smoothColor()
{
    smoothStep++;
    currentColor[0] = prevColor[0] + (((nextColor[0] - prevColor[0]) * smoothStep) / SMOOTH_STEPS);
    currentColor[1] = prevColor[1] + (((nextColor[1] - prevColor[1]) * smoothStep) / SMOOTH_STEPS);
    currentColor[2] = prevColor[2] + (((nextColor[2] - prevColor[2]) * smoothStep) / SMOOTH_STEPS);

    setColor(currentColor[0], currentColor[1], currentColor[2]);
}

// Force all leds OFF
void forceLedsOFF()
{
    setColor(0,0,0);
    clearSmoothColors();
}

// Clear smooth color byte arrays
void clearSmoothColors()
{
    memset(prevColor, 0, sizeof(prevColor));
    memset(currentColor, 0, sizeof(nextColor));
    memset(nextColor, 0, sizeof(nextColor));
}


void handleIR(decode_results *results) {
    // previous code, to ignore repetitions
    static uint16_t lastcode = 0xFFFF;
    static byte lastrepeat = false;

    uint16_t code = results->value;
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

    irrecv.resume();
}



