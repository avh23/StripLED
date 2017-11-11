// SYSTEM_THREAD(ENABLED);
#define FASTLED_ESP8266_RAW_PIN_ORDER
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 0
#include <FastLED.h>

#include "config.h"

WiFiUDP client;

int16_t pointpos[PointCount];
int16_t pointspd[PointCount];
CRGB pointcol[PointCount];

CRGB leds[PixelCount+10];   // +10 to prevent stupid overflows
#if REORDER
CRGB orderedleds[PixelCount+10];
#endif

static byte heat[PixelCount];

byte nextColor[3];
byte prevColor[3];
byte currentColor[3];
byte smoothStep = SMOOTH_STEPS;
unsigned long smoothMillis;

// for "Flag"
uint16_t flagpos = 0;

/*  ******************************************************************
    ** SETUP *********************************************************
    ******************************************************************  */

void setup() {

    Serial.begin(serialBaud);

#if REORDER
    FastLED.addLeds<NEOPIXEL, 3>(orderedleds, PixelCount);
#else
    FastLED.addLeds<NEOPIXEL, 3>(leds, PixelCount);
#endif
    FastLED.setMaxPowerInVoltsAndMilliamps(5, maxMilliAmp);
    FastLED.setCorrection(TypicalSMD5050);
    FastLED.setTemperature(DirectSunlight);

    SetRandomSeed();

    for (int i=0; i<PointCount; i++) {
        initPixel(i);
    }
}

/*  ******************************************************************
    ** LOOP **********************************************************
    ******************************************************************  */

void loop() {
    static uint8_t h = 0;
    static uint16_t p = 0;

    if (pattern == 1) {
        // Strobe
        fill_solid(leds, PixelCount, CRGB::White);
        reorderedShow();
        delay(2);
        fill_solid(leds, PixelCount, CRGB::Black);
        reorderedShow();
        delay(48);

    } else if (pattern == 2) {
        // Color Strobe
        fill_solid(leds, PixelCount, CHSV(random(256), 128, 255));
        reorderedShow();
        delay(2);
        fill_solid(leds, PixelCount, CRGB::Black);
        reorderedShow();
        delay(48);

    } else if (pattern == 3) {
        // Moving Rainbow
        fill_rainbow(leds, PixelCount, h, -2);
        reorderedShow();
        delay(10);
        h++;

    } else if (pattern == 5) {
        // Flagge
        fill_solid(leds, PixelCount, CRGB::Black);

        fill_solid(leds+(flagpos % PixelCount), 5, CRGB::Red);
        fill_solid(leds+((flagpos+5) % PixelCount), 5, CRGB::Yellow);
        fill_solid(leds+((flagpos+10) % PixelCount), 5, CRGB::Green);

        reorderedShow();
        delay(50);
        flagpos++;
        if (flagpos == PixelCount) flagpos = 0;

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
        reorderedShow();
        delay(36);

    } else if (pattern == 6) {    // Fire2012
        // Step 1.  Cool down every cell a little
        for( int i = 0; i < PixelCount; i++) {
            heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / PixelCount) + 2));
        }

        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for( int k= PixelCount - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
        }

        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if( random8() < SPARKING ) {
            int y = random8(7);
            heat[y] = qadd8( heat[y], random8(160,255) );
        }

        // Step 4.  Map from heat cells to LED colors
        for( int j = 0; j < PixelCount; j++) {
            CRGB color = HeatColor( heat[j]);
            int pixelnumber;
            if( gReverseDirection ) {
                pixelnumber = (PixelCount-1) - j;
            } else {
                pixelnumber = j;
            }
            leds[pixelnumber] = color;
        }
        reorderedShow();
        delay(20);

    } else if (pattern == 7) { // random colors
        // https://github.com/Resseguie/FastLED-Patterns/blob/master/fastled-patterns.ino

        for(int i=0; i<PixelCount; i++){
            leds[i] = Wheel(random(256));
        }
        reorderedShow();
        delay(300);

    } else if (pattern == 8) {
        // very simple "pattern" for testing
        fill_solid(leds, PixelCount, CRGB::Black);
        leds[0] = CRGB::Red;
        leds[1] = CRGB::Green;
        leds[2] = CRGB::Blue;
        reorderedShow();
        delay(100);

    } else if (pattern == 9) {
        // travelling white point to debug REORDER
        if (p < 580 || p > 600) p=580;
        fill_solid(leds, PixelCount, CRGB::Black);
        leds[p++] = CRGB::White;
        reorderedShow();
        delay(100);

    } else if (pattern == 10) {
        // solid color for all LEDs
        fill_solid(leds, PixelCount, CRGB::Green);
        reorderedShow();
        delay(1000);
        
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

/*  ******************************************************************
    ** FUNCTIONS *****************************************************
    ******************************************************************  */

void initPixel (int i) {
    pointpos[i] = random(PixelCount << 5);

    // Ensure the point is not standing still
    pointspd[i] = 0;
    while (pointspd[i] == 0)
        pointspd[i] = (int)random(2*PointsMaxSpeed+1) - PointsMaxSpeed;

    pointcol[i] = CHSV(random(256), 128 + random(128), 255);
}

void setColor(byte red, byte green, byte blue)
{
    fill_solid(leds, PixelCount, CRGB(red, green, blue));
    reorderedShow();
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

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
CRGB Wheel(byte WheelPos) {
    if (WheelPos < 85) {
        return CRGB(WheelPos * 3, 255 - WheelPos * 3, 0);
    } else if (WheelPos < 170) {
        WheelPos -= 85;
        return CRGB(255 - WheelPos * 3, 0, WheelPos * 3);
    } else {
        WheelPos -= 170;
        return CRGB(0, WheelPos * 3, 255 - WheelPos * 3);
    }
}


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

void reorderedShow() {
    // Swap some LED strips before sending the data
    // Allows wiring them up in any order

#if DEBUG_POWER
    uint16_t power;
    power = calculate_unscaled_power_mW(orderedleds, PixelCount) / 10000;
    fill_solid(leds+100, power, CRGB::Yellow);
    fill_solid(leds+100+power, 40-power, CRGB::Blue);
#endif

#if REORDER
    // adapt this for your setup
    memcpy(orderedleds, leds, S1 * sizeof(CRGB));
    for (int i=0; i<L1; i++) {
        orderedleds[S1+i] = leds[S1+L1-i-1];
    }
    memcpy(orderedleds+S1+L1, leds+S1+L1, L2 * sizeof(CRGB));
    for (int i=0; i<L3; i++) {
        orderedleds[S1+L1+L2+i] = leds[S1+L1+L2+L3-i-1];
    }
    for (int i=0; i<L4; i++) {
        orderedleds[S1+L1+L2+L3+i] = leds[S1+L1+L2+L3+L4-i-1];
    }

#endif

    FastLED.show();
}
