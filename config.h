// speed of communication between ESPs
const uint32_t serialBaud = 115200;

// "AtmoOrb" server
#define SERVER_PORT 49692
#define DISCOVERY_PORT 49692
unsigned int orbID = 1;

// Number of connected LEDs
// const uint16_t PixelCount = 659;
// const uint16_t PixelCount = 598;
// const uint16_t PixelCount = 32;
const uint16_t PixelCount = 975;

// 0 == Martin's Philips IR
// 1 == Alex's Tween IR
const uint8_t IR_type = 1;

// Each pixel needs ca 1mA even when black
// also, the power-calculation of FastLED seems to be off by a factor of ~0.6
const uint16_t maxMilliAmp = 5000;

// Which pin is the IR-sensor connected to
#define PIN_IR 5   // GPIO5 = D3

// starting pattern
int pattern = 4;

// Configuration for pattern "Points"
const uint8_t PointCount = 50;
const uint8_t PointsDim = 0;  // 0 = no trail, 256 = infinite trail
const uint8_t PointsMaxSpeed = 15;
const float   PointsAge = 2000;

// Settings for AtmoOrb
const uint8_t  AtmoLeds = 1;
const uint16_t AtmoFirstLed = 3;
const uint8_t  AtmoLedsPerLed = 5;
#define TIMEOUT_MS   500
// SMOOTHING SETTINGS
#define SMOOTH_STEPS 50 // Steps to take for smoothing colors
#define SMOOTH_DELAY 4 // Delay between smoothing steps
#define SMOOTH_BLOCK 0 // Block incoming colors while smoothing

// settings for Fire2012
// https://github.com/FastLED/FastLED/blob/master/examples/Fire2012/Fire2012.ino
// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 50, suggested range 20-100 
#define COOLING 70
// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
#define SPARKING 100
bool gReverseDirection = true;     // reverse direction?

// If WiFi isn't configured or doesn't connect, run as AP with this SSID/PW

#define AP_NAME "StripLED"
#define AP_PASS "blink"

// Support LED strips wired up in non-linear fashion?
// modify reorderedShow() accordingly!
#define REORDER 1
// length of individual strips
#define S1 113
#define L1 225
#define L2 157
// #define L3 89+76-65
#define L3 89+65+9

#define L4 89
#define L5 71
// #define L6 226
#define L6 220

// Areas Start/Length
#define A1S 0
#define A1L 338
#define A2S 338
#define A2L 157
#define A3S 585
#define A3L 372

// show a "power usage bar"
#define DEBUG_POWER 0
