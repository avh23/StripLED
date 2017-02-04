#define SERVER_PORT 49692
#define DISCOVERY_PORT 49692
IPAddress multicastIP(239, 15, 18, 2);
unsigned int orbID = 1;

const uint16_t PixelCount = 659;

// 0 == Martin's Philips IR
// 1 == Alex's Tween IR
const uint8_t IR_type = 1;

// Each pixel needs ca 1mA even when black
// also, the power-calculation of FastLED seems to be off by a factor of ~0.6
const uint16_t maxMilliAmp = 2000;

IRrecv irrecv(5); // GPIO5 = D3

int pattern = 4;

const uint8_t PointCount = 50;   // for pattern "Points"
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

