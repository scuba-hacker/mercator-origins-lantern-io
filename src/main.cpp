// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_AHTX0.h>

const uint32_t serialBaud = 38400;

Adafruit_INA219 currentSensor_ina219;
Adafruit_AHTX0 tempHumiditySensor;

float shuntVoltage=0,busVoltage=0,current_mA=0,loadVoltage=0,power_mW=0,power_mW_hardware=0, total_mA=0,total_mAH=0;
float max_current_ma=0, min_load_voltage=10, max_load_voltage=0, powerOnSec=0;
void accumulateEnergyUsage();
float temperature = 0, humidity = 0;

char sensorData[256];
const uint32_t tempHumidityDutyCycle    = 500;
const uint32_t sendSensorDataDutyCycle = 1000; 

uint32_t nextTimeToSendSensorData = 1000;
const uint32_t initialRainbowPeriod = 7000;   // was 7

uint32_t timeOfNextLemonByteExpectedReceived = 0;   // Will be set properly in setup()
const uint32_t maximumWaitForLemonByteReceived = 7000; // if no comms from Lemon for 60 seconds then flash red to show no comms (LC_NONE)

void readAsyncTempHumidity();
void sendSensorDataWhenReady();

// This is not needed for Arduino Nano Every.
// Although AVR is defined, it 
// The Arduino Nano Every is not AVR in the “classic” sense.
// Classic Arduinos (Uno, Nano, Pro Mini, etc.) use ATmega328P with the avr-libc toolchain.
// The Nano Every uses an ATmega4809 (megaAVR-0 family).
// While it’s technically still an AVR-family MCU, the megaAVR-0 chips don’t use the same avr/power.h functions that old sketches needed (e.g. clock_prescale_set).
// The Every runs at its configured clock (16 MHz) right out of the box — no prescaler hacks needed.

//#ifdef __AVR__
// #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
//#endif


// Current sensor GY-471 MAX471
// https://www.analog.com/en/products/max471.html

// https://devboards.info/boards/arduino-nano     (RX1)
// https://docs.arduino.cc/tutorials/nano-every/run-4-uart/

#define REED1_CLOSURE_EDGE_REBOOT_GPIO 4       // Connects to ground on close
#define REED2_SWITCH_SIDE_EDGE_POWER_OFF_ON_GPIO 5   // Connects to ground on close
#define ARDUINO_BOARD_LED 13
#define CIRCUIT_BREAKER_GPIO 15   // energises relay coil via NPN MOSFET
#define CURRENT_SENSOR_ANALOG_IN_GPIO  A6   // not used
#define VCC_HALVED_ANALOG_IN_GPIO      A7   // not used
#define LEAK_SENSOR_GPIO               LL   // leak at bottom of float
#define IGNORE_FLOAT_LEAK_GPIO         SS   // on-off switch, suppress leak alerts from float - 
                              // once blue leak sensors get wet they will probably stay that way

// HardwareSerial Serial1 TX_GPIO is on GPIO D0. Used to send a byte to Mako to say restarting and to send sensor data.
// HardwareSerial Serial1 RX_GPIO is on GPIO D1. Used to receive status for LEDs from Mako.

#define NEOPIXEL_GPIO    6 // GPIO D6 for neopixel ring
// Red Positive LED wire is connected to +5V
// Black Negative LED wire is connected to GND
// White Signal LED wire is connected to pin D6

#define NEOPIXEL_LED_COUNT 8   // number of neopixels in ring

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(NEOPIXEL_LED_COUNT, NEOPIXEL_GPIO, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// setup() function -- runs once at startup --------------------------------

int globalOffset = 0;
const int daytimeBrightness = 255;
const int nighttimeBrightness = 50;
const int rainbowBrightness = 30;       // 65mA for 30, set at 255 current is   210mA

int currentBrightness = 0;


enum AnimationType {
  ANIM_NONE = 0,
  ANIM_CHASE,
  ANIM_RAINBOW, 
  ANIM_THEATER_CHASE,
  ANIM_THEATER_CHASE_RAINBOW,
  ANIM_COLOR_WIPE,
  ANIM_FLASH_SINGLE,
  ANIM_ALL_OFF,
  ANIM_FILL_COLOR,
  ANIM_BLINK_ALL
};

enum AnimationMode {
  ANIM_MODE_INTERRUPT = 0,  // New animation interrupts current one
  ANIM_MODE_QUEUE = 1       // New animation waits for current to finish
};

struct AnimationEvent {
  AnimationType type;
  uint32_t color1, color2, color3, color4;
  int wait;
  int offset;
  int repeat_count;  // -1 for infinite, 0+ for specific count
  bool active;
};

struct AnimationState {
  AnimationEvent current_event;
  uint32_t next_frame_time;
  int frame_counter;
  int step_counter;
  bool animation_active;
};

#define MAX_ANIMATION_QUEUE 5
struct AnimationQueue {
  AnimationEvent events[MAX_ANIMATION_QUEUE];
  int head;
  int tail;
  int count;
  AnimationMode mode;
};

// Animation system function declarations
void initAnimationSystem();
bool startAnimation(AnimationType type, uint32_t color1, uint32_t color2 = 0, uint32_t color3 = 0, uint32_t color4 = 0, 
                   int wait = 100, int offset = 0, int repeat_count = -1, AnimationMode mode = ANIM_MODE_INTERRUPT);
void updateAnimations();
void stopCurrentAnimation();
bool isAnimationActive();
void processAnimationFrame();
void processChaseFrame();
void processRainbowFrame();
void processFlashSingleFrame();
void processBlinkAllFrame();
void processNextQueuedAnimation();

void flipDayNightBrightness();
void chaseStart(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait);
int incI(int i);
int addI(int i,int add);
void chaseEnd(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait);
void chase(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait);
void allOff();
void colorWipe(uint32_t color, int wait);
void theaterChase(uint32_t color, int wait);
void rainbow(int wait);
void theaterChaseRainbow(int wait);

const bool lowPowerTest=false;

void testTightSerialTxLoop()
{
  // must be called at top of loop() and not in setup()
  // do a return straight after calling the function
  // must have flush() call, see above
  // also found need the 1 uS to ensure reliable reception by ESP32-S3 when sending bytes one by one
  Serial1.write(85);  // test byte 0x55  0b01010101 == 85   (was using 123 before - esp32-s3 not always locking onto it)
  Serial1.flush();
  delayMicroseconds(1);
}

void setup() {
  Serial1.begin(serialBaud);  // on TX output there is a 1k and 2k voltage divider to reduce voltage from 5V to 3.3V for ESP32

  while (!Serial1); // needed on Arduino Nano Every 

/*

🔍 while (!Serial); — Why is it important?

On some Arduino boards (especially newer ones like the Nano Every, Leonardo, or any board with native USB), 
the Serial object represents a USB virtual serial port, not a traditional UART. Unlike the classic Uno, 
the USB connection needs to be enumerated by the host PC before Serial is ready to use.


✅ Why it matters on the Nano Every
The Nano Every uses a USB-to-serial bridge, so Serial depends on USB enumeration and initialization.
If you try to call Serial.print(), Serial.write(), or access USART0 before the serial port is fully active, the output may be:
Lost (never sent),
Garbage, or
Cause undefined behavior if you're also tweaking USART registers like USART0.CTRLC.

⏱ What does while (!Serial); do?
It waits until the serial port is ready, which usually means:
The host PC has enumerated the USB connection.
The virtual serial port is open.
The Arduino can now safely send and receive data.
*/


/* WHY FLUSH NEEDED ON ARDUINO NANO EVERY

When you do something like:
Serial.write(123);
Serial.write(123);
Serial.write(123);
...those function calls queue bytes rapidly into the transmit buffer, one after the other. Because the UART hardware has a transmit buffer, the second and third write() don’t wait for the previous byte’s frame to fully complete.
So:
Stop bits are defined in the frame, but...
If another byte is queued before the stop bit is fully transmitted, the UART hardware may:
Cut the stop bit short, or
Transition directly into the start bit of the next byte.
This results in no idle line (high voltage) between frames — especially bad when you use 2 stop bits and expect more spacing.

🔧 Summary:
Condition	What happens
Serial.write() back-to-back	Bytes are queued with no guaranteed spacing
UART TX buffer not empty	Next byte begins immediately after previous
Stop bit length may be shortened	Especially if UART prioritizes throughput
RX on ESP32 may misalign	Reads wrong bits — e.g., 123 becomes 175 or 237
No delay or flush()	No idle high time, so start bit isn't clear

How to force full stop bit transmission
Add Serial.flush(); after write()
This waits until the TX buffer is empty and transmission is complete, including the stop bit.
Add a small delay:
Serial.write(123);
delayMicroseconds(200); // ensures line stays idle
Or batch your data in one call (e.g., buffer then send all at once)

*/

  Serial.begin(115200);
  while (!Serial); // needed on Arduino Nano Every 
  Serial.flush();

  pinMode(REED1_CLOSURE_EDGE_REBOOT_GPIO, INPUT_PULLUP);
  pinMode(REED2_SWITCH_SIDE_EDGE_POWER_OFF_ON_GPIO, INPUT_PULLUP);
  pinMode(CIRCUIT_BREAKER_GPIO, OUTPUT);
  digitalWrite(CIRCUIT_BREAKER_GPIO,LOW);

  if (! currentSensor_ina219.begin()) {
    Serial.println("Failed to find current sensor INA219 chip");
    while (1) { delay(10); }
  }
  else
  {
    Serial.println("Found current sensor INA219 chip");
    currentSensor_ina219.setCalibration_32V_2A();      // Most common: 32V, 2A max
  }
  
  if (!tempHumiditySensor.begin()) 
    Serial.println("Failed to find AHT20 temperature & humidity sensor");
  else
  {
    Serial.println("Found AHT20 temperature & humidity sensor");
    // Trigger the first sensor reading immediately
    tempHumiditySensor.triggerSensorRead();
  }

  // By default the INA219 will be calibrated with a range of 32V, 2A.
  // However uncomment one of the below to change the range.  A smaller
  // range can't measure as large of values but will measure with slightly
  // better precision.
  //ina219.setCalibration_32V_1A();
  //ina219.setCalibration_16V_400mA();

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels AS    AP
  
  strip.setBrightness(daytimeBrightness);

  globalOffset=0;
  
  // Initialize animation system
  initAnimationSystem();
  
  // Initialize Lemon timeout - set to maximum wait period from startup
  timeOfNextLemonByteExpectedReceived = maximumWaitForLemonByteReceived;
}

enum lemonPattern
{
  STARTUP,
  WIFI,
  NO_GPS,
  NO_FIX,
  FIX,
  LEAK,
  LOW_POWER_TEST
};

uint8_t nextByteToSend = 0;

enum e_lemon_status{LC_NONE=0, LC_STARTUP=1, LC_SEARCH_WIFI=2, LC_FOUND_WIFI=3, LC_NO_WIFI=4, LC_NO_GPS=5, 
                    LC_NO_FIX=6, LC_GOOD_FIX=7, LC_ALL_OFF=8, LC_DIVE_IN_PROGRESS=64, LC_NO_STATUS_UPDATE=127, LC_NO_INTERNET=128};

// Animation System
// Global animation system variables
bool useAsyncAnimations = true;  // Set to false to use old synchronous animations
AnimationState anim_state = {{ANIM_NONE, 0, 0, 0, 0, 0, 0, 0, false}, 0, 0, 0, false};
AnimationQueue anim_queue = {{}, 0, 0, 0, ANIM_MODE_INTERRUPT};


e_lemon_status lastLemonStatus = LC_STARTUP;

e_lemon_status flushLemonStatus()
{
  while (Serial1.available())
    Serial1.read();
}

e_lemon_status readLemonStatus()
{
  e_lemon_status byteReceived = LC_NO_STATUS_UPDATE;

  while (Serial1.available())
  {
    byteReceived = (e_lemon_status) (Serial1.read());
    timeOfNextLemonByteExpectedReceived = millis() + maximumWaitForLemonByteReceived;
  }

  if (millis() > timeOfNextLemonByteExpectedReceived)
  {
    byteReceived = LC_NONE;
  }

  return byteReceived;
}

uint32_t nextStatusCheck = 1000;
const uint32_t checkStatusDutyCycle = 500;

uint8_t singlePixelToToggle = 0;

bool circuitBreakerTripped = false;

void(* resetArduino) (void) = 0; //declare reset function @ address 0

void flashAndPauseNextPixel(uint32_t colour, uint32_t wait)
{
  singlePixelToToggle = (singlePixelToToggle+1) % 8;
  // Rotating single low brightness
  strip.setPixelColor(singlePixelToToggle, colour);
  strip.show();                          //  Update strip to match
  delay(100);
  strip.setPixelColor(singlePixelToToggle, strip.Color(0,0,0));
  strip.show();                          //  Update strip to match
  delay(wait);
}

void sendRebootInitiated()
{
  const char rebootMessage[] = "{\"type\":\"lanternReboot\"}\n";
  Serial1.write(rebootMessage);
  Serial1.flush();
}

void sendPowerOnInitiated()
{
  const char powerOnMessage[] = "{\"type\":\"lanternPowerOn\"}\n";
  Serial1.write(powerOnMessage);
  Serial1.flush();
}

void sendPowerOffInitiated()
{
  const char powerOffMessage[] = "{\"type\":\"lanternPowerOff\"}\n";
  Serial1.write(powerOffMessage);
  Serial1.flush();
}

void loop()
{
  accumulateEnergyUsage();
  readAsyncTempHumidity();
  sendSensorDataWhenReady();
  
  // Update animation system only if using async animations
  if (useAsyncAnimations) {
    updateAnimations();
  }

  if (digitalRead(REED1_CLOSURE_EDGE_REBOOT_GPIO) == LOW)     // Restart / Reboot system
  {
    sendRebootInitiated();

    strip.fill(strip.Color(0,10,0),0,8);
    strip.show();                          //  Update strip to match
    digitalWrite(CIRCUIT_BREAKER_GPIO,HIGH);
    circuitBreakerTripped = true;
    delay(5000);
    digitalWrite(CIRCUIT_BREAKER_GPIO,LOW);
    circuitBreakerTripped = false;
    strip.fill(strip.Color(0,0,0),0,8);
    strip.show();                          //  Update strip to match
    resetArduino();
  }
  else if (digitalRead(REED2_SWITCH_SIDE_EDGE_POWER_OFF_ON_GPIO) == LOW)      // Toggle system power on/off
  {
    if (circuitBreakerTripped)
    {
      sendPowerOnInitiated();

      // All green LEDs, disable the circuit breaker - turns on the rest of the system.
      strip.fill(strip.Color(0,10,0),0,8);
      strip.fill(strip.Color(255,0,0),0,8);
      strip.show();
      digitalWrite(CIRCUIT_BREAKER_GPIO,LOW);
      delay(3000);
      strip.fill(strip.Color(0,0,0),0,8);
      strip.show();                          //  Update strip to match
      resetArduino();
    }
    else
    {
      sendPowerOffInitiated();

      // All red LEDs, enable the circuit breaker - turns off the rest of the system.
      strip.fill(strip.Color(0,255,0),0,8);
      strip.show();
      digitalWrite(CIRCUIT_BREAKER_GPIO,HIGH);
      delay(3000);
      strip.fill(strip.Color(0,0,0),0,8);
      lastLemonStatus = LC_NONE;
      flushLemonStatus();
    }
    
    circuitBreakerTripped = !circuitBreakerTripped;
  }
  
  if (millis() > nextStatusCheck)
  {
    nextStatusCheck += checkStatusDutyCycle;
    e_lemon_status newStatus = readLemonStatus();
    if (newStatus != LC_NO_STATUS_UPDATE && newStatus != lastLemonStatus)
    {
      lastLemonStatus = newStatus;
    }
  }

  if (lastLemonStatus == LC_STARTUP && millis() > initialRainbowPeriod)
    lastLemonStatus = LC_NONE;

  int ledWait=100;

  if (lowPowerTest)
  {
    if (useAsyncAnimations) {
      if (!isAnimationActive()) {
        startAnimation(ANIM_FLASH_SINGLE, strip.Color(10,10,0), 0, 0, 0, 200, 0, -1);
      }
    } else {
      // Original synchronous code
      flashAndPauseNextPixel(strip.Color(10,10,0), 200);
    }
    return;
  }

  // LC_DIVE_IN_PROGRESS flag reduces LED usage to save power
  switch(lastLemonStatus)
  {
    case LC_NONE:
    case LC_NONE | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (circuitBreakerTripped || lastLemonStatus & LC_DIVE_IN_PROGRESS)    // Rotating single low brightness LED
        {
          if (!isAnimationActive()) {
            startAnimation(ANIM_FLASH_SINGLE, strip.Color(0,10,0), 0, 0, 0, 200, 0, -1);
          }
        }
        else                           // All pixels low brightness LED flash
        {
          // Use proper blink animation
          if (!isAnimationActive()) {
            startAnimation(ANIM_BLINK_ALL, strip.Color(0,10,0), 0, 0, 0, 100, 0, -1);
          }
        }
      } else {
        // Original synchronous code
        if (circuitBreakerTripped || lastLemonStatus & LC_DIVE_IN_PROGRESS)    // Rotating single low brightness LED
        {
          flashAndPauseNextPixel(strip.Color(0,10,0), 200);
        }
        else                           // All pixels low brightness LED flash
        {
          // flash entire ring red/off/red/off
          strip.fill(strip.Color(0,10,0),0,8);
          strip.show();                          //  Update strip to match
          delay(100);
          strip.fill(strip.Color(0,0,0),0,8);
          strip.show();                          //  Update strip to match
          delay(100);
        }
      }
      break;
    }

    case LC_STARTUP:
    case LC_STARTUP | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          strip.setBrightness(rainbowBrightness);
          Serial.print("Starting rainbow animation, active: ");
          Serial.println(isAnimationActive());
          bool started = startAnimation(ANIM_RAINBOW, 0, 0, 0, 0, 1, 0, 1);  // 1 cycle, 1ms per frame (matches sync)
          Serial.print("Animation started: ");
          Serial.println(started);
          Serial.print("Now active: ");
          Serial.println(isAnimationActive());
        }
      } else {
        // Original synchronous code
        strip.setBrightness(rainbowBrightness);
        rainbow(1);
        strip.setBrightness(daytimeBrightness);
      }
      break;
    }
    case LC_SEARCH_WIFI:
    case LC_SEARCH_WIFI | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(0,0,255),   // BLUE
                        strip.Color(0,0,100),
                        strip.Color(0,0,50),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(0,0,255),   // BLUE
              strip.Color(0,0,100),
              strip.Color(0,0,50),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_FOUND_WIFI:
    case LC_FOUND_WIFI | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(255,50,255),   // CYAN
                        strip.Color(100,50,100),
                        strip.Color(50,50,50),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(255,50,255),   // CYAN
              strip.Color(100,50,100),
              strip.Color(50,50,50),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_NO_WIFI:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(0,255,255),   // MAGENTA
                        strip.Color(0,100,100),
                        strip.Color(0,50,50),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(0,255,255),   // MAGENTA
              strip.Color(0,100,100),
              strip.Color(0,50,50),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_NO_WIFI | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_FLASH_SINGLE, strip.Color(0,10,10), 0, 0, 0, 200, 0, -1);
        }
      } else {
        // Original synchronous code
        flashAndPauseNextPixel(strip.Color(0,10,10), 200);
      }
      break;
    }
    case LC_NO_GPS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(0,255,0),   // RED
                        strip.Color(0,100,0),
                        strip.Color(0,50,0),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(0,255,0),   // RED
              strip.Color(0,100,0),
              strip.Color(0,50,0),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_NO_GPS | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_FLASH_SINGLE, strip.Color(0,10,0), 0, 0, 0, 200, 0, -1);
        }
      } else {
        // Original synchronous code
        flashAndPauseNextPixel(strip.Color(0,10,0), 200);
      }
      break;
    }
    case LC_NO_FIX:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(255,255,0),   // YELLOW
                        strip.Color(100,100,0),
                        strip.Color(50,50,0),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(255,255,0),   // YELLOW
              strip.Color(100,100,0),
              strip.Color(50,50,0),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_NO_FIX | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_FLASH_SINGLE, strip.Color(10,10,0), 0, 0, 0, 200, 0, -1);
        }
      } else {
        // Original synchronous code
        flashAndPauseNextPixel(strip.Color(10,10,0), 200);
      }
      break;
    }
    case LC_GOOD_FIX:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(255,0,0),   // GREEN
                        strip.Color(100,0,0),
                        strip.Color(50,0,0),
                        strip.Color(0,0,0),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(255,0,0),   // GREEN
              strip.Color(100,0,0),
              strip.Color(50,0,0),
              strip.Color(0,0,0),
              ledWait);
      }
      break;
    }
    case LC_GOOD_FIX | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_FLASH_SINGLE, strip.Color(10,0,0), 0, 0, 0, 200, 0, -1);
        }
      } else {
        // Original synchronous code
        flashAndPauseNextPixel(strip.Color(10,0,0), 200);
      }
      break;
    }
    case LC_NO_INTERNET:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_CHASE,
                        strip.Color(30,255,80),   // BLUE BACKGROUND WITH MAGENTA CHASE
                        strip.Color(30,100,80),
                        strip.Color(30,50,80),
                        strip.Color(30,0,80),
                        ledWait, globalOffset, -1);  // infinite repeat
        }
      } else {
        // Original synchronous code
        chase(globalOffset,
              strip.Color(30,255,80),   // BLUE BACKGROUND WITH MAGENTA CHASE
              strip.Color(30,100,80),
              strip.Color(30,50,80),
              strip.Color(30,0,80),
              ledWait);
      }
      break;
    }
    case LC_NO_INTERNET | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (!isAnimationActive()) {
          startAnimation(ANIM_FLASH_SINGLE, strip.Color(0,10,10), 0, 0, 0, 200, 0, -1);
        }
      } else {
        // Original synchronous code
        flashAndPauseNextPixel(strip.Color(0,10,10), 200);
      }
      break;
    }
    case LC_ALL_OFF:
    case LC_ALL_OFF | LC_DIVE_IN_PROGRESS:
    {
      if (useAsyncAnimations) {
        if (isAnimationActive()) {
          stopCurrentAnimation();
        }
        startAnimation(ANIM_ALL_OFF, 0, 0, 0, 0, 0, 0, 1);
      } else {
        // Original synchronous code
        strip.fill(strip.Color(0,0,0),0,8);
        strip.show();
      }
      break;
    }
    default:
      break;
  }
}

void chaseStart(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait)
{
//  int i=(2+offset)%7;  i=(i?i:1);
  int i=-1;
  if (NEOPIXEL_LED_COUNT==7)
    i=addI(1,offset);
  else if (NEOPIXEL_LED_COUNT==6)
    i=addI(0,offset);
  else if (NEOPIXEL_LED_COUNT==8)
    i=addI(0,offset);
  else
    i=addI(0,offset);

  // assumes start at all dark
  
  // frame 1: first led is at color3
  // frame 2: first led is at color3, second led is at color2
  // frame 3: first led is at color3, second led is at color2, color 1
    strip.setPixelColor(i,color3);
    i=incI(i);

    strip.show();                          //  Update strip to match
    delay(wait);
    strip.setPixelColor(i,color2);
    i=incI(i);
    strip.show();                          //  Update strip to match
    delay(wait);
    strip.setPixelColor(i,color1);
    i=incI(i);
    strip.show();                          //  Update strip to match
    delay(wait);
}

int incI(int i)
{
  return addI(i,1);
}

int addI(int i,int add)
{
  if (NEOPIXEL_LED_COUNT==8)
  {
    i=(i+add)%8;
    return i;
  }
  else if (NEOPIXEL_LED_COUNT==7)  // the 7 count neopixel jewel had led in the middle that needed to be skipped.
  {
    i=(i-1+add)%6;
    return (i ? i+1 : 1);
  }
  else if (NEOPIXEL_LED_COUNT==6)
  {
    i=(i+add)%6;
    return i;
  }
  else
  {
    return i;
  }
}

void chaseEnd(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait)
{
  // assumes start at all dark
  
  // frame 0: first led is at color3, second led is at color2, second led is at color1,
  // frame 1: first led is at color4, second led is at color3, second led is at color2
  // frame 2: first led is at color4, second led is at color4, second led is at color3
  // frame 3: first led is at color4, second led is at color4, second led is at color4
  
  int i=-1;
  if (NEOPIXEL_LED_COUNT==7)
    i=addI(0,offset);
  else if (NEOPIXEL_LED_COUNT==7)
    i=addI(1,offset);
  else if (NEOPIXEL_LED_COUNT==6)
    i=addI(0,offset);
  else
    i=addI(0,offset);

    strip.setPixelColor(i,color4);
    strip.setPixelColor(addI(i,1),color3);
    strip.setPixelColor(addI(i,2),color2);
    strip.show();                          //  Update strip to match
    delay(wait);

    i=incI(i);
    strip.setPixelColor(i,color4);
    strip.setPixelColor(addI(i,1),color3);
    strip.show();                          //  Update strip to match
    delay(wait);

    i=incI(i);
    strip.setPixelColor(i,color4);
    strip.show();                          //  Update strip to match
    delay(wait);

    // last one will go to color4, last but one will go to color 
    // last but one will go da
}

void chase(int offset,
          uint32_t color1, 
          uint32_t color2, 
          uint32_t color3, 
          uint32_t color4, 
          int wait)
{
  int start=-1;
  if (NEOPIXEL_LED_COUNT==7)
    start=addI(1,offset);
  else if (NEOPIXEL_LED_COUNT==6)
    start=addI(0,offset);
  else
    start=addI(0,offset);

  
    for(int i=start; i<strip.numPixels()+start; i++) 
    
    { // For each pixel in strip...
      strip.setPixelColor(addI(i,3), color1);         //  Set pixel's color (in RAM)
      strip.setPixelColor(addI(i,2), color2);         //  Set pixel's color (in RAM)
      strip.setPixelColor(addI(i,1), color3);         //  Set pixel's color (in RAM)
      strip.setPixelColor(addI(i,0), color4);         //  Set pixel's color (in RAM)

/*
      strip.setPixelColor((i+3)%6+1, color1);         //  Set pixel's color (in RAM)
      strip.setPixelColor((i+2)%6+1, color2);         //  Set pixel's color (in RAM)
      strip.setPixelColor((i+1)%6+1, color3);         //  Set pixel's color (in RAM)
      strip.setPixelColor((i+0)%6+1, color4);         //  Set pixel's color (in RAM)
*/
      strip.show();                          //  Update strip to match
      delay(wait);                           //  Pause for a moment
    }
}

void allOff()
{
  for(int i=0; i<strip.numPixels(); i++) 
  { // For each pixel in strip...
    strip.setPixelColor(i, 0);         //  Set pixel's color (in RAM)
  }
    strip.show();                          //  Update strip to match
}

// Some functions of our own for creating animated effects -----------------

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<20; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}


void readAsyncTempHumidity()
{
  static sensors_event_t humidityRead, tempRead;
  static uint32_t nextSensorReadTrigger = 0;

  if (millis() >= nextSensorReadTrigger) {
    nextSensorReadTrigger = millis() + tempHumidityDutyCycle;
    tempHumiditySensor.triggerSensorRead();
  }

  // Call async sensor reading on every loop iteration
  if (tempHumiditySensor.getEventAsync(&humidityRead, &tempRead))
  {
    // Sensor data is ready!
    humidity = humidityRead.relative_humidity;
    temperature = tempRead.temperature;
  }
}

void sendSensorDataWhenReady()
{
  // Send sensor data at configured interval
  if (millis() >= nextTimeToSendSensorData)
  {
    nextTimeToSendSensorData += sendSensorDataDutyCycle;

    snprintf(sensorData, sizeof(sensorData),
      "{\"type\":\"lanternReadings\", \"Temp\":%f, \"Humid\":%f, \"I\":%f,\"V\":%f,\"mAH\":%f,\"Imax\":%f,\"Vmin\":%f,\"Vmax\":%f,\"mwMax\":%f}\n",
      temperature, humidity, current_mA, loadVoltage, total_mAH, max_current_ma, min_load_voltage, max_load_voltage, power_mW_hardware);

    Serial1.write(sensorData);
    // Serial1.flush();  // Remove blocking flush for async animations

    Serial.write(sensorData);
    // Serial.flush();   // Remove blocking flush for async animations
  }
}

// ================= ANIMATION SYSTEM IMPLEMENTATION =================

void initAnimationSystem() {
  anim_state.animation_active = false;
  anim_state.current_event.type = ANIM_NONE;
  anim_queue.head = 0;
  anim_queue.tail = 0;
  anim_queue.count = 0;
  anim_queue.mode = ANIM_MODE_INTERRUPT;
}

bool startAnimation(AnimationType type, uint32_t color1, uint32_t color2, uint32_t color3, uint32_t color4, 
                   int wait, int offset, int repeat_count, AnimationMode mode) {
  
  // Create animation event
  AnimationEvent new_event = {type, color1, color2, color3, color4, wait, offset, repeat_count, true};
  
  if (mode == ANIM_MODE_INTERRUPT || !anim_state.animation_active) {
    // Start animation immediately - reset all state
    anim_state.current_event = new_event;
    anim_state.animation_active = true;
    anim_state.next_frame_time = millis();
    anim_state.frame_counter = 0;
    anim_state.step_counter = 0;  // Always reset step counter for clean start
    
    if (type == ANIM_RAINBOW) {
      Serial.print("🌈 Starting RAINBOW animation with ");
      Serial.print(wait);
      Serial.print("ms delay, ");
      Serial.print(repeat_count);
      Serial.println(" cycles");
    }
    
    return true;
  } 
  else if (mode == ANIM_MODE_QUEUE) {
    // Add to queue if there's space
    if (anim_queue.count < MAX_ANIMATION_QUEUE) {
      anim_queue.events[anim_queue.tail] = new_event;
      anim_queue.tail = (anim_queue.tail + 1) % MAX_ANIMATION_QUEUE;
      anim_queue.count++;
      return true;
    }
  }
  
  return false; // Queue full or other error
}

void stopCurrentAnimation() {
  anim_state.animation_active = false;
  anim_state.current_event.type = ANIM_NONE;
  strip.clear();
  strip.show();
}

bool isAnimationActive() {
  return anim_state.animation_active;
}

void processNextQueuedAnimation() {
  if (anim_queue.count > 0) {
    // Get next animation from queue
    AnimationEvent next_event = anim_queue.events[anim_queue.head];
    anim_queue.head = (anim_queue.head + 1) % MAX_ANIMATION_QUEUE;
    anim_queue.count--;
    
    // Start the animation with clean state
    anim_state.current_event = next_event;
    anim_state.animation_active = true;
    anim_state.next_frame_time = millis();
    anim_state.frame_counter = 0;
    anim_state.step_counter = 0;  // Reset step counter for clean start
  }
}

void updateAnimations() {
  if (!anim_state.animation_active) {
    processNextQueuedAnimation();
    return;
  }
  
  if (millis() >= anim_state.next_frame_time) {
    processAnimationFrame();
  }
}

void processAnimationFrame() {
  AnimationEvent* event = &anim_state.current_event;
  
  switch (event->type) {
    case ANIM_CHASE:
      processChaseFrame();
      break;
    case ANIM_RAINBOW:
      processRainbowFrame();
      break;
    case ANIM_FLASH_SINGLE:
      processFlashSingleFrame();
      break;
    case ANIM_BLINK_ALL:
      processBlinkAllFrame();
      break;
    case ANIM_ALL_OFF:
      strip.clear();
      strip.show();
      anim_state.animation_active = false;
      break;
    case ANIM_FILL_COLOR:
      strip.fill(event->color1, 0, strip.numPixels());
      strip.show();
      anim_state.animation_active = false;
      break;
    default:
      anim_state.animation_active = false;
      break;
  }
}

// Async Chase Implementation
void processChaseFrame() {
  AnimationEvent* event = &anim_state.current_event;
  
  int start=-1;
  if (NEOPIXEL_LED_COUNT==7)
    start=addI(1,event->offset);
  else if (NEOPIXEL_LED_COUNT==6)
    start=addI(0,event->offset);
  else
    start=addI(0,event->offset);
    
  int current_step = anim_state.step_counter % strip.numPixels();
  
  // Clear and set new pattern
  strip.clear();
  
  int i = addI(start, current_step);
  strip.setPixelColor(addI(i,3), event->color1);
  strip.setPixelColor(addI(i,2), event->color2);
  strip.setPixelColor(addI(i,1), event->color3);
  strip.setPixelColor(addI(i,0), event->color4);
  
  strip.show();
  
  anim_state.step_counter++;
  anim_state.next_frame_time = millis() + event->wait;
  
  // Check if animation should continue
  if (current_step >= strip.numPixels() - 1) {
    anim_state.frame_counter++;
    anim_state.step_counter = 0;
    
    if (event->repeat_count >= 0 && anim_state.frame_counter >= event->repeat_count) {
      anim_state.animation_active = false;
    }
  }
}

// Async Rainbow Implementation 
void processRainbowFrame() {
  AnimationEvent* event = &anim_state.current_event;

  /*
  static uint32_t lastDebug = 0;
  if (millis() - lastDebug > 5000) {  // Every 5 seconds - reduce debug frequency
    Serial.print("Rainbow: step=");
    Serial.print(anim_state.step_counter);
    Serial.print("/1280 (");
    Serial.print((anim_state.step_counter * 100) / 1280);
    Serial.println("%)");
    lastDebug = millis();
  }*/
  
  long firstPixelHue = anim_state.step_counter * 256;
  
  for(int i = 0; i < strip.numPixels(); i++) {
    int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
  
  strip.show();
  
  anim_state.step_counter++;
  anim_state.next_frame_time = millis() + event->wait;
  
  // One complete cycle is 5*256 = 1280 steps
  if (anim_state.step_counter >= 1280) {
    anim_state.frame_counter++;
    anim_state.step_counter = 0;
    Serial.print("🌈 RAINBOW CYCLE COMPLETED! Frame #");
    Serial.print(anim_state.frame_counter);
    Serial.println(" - Starting new cycle");
    
    if (event->repeat_count >= 0 && anim_state.frame_counter >= event->repeat_count) {
      anim_state.animation_active = false;
      // Reset brightness after rainbow completes (like sync version)
      strip.setBrightness(daytimeBrightness);
      Serial.println("Rainbow animation finished, brightness reset");
    }
  }
}

// Async Flash Single Implementation
void processFlashSingleFrame() {
  AnimationEvent* event = &anim_state.current_event;
  
  if (anim_state.step_counter == 0) {
    // Advance pixel first (like sync version)
    singlePixelToToggle = (singlePixelToToggle + 1) % 8;
    // Flash on
    strip.setPixelColor(singlePixelToToggle, event->color1);
    strip.show();
    anim_state.next_frame_time = millis() + 100;
    anim_state.step_counter = 1;
  } else {
    // Flash off 
    strip.setPixelColor(singlePixelToToggle, strip.Color(0,0,0));
    strip.show();
    anim_state.next_frame_time = millis() + event->wait;
    anim_state.step_counter = 0;
    anim_state.frame_counter++;
    
    if (event->repeat_count >= 0 && anim_state.frame_counter >= event->repeat_count) {
      anim_state.animation_active = false;
    }
  }
}

// Async Blink All Implementation
void processBlinkAllFrame() {
  AnimationEvent* event = &anim_state.current_event;
  
  if (anim_state.step_counter == 0) {
    // Flash on - fill all pixels with color
    strip.fill(event->color1, 0, strip.numPixels());
    strip.show();
    anim_state.next_frame_time = millis() + 100;  // On for 100ms (like sync version)
    anim_state.step_counter = 1;
  } else {
    // Flash off - turn off all pixels
    strip.fill(strip.Color(0,0,0), 0, strip.numPixels());
    strip.show();
    anim_state.next_frame_time = millis() + 100;  // Off for 100ms (like sync version)
    anim_state.step_counter = 0;
    anim_state.frame_counter++;
    
    if (event->repeat_count >= 0 && anim_state.frame_counter >= event->repeat_count) {
      anim_state.animation_active = false;
    }
  }
}

void accumulateEnergyUsage()
{
  const  uint32_t energyCalcsDutyCycle = 1000;
  static uint32_t nextEnergyCalcDue = 0;

  if (millis() >= nextEnergyCalcDue)
  {
    nextEnergyCalcDue += energyCalcsDutyCycle;

    // Read voltage and current from INA219.
    shuntVoltage = currentSensor_ina219.getShuntVoltage_mV();
//    snprintf(sensorData,sizeof(sensorData),"Shunt Voltage Success: %d %f",currentSensor_ina219.success(),shuntVoltage);
//    Serial.print(sensorData);
//    Serial.println();

    busVoltage = currentSensor_ina219.getBusVoltage_V();
//    snprintf(sensorData,sizeof(sensorData),"Bus Voltage Success: %d %f",currentSensor_ina219.success(),busVoltage);
//    Serial.print(sensorData);
//    Serial.println();

    current_mA = currentSensor_ina219.getCurrent_mA();
//    snprintf(sensorData,sizeof(sensorData),"current_mA Success: %d %f",currentSensor_ina219.success(),current_mA);
//    Serial.print(sensorData);
//    Serial.println();


    power_mW_hardware = currentSensor_ina219.getPower_mW();
//    snprintf(sensorData,sizeof(sensorData),"power_mW Success: %d %f",currentSensor_ina219.success(),power_mW);
//    Serial.print(sensorData);
//    Serial.println();

    // Compute load voltage, power, and milliamp-hours.
    loadVoltage = busVoltage + (shuntVoltage / 1000.0);
    power_mW = loadVoltage * current_mA;

    total_mA += current_mA;
    powerOnSec += energyCalcsDutyCycle / 1000.0;

    const float dutyCyclesInOneHour = 3600000.0 / energyCalcsDutyCycle;
    total_mAH = total_mA / dutyCyclesInOneHour;

    max_current_ma = (current_mA > max_current_ma ? current_mA : max_current_ma);
    min_load_voltage = (loadVoltage < min_load_voltage ? loadVoltage : min_load_voltage);
    max_load_voltage = (loadVoltage > max_load_voltage ? loadVoltage : max_load_voltage);
  }
}
