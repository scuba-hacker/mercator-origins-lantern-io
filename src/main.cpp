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

void initializeI2C();

// https://kunkune.co.uk/shop/arduino-sensors/cjmcu-1080-high-precision-temperature-and-humidity-sensor-hdc1080/
// https://www.ti.com/lit/ds/symlink/hdc1080.pdf
// Uses TI HDC1080 IC
// I2C Address: 0x40
void readTempHumidityCJMCU_1080_Sensor(double* temperature, double* humidity);

// Current sensor GY-471 MAX471
// https://www.analog.com/en/products/max471.html

// https://devboards.info/boards/arduino-nano     (RX1)
// https://docs.arduino.cc/tutorials/nano-every/run-4-uart/

#define REED1_CLOSURE_EDGE_REBOOT_GPIO 4       // Connects to ground on close
#define REED2_SWITCH_SIDE_EDGE_POWER_OFF_ON_GPIO 5   // Connects to ground on close
#define ARDUINO_BOARD_LED 13
#define CIRCUIT_BREAKER_GPIO 15   // energises relay coil via NPN MOSFET
#define CURRENT_SENSOR_ANALOG_IN_GPIO  A6
#define VCC_HALVED_ANALOG_IN_GPIO      A7
#define LEAK_SENSOR_GPIO               LL   // leak at bottom of float
#define IGNORE_FLOAT_LEAK_GPIO         SS   // on-off switch, suppress leak alerts from float - 
                              // once blue leak sensors get wet they will probably stay that way

// HardwareSerial Serial1 TX_GPIO is on GPIO D0. Used to send a byte to Mako to say restarting.
// HardwareSerial Serial1 RX_GPIO is on GPIO D1. Used to receive status for LEDs from Mako.

int rawVccInSample, rawCurrentSample;

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

const bool lowPowerTest=true;

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
  Serial1.begin(9600);  // on TX output there is a 1k and 2k voltage divider to reduce voltage from 5V to 3.3V for ESP32

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

  pinMode(REED1_CLOSURE_EDGE_REBOOT_GPIO, INPUT_PULLUP);
  pinMode(REED2_SWITCH_SIDE_EDGE_POWER_OFF_ON_GPIO, INPUT_PULLUP);
  pinMode(CIRCUIT_BREAKER_GPIO, OUTPUT);
  digitalWrite(CIRCUIT_BREAKER_GPIO,LOW);

  analogReference(INTERNAL1V5);
  rawVccInSample = analogRead(VCC_HALVED_ANALOG_IN_GPIO);
  rawCurrentSample = analogRead(CURRENT_SENSOR_ANALOG_IN_GPIO);

  initializeI2C();

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels AS    AP
  
  strip.setBrightness(daytimeBrightness);

  globalOffset=0;
}

uint8_t byteFromLemon = 0xFF;

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

uint32_t timeOfNextLemonByteExpectedReceived = 10000;   // initialise at 10 seconds from startup
const uint32_t maximumWaitForLemonByteReceived = 60000; // if no comms from Lemon for 60 seconds then flash red to show no comms (LC_NONE)

e_lemon_status lastLemonStatus = LC_STARTUP;

e_lemon_status flushLemonStatus()
{
  while (Serial1.available())
    Serial1.read();
}

e_lemon_status readLemonStatus()
{
  e_lemon_status byteFromLemon = LC_NO_STATUS_UPDATE;

  while (Serial1.available())
  {
    byteFromLemon = (e_lemon_status) (Serial1.read());
    timeOfNextLemonByteExpectedReceived = millis() + maximumWaitForLemonByteReceived;
  }

  if (millis() > timeOfNextLemonByteExpectedReceived)
  {
    byteFromLemon = LC_NONE;
  }

  return byteFromLemon;
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

void loop()
{   
  // get temperature and humidity
  double temperature = 0;
  double humidity = 0;

  // readTempHumidityCJMCU_1080_Sensor(temperature, humidity);

  if (digitalRead(REED1_CLOSURE_EDGE_REBOOT_GPIO) == LOW)     // Restart / Reboot system
  {
    Serial1.write(100);   // send byte 100 to Lemon to indicate reboot initiated 
    Serial1.flush();
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
      // All green LEDs, disable the circuit breaker - turns on the rest of the system.
      Serial1.write(200);   // send byte 200 to Lemon to indicate power-up initiated 
      Serial1.flush();
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
      // All red LEDs, enable the circuit breaker - turns off the rest of the system.
      Serial1.write(200);  // send byte 200 to Lemon to indicate power-down initiated 
      Serial1.flush();
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

  if (lastLemonStatus == LC_STARTUP && millis() > 7000)
    lastLemonStatus = LC_NONE;

  int ledWait=100;

  if (lowPowerTest)
  {
    flashAndPauseNextPixel(strip.Color(10,10,0), 200);
  return;
  }

  // LC_DIVE_IN_PROGRESS flag reduces LED usage to save power
  switch(lastLemonStatus)
  {
    case LC_NONE:
    case LC_NONE | LC_DIVE_IN_PROGRESS:
    {
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
      break;
    }

    case LC_STARTUP:
    case LC_STARTUP | LC_DIVE_IN_PROGRESS:
    {
      strip.setBrightness(rainbowBrightness);
      rainbow(1);
      strip.setBrightness(daytimeBrightness);
      break;
    }
    case LC_SEARCH_WIFI:
    case LC_SEARCH_WIFI | LC_DIVE_IN_PROGRESS:
    {
      chase(globalOffset,
            strip.Color(0,0,255),   // BLUE
            strip.Color(0,0,100),
            strip.Color(0,0,50),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_FOUND_WIFI:
    case LC_FOUND_WIFI | LC_DIVE_IN_PROGRESS:
    {
      chase(globalOffset,
            strip.Color(255,50,255),   // CYAN
            strip.Color(100,50,100),
            strip.Color(50,50,50),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_NO_WIFI:
    {
      chase(globalOffset,
            strip.Color(0,255,255),   // MAGENTA
            strip.Color(0,100,100),
            strip.Color(0,50,50),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_NO_WIFI | LC_DIVE_IN_PROGRESS:
    {
      // Rotating single low brightness LED Magenta
      flashAndPauseNextPixel(strip.Color(0,10,10), 200);
      break;
    }
    case LC_NO_GPS:
    {
      chase(globalOffset,
            strip.Color(0,255,0),   // RED
            strip.Color(0,100,0),
            strip.Color(0,50,0),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_NO_GPS | LC_DIVE_IN_PROGRESS:
    {
      // Rotating single low brightness LED Red
      flashAndPauseNextPixel(strip.Color(0,10,0), 200);
      break;
    }
    case LC_NO_FIX:
    {
      chase(globalOffset,
            strip.Color(255,255,0),   // YELLOW
            strip.Color(100,100,0),
            strip.Color(50,50,0),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_NO_FIX | LC_DIVE_IN_PROGRESS:
    {
      // Rotating single low brightness LED Yellow
      flashAndPauseNextPixel(strip.Color(10,10,0), 200);
      break;
    }
    case LC_GOOD_FIX:
    {
      chase(globalOffset,
            strip.Color(255,0,0),   // GREEN
            strip.Color(100,0,0),
            strip.Color(50,0,0),
            strip.Color(0,0,0),
            ledWait);
      break;
    }
    case LC_GOOD_FIX | LC_DIVE_IN_PROGRESS:
    {
      // Rotating single low brightness LED Green
      flashAndPauseNextPixel(strip.Color(10,0,0), 200);
      break;
    }
    case LC_NO_INTERNET:
    {
      chase(globalOffset,
            strip.Color(30,255,80),   // BLUE BACKGROUND WITH MAGENTA CHASE
            strip.Color(30,100,80),
            strip.Color(30,50,80),
            strip.Color(30,0,80),
            ledWait);
      break;
    }
    case LC_NO_INTERNET | LC_DIVE_IN_PROGRESS:
    {
      // Rotating single low brightness LED Cyan
      flashAndPauseNextPixel(strip.Color(0,10,10), 200);
      break;
    }
    case LC_ALL_OFF:
    case LC_ALL_OFF | LC_DIVE_IN_PROGRESS:
    {
      strip.fill(strip.Color(0,0,0),0,8);
      strip.show();
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

void initializeI2C()
{
  Wire.begin();

  //Configure HDC1080
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  Wire.write(0x90);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(20);
}

void readTempHumidityCJMCU_1080_Sensor(double* temperature, double* humidity)
{
  //holds 2 bytes of data from I2C Line
  uint8_t Byte[4];

  uint16_t temp;
  uint16_t humid;

  //Point to device 0x40 (Address for HDC1080)
  Wire.beginTransmission(0x40);

  //Point to register 0x00 (Temperature Register)
  Wire.write(0x00);

  //Relinquish master control of I2C line
  //Pointing to the temp register triggers a conversion
  Wire.endTransmission();
  
  delay(20);                  // Allow for sufficient conversion time

  Wire.requestFrom(0x40, 4);  // Request four bytes from registers
  delay(1);

  //If the 4 bytes were returned sucessfully
  if (4 <= Wire.available())
  {
    Byte[0] = Wire.read();    // upper byte of temp reading
    Byte[1] = Wire.read();    // lower byte of temp reading
    Byte[3] = Wire.read();    // upper byte of humidity reading
    Byte[4] = Wire.read();    // lower byte of humidity reading

    temp = (((unsigned int)Byte[0] <<8 | Byte[1]));
    *temperature = (double)(temp)/(65536)*165-40;

    humid = (((unsigned int)Byte[3] <<8 | Byte[4]));
    *humidity = (double)(humid)/(65536)*100;
  }
}
