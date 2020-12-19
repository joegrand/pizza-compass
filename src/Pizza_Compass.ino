/*
 * Project: Pizza Compass
 * Author: Joe Grand (@joegrand), Grand Idea Studio (www.grandideastudio.com)
 * Description: Digital compass that leads you to the nearest location of your choosing 
 * no matter where you are in the world.
 * License: Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)
 */

/* LED states:
 * White: LED test at power-up
 * Spinning red: Connecting to Particle network
 * Spinning green: Waiting for valid GPS fix
 * Spinning blue: Waiting for compass calibration (short press to skip, press and hold to calibrate)
 * Solid blue: Compass calibration (spin around each axis like you're painting the inside of a sphere, press button when done)
 * Flashing blue: Compass error (can't communicate)
 * Rainbow/color wheel: Waiting to begin
 * Solid purple: Sending data request to Particle (local coordinates + search query)
 * Spinning purple: Waiting for response from Particle (target coordinates)
 */

/* ======================= Includes ================================= */

#include "Particle.h"

// External libraries
#include "neopixel.h"               // Adafruit Neopixel
#include "LSM303.h"                 // Pololu LSM303
#include "TinyGPS++.h"              // GPS NMEA parsing

/* ======================= Constants =============================== */

SYSTEM_MODE(AUTOMATIC);                   // Connect to Particle network on start
SYSTEM_THREAD(ENABLED);                   // Enable multithreading (application and system loops execute in parallel)

#define PARTICLE_PUBLISH_TIME   10000     // Minimum time (in ms) allowed between Particle.publish events
#define PARTICLE_CONSOLE_BAUD   115200

/*
 * Only these pins can be used for Neopixel by the Particle Boron:
 * - D2, D3, A4, A5
 * - D4, D6, D7, D8
 * - A0, A1, A2, A3
 */
#define PIXEL_PIN           D2
#define PIXEL_COUNT         8
#define PIXEL_TYPE          WS2812B
#define PIXEL_BRIGHTNESS    32     // 0 = low, 255 = high
#define PIXEL_CYCLE_SPEED   5      // Speed of rainbow color wheel (in ms)
#define PIXEL_WIPE_SPEED    50     // Speed of pixel wipe effect

#define COMPASS_CAL_DELAY   3000   // Length of button press required to enter calibration mode on start-up (in ms)

// Hard-coded GPS coordinates
// https://www.gps-coordinates.net/
#define GPS_LOCAL_LAT       40.6892532      // Statue of Liberty
#define GPS_LOCAL_LNG       -74.0445482

#define GPS_TARGET_LAT      37.8184509      // Golden Gate Bridge
#define GPS_TARGET_LNG      -122.4784088

#define GPS_BAUD            9600
#define GPS_LOCATION_AGE    5000    // Time (in ms) before data is considered stale (e.g., we may have lost fix)

#define UI_INTERVAL_TIME    100     // Time (in ms) to wait between GPS/compass updates
#define UI_DISTANCE_ALERT   50.0    // Distance to target (in meters) in order to enable visual alert
#define UI_BLINK_TIME       250     // Blink time (in ms) for visual alert when we're within range of target


/* ==================== Global Variables ============================ */

// Pin definitions
const int ledBoron = D7;        // Particle Boron LTE on-board LED
const int buttonStart = D3;     // External pushbutton
const int gpsEnable = D8;       // GPS Enable pin (active HIGH)

// UI
bool first_time = true;         // If we haven't searched for pizza yet
bool blinkFlag = false;         // Blink LEDs if we are within range of target
unsigned long lastUpdateTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastBlinkTime = 0;

// Neopixel LEDs
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// LSM303 Magnetometer (compass)
LSM303 compass;

// GPS
TinyGPSPlus gps;
double local_lat, local_lng;
double target_lat, target_lng;
double courseToPizza;   // Heading (in degrees) from our current location to the target
double distanceToPizza; // Distance (in meters) from our current location to the target
int direction_index;

// Particle
bool requestSent = false;
bool requestRcv = false;
void hookResponseHandler(const char *event, const char *data);


/* ======================= Functions =============================== */

// setup() runs once, when the device is first turned on.
void setup() 
{
  // Pin configuration
  pinMode(ledBoron, OUTPUT);
  pinMode(gpsEnable, OUTPUT);
  pinMode(buttonStart, INPUT);

  digitalWrite(gpsEnable, 1);  // Make sure the GPS module is active

  Serial.begin(PARTICLE_CONSOLE_BAUD);
  delay(5000);  // Give time for serial monitor to open

  Serial.println("\n\nWelcome to Joe Grand's Pizza Compass!\n");

  Serial.print("[*] Initializing Neopixels...");
  strip.begin();
  strip.clear();
  strip.show();
  strip.setBrightness(PIXEL_BRIGHTNESS);
  Serial.println("Done!");

  // Wait for Particle to connect to the network
  LED_SetAll(strip.Color(255, 255, 255), 0); // All LEDs on immediately after power-up
  delay(5000);
  LED_SetAll(strip.Color(0, 0, 0), 0); // Off

  Serial.print("[*] Connecting to Particle network...");
  while (Particle.connected() == false)
  {
    LED_ColorWipe(strip.Color(255, 0, 0), PIXEL_WIPE_SPEED); // Red
    LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off
  }
  Serial.println("Done!");

  // Wait for valid GPS fix
  Serial.print("[*] Waiting for GPS...");
  Serial1.begin(GPS_BAUD);
  while (!Serial1);   // Wait until the serial port is ready
  gps.encode(Serial1.read()); 
  while (gps.location.isValid() == false)   // Wait until the valid first NMEA sentence with a fix is parsed
  {
    LED_ColorWipe(strip.Color(0, 255, 0), PIXEL_WIPE_SPEED); // Green
    LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off

    while (Serial1.available() > 0)  // Wait until there is data in the serial buffer
    {
      gps.encode(Serial1.read()); 
    }
  }
  //GPS_DisplayInfo();
  Serial.println("Done!");

  Serial.print("[*] Initializing magnetometer...");
  Wire.begin();
  bool error = compass.init();	 // Automatically detect type of LSM303
  if (!error)
  {
    Serial.println("Failed!");
    Compass_Error();             // Cannot detect the LSM303
  }
  else
  {
	  compass.enableDefault();     // Initilize accelerometer and magnetometer
    Serial.println("Done!");
  }

  Serial.print("[*] Calibrating magnetometer...");
  while (digitalRead(buttonStart) == 1) // Wait here for a button press
  {
    LED_ColorWipe(strip.Color(0, 0, 255), PIXEL_WIPE_SPEED); // Blue
    LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off
  }

  // Calibrate the compass
  Compass_Warmup();   // Discard initial values

  unsigned long start_time = millis();
  bool new_cal = false;  // If button press is short, skip calibration and use default values
  while (digitalRead(buttonStart) == 0)
  {
    if (millis() - start_time > COMPASS_CAL_DELAY)  // If button is held down for long enough...
    {
      LED_ColorWipe(strip.Color(0, 0, 255), PIXEL_WIPE_SPEED); // Blue
      new_cal = true;  // Read new compass values to calculate minimum/maximum ranges
    }
  }
  Compass_Calibration(new_cal);   
  delay(100);
  if (new_cal)
  {
    Serial.println("Done!");
  }
  else
  {
    Serial.println("Skipped!");
  }
  LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off
  Compass_DisplayCalibration();

  Serial.print("[*] Subscribing to Particle integration response event...");
  if (Particle.subscribe("hook-response/get_pizza", hookResponseHandler, MY_DEVICES) == true)
  {
    Serial.println("Done!");
  }
  else
  {
    Serial.println("Failed!");
  }
  
  while(digitalRead(buttonStart) == 0);  // Wait until button is released
  delay(100);

  // We're all set up and good to go!
  Serial.print("[*] Waiting for button...");
}

/* ---------------------------------------------------------------- */

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{
  while (first_time == true)   // Wait here until we begin pizza search for the first time
  {
      LED_RainbowCycle(PIXEL_CYCLE_SPEED);

      if (digitalRead(buttonStart) == 0)  // If the button is pressed
      {
        Serial.println("Done!");
        first_time = false;
      }
  }
 
  // At this point, we assume the GPS retains a valid fix (no error checking for invalid/stale data)
  while (Serial1.available() > 0)  // If characters have been received from the GPS...
  {
    gps.encode(Serial1.read());      // ...Push them into the GPS object
  }

  if (digitalRead(buttonStart) == 0)  // Get coordinates of target based on our current location
  {
    delay(100); // Debounce
    if (digitalRead(buttonStart) == 0) // If the button is still pressed...
    {
      if (requestSent == false && (millis() - lastPublishTime >= PARTICLE_PUBLISH_TIME))  // If we haven't sent a request recently
      {
        lastPublishTime = millis();
        LED_ColorWipe(strip.Color(255, 0, 255), PIXEL_WIPE_SPEED); // Purple

        while(digitalRead(buttonStart) == 0);  // Wait until button is released
        delay(100);

        Serial.printlnf("[*] Sending request to Particle...");

        // Get current local GPS coordinates
        local_lat = gps.location.lat();
        local_lng = gps.location.lng();
        
        // Prepare data for query
        // https://github.com/rickkas7/particle-webhooks
        /* Set up Webhook custom template in Particle.io dashboard
        * {
        *  "event": "get_pizza",
        *  "deviceID": "<YOUR_DEVICE_NAME>",
        *  "url": "https://maps.googleapis.com/maps/api/place/textsearch/json",
        *  "requestType": "GET",
        *  "noDefaults": true,
        *  "rejectUnauthorized": true,
        *  "responseTemplate": "{{results.0.geometry.location.lat}},{{results.0.geometry.location.lng}},{{results.0.name}}",
        *  "query": 
        *   {
        *     "query": "pizza",
        *     "location": "{{lat}},{{lng}}",
        *     "key": "<YOUR_GOOGLE_API_KEY>"
        *   }
        * }
        */
        requestSent = true;
        requestRcv = false; // This flag will be set when a response is received from the Webhook
        char data[256];
        snprintf(data, sizeof(data), "{\"lat\":%.10g,\"lng\":%.10g}", local_lat, local_lng);
        Serial.printlnf("-> Data: %s", data);
        Particle.publish("get_pizza", data, PRIVATE);

        LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off
      }
    }
  }
  else  // Display heading for us to follow
  {
    if (Particle.connected() == true && requestSent == true && requestRcv == false)  // If data has just been sent to Particle
    {
      LED_ColorWipe(strip.Color(255, 0, 255), PIXEL_WIPE_SPEED); // Purple
      LED_ColorWipe(strip.Color(0, 0, 0), PIXEL_WIPE_SPEED); // Off
    }

    if (requestRcv == true && (millis() - lastUpdateTime >= UI_INTERVAL_TIME))  // If we've received target coordinates back from Particle/Google API
    {
      lastUpdateTime = millis();
      
      requestSent = false;  // Clear flag so we can perform another request in the future
      Serial.printlnf("[*] Let's go!");

      // Get current local GPS coordinates
      local_lat = gps.location.lat();
      local_lng = gps.location.lng();
      // target_lat and target_lng already exist via hookResponseHandler

      // Force hard-coded coordinates here for testing
      //local_lat = GPS_LOCAL_LAT;
      //local_lng = GPS_LOCAL_LNG;
      //target_lat = GPS_TARGET_LAT;
      //target_lng = GPS_TARGET_LNG;

      Serial.printlnf("-> Local: %.10g,%.10g", local_lat, local_lng);
      Serial.printlnf("-> Target: %.10g,%.10g", target_lat, target_lng);

      distanceToPizza = 
      TinyGPSPlus::distanceBetween(
        local_lat,
        local_lng,
        target_lat, 
        target_lng);

      courseToPizza = 
      TinyGPSPlus::courseTo(
        local_lat,
        local_lng,
        target_lat, 
        target_lng);

      Serial.printlnf("-> Distance: %.6g meters", distanceToPizza);
      Serial.printf("-> Course: %.6g [", courseToPizza);
      Serial.print(TinyGPSPlus::cardinal(courseToPizza));
      Serial.printlnf("]");

      /*
        When given no arguments, the heading() function returns the angular
        difference in the horizontal plane between a default vector and
        north, in degrees.
            
        To use a different vector as a reference, use the version of heading()
        that takes a vector argument; for example, use
            
          compass.heading((LSM303::vector<int>){0, 0, 1});
            
        to use the +Z axis as a reference.
      */
      compass.read();  // Read new compass values
      float heading = compass.heading((LSM303::vector<int>){1, 0, 0});  // Get heading using the +X axis as our reference
      Serial.printlnf("-> Current Heading: %d", (int)heading);

      int courseChangeNeeded = (int)(360 + courseToPizza - heading) % 360;
      Serial.printlnf("-> Course Change: %d", courseChangeNeeded);

      // Divide to find which pie slice it's in (360 degrees / number of pixels)
      direction_index = courseChangeNeeded / (360 / PIXEL_COUNT);

      /*char report[80];
      snprintf(report, sizeof(report), "-> A: %6d %6d %6d\n-> M: %6d %6d %6d", compass.a.x, compass.a.y, compass.a.z, compass.m.x, compass.m.y, compass.m.z);
      Serial.println(report);*/
      Serial.printlnf("-> LED: %d", direction_index);

      unsigned long currentBlinkTime = millis();
      if (distanceToPizza <= UI_DISTANCE_ALERT)   // If we are within range of our target...
      {
        if (currentBlinkTime - lastBlinkTime >= UI_BLINK_TIME)
        {
          lastBlinkTime = currentBlinkTime;
          if (blinkFlag == false)
          {
            blinkFlag = true;
          }
          else
          {
            blinkFlag = false;
          }
        }
      }
      else
      {
        blinkFlag = false;
      }
        
      // Light the pixel for the direction we need to go
      for (int i = 0; i < strip.numPixels(); i++) 
      {
        if (i == direction_index && blinkFlag == false)
        {
          if (direction_index == 0) // If we're facing in the direction of our target
          {
            strip.setPixelColor(i, strip.Color(0, 255, 0)); // Green
          }
          else
          {
            strip.setPixelColor(i, strip.Color(255, 0, 0)); // Red
          }
        }
        else
          strip.setPixelColor(i, strip.Color(0, 0, 0));   // Turn off all others
      }

      strip.show();
    }
  }
}

/* ------------------------ Particle.io ---------------------------- */

void hookResponseHandler(const char *event, const char *data) 
{
  // Handle the integration response
  // https://rickkas7.github.io/mustache/
  char input_string[256];

  requestRcv = true; 

  // Parse the received data
  memcpy(input_string, data, sizeof(input_string));

  Serial.println("[*] Target acquired...");
  //Serial.printf("-> Raw: %s", input_string); 
  //Serial.println("");

	target_lat = atof(strtok(input_string, ","));    // Extract first string from string sequence
  target_lng = atof(strtok(NULL, ","));            // Extract second string from string sequence

  Serial.printlnf("-> Name: %s", strtok(NULL, ",")); // Extract remainder of string sequence
	Serial.printlnf("-> Target lat: %.10g", target_lat);
	Serial.printlnf("-> Target long: %.10g", target_lng);
}        

/* ------------------------ Neopixel ------------------------------ */

// Rainbow color wheel, makes the rainbow equally distributed, then wait (ms)
void LED_RainbowCycle(unsigned long wait) 
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)  // 1 cycle of all colors on wheel
  { 
    for (i = 0; i < strip.numPixels(); i++) 
    {
      strip.setPixelColor(i, LED_Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    
    strip.show();
    delay(wait);
  }
}

/* ---------------------------------------------------------------- */

// Set all pixels in the strip to a solid color, then wait (ms)
void LED_SetAll(uint32_t c, unsigned long wait) 
{
  uint16_t i;

  for(i = 0; i < strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, c);
  }
  
  strip.show();
  delay(wait);
}

/* ---------------------------------------------------------------- */

// Fill the dots one after the other with a color, wait (ms) after each one
void LED_ColorWipe(uint32_t c, unsigned long wait) 
{
  uint16_t i;

  for (i = 0; i < strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

/* ---------------------------------------------------------------- */

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t LED_Wheel(byte WheelPos) 
{
  if(WheelPos < 85) 
  {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) 
  {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else 
  {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

/* ------------------------ Magnetometer --------------------------- */
// Code based on https://github.com/pololu/lsm303-arduino

// Calibrate sensor by finding the range of X, Y, and Z values 
// for the accelerometer and magnetometer. This is done by rotating 
// the device around each axis.
void Compass_Calibration(bool read_new_values)
{
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

  // Set default values in case user skips calibration (not recommended)
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};

  if (read_new_values)
  {
    // Continually get readings until the button is pressed
    while (digitalRead(buttonStart) == 1)
    {
      compass.read();  // Read all 6 channels of the LSM303 and stores them in the object variables
  
      // Set minimum and maximum values of magnetometer readings
      if (compass.m.x != 0.0 && compass.m.y != 0.0 && compass.m.z != 0)  // Ignore zero readings
      {
        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);
        running_min.z = min(running_min.z, compass.m.z);

        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);
        running_max.z = max(running_max.z, compass.m.z);
    
        /*char report[80];
        snprintf(report, sizeof(report), "min: {%6d, %6d, %6d}  max: {%6d, %6d, %6d}", running_min.x, running_min.y, running_min.z, running_max.x, running_max.y, running_max.z);		  
        Serial.println(report);*/
      }
      
      delay(5);
    }

    delay(100);
    while (digitalRead(buttonStart) == 0) // Wait until button is released
    delay(100); 

    // Update global variables with calibrated results
    compass.m_min = running_min;
    compass.m_max = running_max;
  }
}

/* ---------------------------------------------------------------- */

void Compass_DisplayCalibration(void)
{
  Serial.printlnf("-> Max Value (X): %6d", compass.m_max.x);
  Serial.printlnf("-> Min Value (X): %6d", compass.m_min.x);

  Serial.printlnf("-> Max Value (Y): %6d", compass.m_max.y);
  Serial.printlnf("-> Min Value (Y): %6d", compass.m_min.y);

  Serial.printlnf("-> Max Value (Z): %6d", compass.m_max.z);
  Serial.printlnf("-> Min Value (Z): %6d", compass.m_min.z);
}

/* ---------------------------------------------------------------- */

void Compass_Warmup(void) 
{
  // Discard samples on power up
  for (int ignore = 0; ignore < 100; ignore++) 
  {
    compass.read();  // Read all 6 channels of the LSM303
    delay(10);
  }
}

/* ---------------------------------------------------------------- */

void Compass_Error(void)
{
  while (1)
  {
    // Blink LEDs to indicate failure
    // Without a compass, we don't have much of a way to find pizza
    LED_SetAll(strip.Color(0, 0, 255), 500);  // Blue
    LED_SetAll(strip.Color(0, 0, 0), 500);    // Off
  }
}

/* ----------------------------- GPS ------------------------------- */

void GPS_DisplayInfo(void)
{
  Serial.print(F("-> Local: ")); 
  if (gps.location.isValid() == false)
  {
    Serial.print(F("INVALID! "));
  }
  Serial.print(gps.location.lat(), 6);
  Serial.print(F(","));
  Serial.print(gps.location.lng(), 6);

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid() == false)
  {
    Serial.print(F("INVALID! "));
  }
  Serial.print(gps.date.month());
  Serial.print(F("/"));
  Serial.print(gps.date.day());
  Serial.print(F("/"));
  Serial.print(gps.date.year());

  Serial.print(F(" "));
  if (gps.time.isValid() == false)
  {
    Serial.print(F("INVALID! "));
  }
  if (gps.time.hour() < 10) Serial.print(F("0"));
  Serial.print(gps.time.hour());
  Serial.print(F(":"));
  if (gps.time.minute() < 10) Serial.print(F("0"));
  Serial.print(gps.time.minute());
  Serial.print(F(":"));
  if (gps.time.second() < 10) Serial.print(F("0"));
  Serial.print(gps.time.second());
  Serial.print(F("."));
  if (gps.time.centisecond() < 10) Serial.print(F("0"));
  Serial.print(gps.time.centisecond());

  Serial.println();
}

/* ------------------------ End of code! -------------------------- */
