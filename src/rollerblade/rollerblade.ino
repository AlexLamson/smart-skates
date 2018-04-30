#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// pins used
#define LEFT_PRESSURE_SENSOR 6
#define RIGHT_PRESSURE_SENSOR 7
#define LEFT_HALL_SENSOR 4
#define RIGHT_HALL_SENSOR 5
#define LEFT_NEOP 14
#define RIGHT_NEOP 15

//pressure variables
boolean left_stepped_on = false;
boolean right_stepped_on = false;

// neopixel variables
#define PIXEL_COUNT 30
#define PIXEL_INNER_COUNT 12

// for connecting to blynk and temboo
char auth[] = "128c8d58f05d4913be0a340473683013"; //blynk auth token
BlynkTimer timer;

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "CS390N";
//char pass[] = "internetofthings";
char ssid[] = "NETGEAR50";
char pass[] = "magicalfire545";


// variables about physical entities
const float circum = 251.327412; // mm - circumference of wheels
float inter_pixel_distance = 1000.0 / 60; // mm - distance between neopixels


//statistics variables
float max_speed = 0;
float avg_speed = 0; //running average
unsigned long statistic_ticks = 0; //number of speeds that have been recorded


// general variables for updating lights at intervals
byte tickMillis = 20;
unsigned long lastTickMillis = 0;
CRGB left_leds[PIXEL_COUNT];
CRGB right_leds[PIXEL_COUNT];

//~~~~~~~~~~~~~~~~~~~~~~~~~
// pattern variables
//~~~~~~~~~~~~~~~~~~~~~~~~~

byte pattern = 1; // the current pattern - {fixed light, fixed rainbow, speed=brightness, speed=hue, tail lights, onground, footstep hue, footstep brightness}

// variables for fixed light speed-dependent pattern
CRGB color = 0x1144ff;

float position = 0;
float inter_light_distance = inter_pixel_distance * PIXEL_COUNT / 2; // mm - distance for the logical lights to display
float logical_light_distance = inter_light_distance / inter_pixel_distance; // what the program uses to draw the lights

// fixed rainbow
float hue_position = 0;
float hue_per_meter = 255;
float hue_per_pixel = inter_pixel_distance * hue_per_meter / 1000;

// speed = brightness
CRGB color_p2 = 0xff0000;
float bright_speed = 3; // m/s - the speed at which the lights are brightest

// speed = hue
byte hue_at_zero = 0;
float rainbow_speed = 5; // speed to go all the way around the hue cycle

// tail lights
float last_speed = 0;
float decel_threshold = 0.05; // m/s


class WheelSpeed {
  public:
    WheelSpeed(int);
    void update();
    float get_speed(); // meters per second
    
  private:
    int pin;
    float speed;
    unsigned long last_fall;
    bool hall_last_value;

    void update_speed();
};

WheelSpeed left_wheel = WheelSpeed(LEFT_HALL_SENSOR);
WheelSpeed right_wheel = WheelSpeed(RIGHT_HALL_SENSOR);

void drawSmoothedPixel(CRGB*, int, int, float, CRGB);
byte mapBrightness(float);
void update_speed_stats(float);
void myTimerEvent();
CRGB scaleColor(CRGB, byte);

void setup() {
  Serial.begin(115200);

  FastLED.addLeds<NEOPIXEL, LEFT_NEOP>(left_leds, PIXEL_COUNT);
  FastLED.addLeds<NEOPIXEL, RIGHT_NEOP>(right_leds, PIXEL_COUNT);

  Serial.println("trying to connect to wifi");
  Blynk.begin(auth, ssid, pass);
  Serial.println("wifi connected");

  timer.setInterval(1000L, myTimerEvent);

  // pull up some pins
  pinMode(RIGHT_HALL_SENSOR, INPUT_PULLUP);
  pinMode(LEFT_HALL_SENSOR, INPUT_PULLUP);
  pinMode(LEFT_PRESSURE_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_PRESSURE_SENSOR, INPUT_PULLUP);
}

void loop() {
  // check the wheel speeds
  left_wheel.update();
  right_wheel.update();

  // check if the skate is being stepped on
  left_stepped_on = (digitalRead(LEFT_PRESSURE_SENSOR) == LOW);
  right_stepped_on = (digitalRead(RIGHT_PRESSURE_SENSOR) == LOW);

  // update blynk stuff
  Blynk.run();
  timer.run();

  //update some statistics
  float left_wheel_speed = left_wheel.get_speed();
  float right_wheel_speed = right_wheel.get_speed();
  float avg_speed = (left_wheel_speed + right_wheel_speed)/2.0;
  update_speed_stats(avg_speed);

  // draw patterns
  if (millis() >= lastTickMillis + tickMillis) {
    lastTickMillis = millis();

    switch (pattern) {
      case 1: // fixed light
      {
        position += (avg_speed * tickMillis) / inter_pixel_distance;
        if (position >= logical_light_distance) { position -= logical_light_distance; } // can't mod floats
    
        FastLED.clear(); // clear both strips
        
        // draw lights at intervals
        for (float f = position - logical_light_distance; f < PIXEL_COUNT - PIXEL_INNER_COUNT + 1; f += logical_light_distance) {
          drawSmoothedPixel(left_leds, 0, PIXEL_INNER_COUNT, PIXEL_INNER_COUNT - f, color);
          drawSmoothedPixel(left_leds, PIXEL_INNER_COUNT, PIXEL_COUNT, f + PIXEL_INNER_COUNT, color);
  
          drawSmoothedPixel(right_leds, 0, PIXEL_INNER_COUNT, PIXEL_INNER_COUNT - f, color);
          drawSmoothedPixel(right_leds, PIXEL_INNER_COUNT, PIXEL_COUNT, f + PIXEL_INNER_COUNT, color);
        }
      
        break;
      }

      case 2: // fixed rainbow
      {
        hue_position += (avg_speed * tickMillis) / inter_pixel_distance;
        if (hue_position >= 255) { position -= 255; } // can't mod floats
        
        for (int i = 0; i < PIXEL_COUNT - PIXEL_INNER_COUNT; i++) {
          byte h = byte(hue_position + i * hue_per_pixel);
          if (i < PIXEL_INNER_COUNT) {
            left_leds[PIXEL_INNER_COUNT - i - 1] = CHSV( h, 255, 255 );
            right_leds[PIXEL_INNER_COUNT - i - 1] = CHSV( h, 255, 255 );
          }
          left_leds[PIXEL_INNER_COUNT + i] = CHSV( h, 255, 255 );
          right_leds[PIXEL_INNER_COUNT + i] = CHSV( h, 255, 255 );
        }
        break;
      }

      case 3: // speed = brightness
      {
        byte left_bright = byte(255 * _min(1.0f, avg_speed / bright_speed)); // later, get the correct aggregate speed
        byte right_bright = byte(255 * _min(1.0f, avg_speed / bright_speed)); // later, get the correct aggregate speed
        fill_solid(left_leds, PIXEL_COUNT, scaleColor(color_p2, left_bright));
        fill_solid(right_leds, PIXEL_COUNT, scaleColor(color_p2, right_bright));
  
        break;
      }
      
      case 4: // speed = hue
      {
        byte left_h = byte(255 * left_wheel_speed / rainbow_speed);
        left_h += hue_at_zero;

        byte right_h = byte(255 * right_wheel_speed / rainbow_speed);
        right_h += hue_at_zero;

        fill_solid(left_leds, PIXEL_COUNT, CHSV(left_h, 255, 255));
        fill_solid(right_leds, PIXEL_COUNT, CHSV(right_h, 255, 255));
        break;
      }

      case 5: // tail lights
      {
        FastLED.clear();
        left_leds[PIXEL_COUNT - 1].r = 255; // running lights
        right_leds[PIXEL_COUNT - 1].r = 255; // running light
  
        int curr_speed = avg_speed;
  
        if (last_speed - curr_speed > decel_threshold) {
          const int upto = 2;
          for (int i = 0; i < upto; i++) {
            left_leds[PIXEL_COUNT - 2 - i].r = 255;
            right_leds[PIXEL_COUNT - 2 - i].r = 255;
          }
        }

        last_speed = curr_speed;
        break;
      }

      case 9: // debug
      {
        FastLED.clear();
        byte left_bright = byte(255 * _min(1.0f, left_wheel_speed / bright_speed));
        byte right_bright = byte(255 * _min(1.0f, right_wheel_speed / bright_speed));
        
        byte g = 0;
        if(left_stepped_on)
          g = 255;

        byte b = 0;

        fill_solid(left_leds, PIXEL_COUNT, CRGB(left_bright, g, b));
        fill_solid(right_leds, PIXEL_COUNT, CRGB(right_bright, g, b));
        break;
      }
    } //end of switch case
    
    FastLED.show();
  }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Wheel speed code
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

WheelSpeed::WheelSpeed(int pin) {
  last_fall = 0;

  this->pin = pin;
  hall_last_value = digitalRead(pin);

  speed = 0;
}

void WheelSpeed::update() {
  bool reading = digitalRead(pin);
  if (reading != hall_last_value) {

    if (reading == LOW) { // falling edge
      update_speed();
      last_fall = millis();
    }

    hall_last_value = reading;
  }
}

void WheelSpeed::update_speed() {
  long time_diff = millis() - last_fall; //inter-valley duration
  
  speed = circum/time_diff; //(in m/s) they are both in milli- units, so they cancel each other out
}

float WheelSpeed::get_speed() { // returns meters per second
  long time_diff = millis() - last_fall; //inter-valley duration
  
  if (time_diff > circum / speed) { // the wheel is slowing down since we haven't seen it in a while
    if (time_diff > 1000) {
      speed = 0;
    } else {
      update_speed();
    }
  }

  return speed;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Patterns code
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void drawSmoothedPixel(CRGB* leds, int start, int end, float position, CRGB color) {
  for (int i = int(position - 1); i <= int(position + 1); i++) {
    if (i < 0 || i < start || i >= end) { continue; }
    
    float normBrightness = _max(0.0, 1.0-float(abs(i - position + 0.5))*2.0/3); // kernel that's 2 pixels wide, requires at most 3 pixels to show
    byte brightness = 255*(0.5-cos( PI * pow(normBrightness, 1.5) )/2); // power of 1.5 seems to preserve brightness pretty well.
    
    leds[i] = scaleColor(color, brightness);
  }
}
CRGB scaleColor(CRGB color, byte brightness) {
  return CRGB( scale8(color.r, brightness), scale8(color.g, brightness), scale8(color.b, brightness) );
}

// input within [0, 1]
byte mapBrightness(float input) {
  return 255*(0.5-cos( PI * pow(input, 1.5) )/2); // power of 1.5 seems to preserve brightness pretty well.
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Blynk code
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/*
 * Virtual pins layout
 * -------------------
 * V0 - upload stats button
 * V1 - pattern selector
 * V2 - max speed text box
 * V3 - total time text box
 * V4 - average speed
 * V5 - left skate speed
 * V6 - right skate speed
 */

//upload stats button
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt();
  boolean button_down = (pinValue == HIGH);
  if(button_down) {
    Serial.println("Upload stats button pressed");
//    Serial.println("Uploading stats to spreadsheet");
  }
}

//pattern selector
BLYNK_WRITE(V1) {
  pattern = param.asInt();
  switch (pattern)
  {
    case 1:
      Serial.println("pattern: fixed lights");
      break;
    case 2:
      Serial.println("pattern: fixed rainbow");
      break;
    case 3:
      Serial.println("pattern: brightness=speed");
      break;
    case 4:
      Serial.println("pattern: hue=speed");
      break;
    case 5:
      Serial.println("pattern: brakes");
      break;
    case 6:
      Serial.println("pattern: lights on when pressure");
      break;
    case 7:
      Serial.println("pattern: step=add hue delta");
      break;
    case 8:
      Serial.println("pattern: step=increase brightness");
      break;
    case 9:
      Serial.println("pattern: debug: red=speed, green=pressure");
      break;
    default:
      Serial.println("pattern: Unknown pattern code (defaulting to fixed lights)");
      pattern = 1;
  }
}

//blynk timer called every 1 second
void myTimerEvent()
{
  //max speed
  Blynk.virtualWrite(V2, max_speed);

  //total time spent skating
  Blynk.virtualWrite(V3, millis() / 1000);

  
//  float left_speed = left_wheel.get_speed();
  float left_speed = 0.0;
  Blynk.virtualWrite(V5, left_speed);
  
  float right_speed = right_wheel.get_speed();
  Blynk.virtualWrite(V6, right_speed);

  float avg_of_both_skates = (left_speed + right_speed)/2.0;
  Blynk.virtualWrite(V4, avg_of_both_skates);

  Serial.print("left speed: ");
  Serial.print(left_speed);
  Serial.print(" ");
  Serial.print("right speed: ");
  Serial.print(right_speed);
  Serial.print(" ");
  Serial.print("avg speed: ");
  Serial.println(avg_speed);
}

void update_speed_stats(float speed) {
  max_speed = _max(speed, max_speed);

  statistic_ticks++;
  avg_speed = 1.0*(avg_speed*(statistic_ticks-1) + speed)/statistic_ticks;
}

