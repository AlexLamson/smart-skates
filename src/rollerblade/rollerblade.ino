#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

#define RIGHT_HALL_SENSOR 2

#define NEOP 5
#define PIXELCOUNT 30
#define PIXELINNERCOUNT 12

// variables about physical entities
const float circum = 251.327412; // mm - circumference of wheels
float inter_pixel_distance = 1000.0 / 60; // mm - distance between neopixels

// general variables for updating lights at intervals
byte tickMillis = 20;
unsigned long lastTickMillis = 0;
CRGB leds[PIXELCOUNT];

// variables for speed-dependent pattern
CRGB color = 0xdd00ff;

float position = 0;
float inter_light_distance = inter_pixel_distance * PIXELCOUNT / 2; // mm - distance for the logical lights to display
float logical_light_distance = inter_light_distance / inter_pixel_distance; // what the program uses to draw the lights

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

WheelSpeed right_wheel = WheelSpeed(RIGHT_HALL_SENSOR);
//WheelSpeed left_wheel = WheelSpeed(LEFT_HALL_SENSOR);

void drawSmoothedPixel(CRGB*, int, int, float, CRGB);
byte mapBrightness(float);

void setup() {
  Serial.begin(115200);
  
  pinMode(RIGHT_HALL_SENSOR, INPUT_PULLUP);
  
  FastLED.addLeds<NEOPIXEL, NEOP>(leds, PIXELCOUNT);
}

void loop() {
  // update sensors and things
  right_wheel.update();
  
  Serial.print(right_wheel.get_speed());

  Serial.print(" ");
  Serial.print(-0.1);
  Serial.print(" ");
  Serial.println(5);

  // draw patterns
  if (millis() >= lastTickMillis + tickMillis) {
    lastTickMillis = millis();
    
    position += (right_wheel.get_speed() * tickMillis) / inter_pixel_distance;
    if (position >= logical_light_distance) { position -= logical_light_distance; } // can't mod floats

    fill_solid(leds, PIXELCOUNT, 0); // clear the lightstrip
    
    // draw lights at intervals
    for (float f = position - logical_light_distance; f < PIXELCOUNT - PIXELINNERCOUNT + 1; f += logical_light_distance) {
      drawSmoothedPixel(leds, 0, PIXELINNERCOUNT, PIXELINNERCOUNT - f, color);
      drawSmoothedPixel(leds, PIXELINNERCOUNT, PIXELCOUNT, f + PIXELINNERCOUNT, color);
    }
    
    FastLED.show();
  }
}

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

void drawSmoothedPixel(CRGB* leds, int start, int end, float position, CRGB color) {
  for (int i = int(position - 1); i <= int(position + 1); i++) {
    if (i < 0 || i < start || i >= end) { continue; }
    
    float normBrightness = max(0.0, 1.0-float(abs(i - position + 0.5))*2.0/3); // kernel that's 2 pixels wide, requires at most 3 pixels to show
    byte brightness = 255*(0.5-cos( PI * pow(normBrightness, 1.5) )/2); // power of 1.5 seems to preserve brightness pretty well.
    
    leds[i] = CRGB( scale8(color.r, brightness), scale8(color.g, brightness), scale8(color.b, brightness) );
  }
}

// input within [0, 1]
byte mapBrightness(float input) {
  return 255*(0.5-cos( PI * pow(input, 1.5) )/2); // power of 1.5 seems to preserve brightness pretty well.
}

