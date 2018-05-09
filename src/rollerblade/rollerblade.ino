#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

//to get the absolute time
#include <time.h>
const int timezone = -5;
const int dst = 1;

//temboo stuff
#include <Temboo.h>
#include "TembooAccount.h"
int calls = 1;   // Execution count, so this doesn't run forever
int maxCalls = 100;   // Maximum number of times the Choreo should be executed
boolean should_execute_temboo_task = false;
WiFiClient client;

//to be able to disable blynk when you want
//#include <PinChangeInterrupt.h>


//makerboard pins
//#define LEFT_PRESSURE_SENSOR 4
//#define RIGHT_PRESSURE_SENSOR 13
//#define LEFT_HALL_SENSOR 5
//#define RIGHT_HALL_SENSOR 12
//#define LEFT_NEOP 0
//#define RIGHT_NEOP 14

//ESP8266 pins
#define BUTTON_PIN 0
#define BUZZER_PIN 13
#define LEFT_PRESSURE_SENSOR 2
#define RIGHT_PRESSURE_SENSOR 14
#define LEFT_HALL_SENSOR 12
#define RIGHT_HALL_SENSOR 4
#define LEFT_NEOP 15
#define RIGHT_NEOP 5

//button variables
volatile boolean interrupts_attached = false;
volatile boolean enable_blynk = true;
volatile boolean can_toggle_blynk = true;
volatile unsigned long last_time_button_pressed = 0;
volatile unsigned long last_time_button_released = 0;
byte button_prev_state = 0;

//pressure variables
unsigned long left_last_pressure_time = 0;
unsigned long right_last_pressure_time = 0;
const int pressure_debounce_time = 50;
boolean left_stepping_on = false;
boolean right_stepping_on = false;

// neopixel variables
#define PIXEL_COUNT 30
#define PIXEL_INNER_COUNT 12

// for connecting to blynk and temboo
char auth[] = "128c8d58f05d4913be0a340473683013"; //blynk auth token
BlynkTimer timer;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "CS390N";
char pass[] = "internetofthings";


// variables about physical entities
const float circum = 251.327412; // mm - circumference of wheels
float inter_pixel_distance = 1000.0 / 60; // mm - distance between neopixels


//statistics variables
const int SPEED_BUFFER_SIZE = 10*60;
const int TIME_STOPPED_TIL_DATA_DUMP = 3*1000;
const int MIN_TIME_TIL_DATA_DUMP = 30*1000;
float max_speed = 0;
float avg_speed = 0; //running average
float left_speeds[SPEED_BUFFER_SIZE];
float right_speeds[SPEED_BUFFER_SIZE];
int speed_index = 0; //index where we are in the speeds array
unsigned long time_last_stopped = 0;
unsigned long time_last_dumped = 0;
unsigned long statistic_ticks = 0; //number of speeds that have been recorded
unsigned int total_steps = 0;

// general variables for updating lights at intervals
byte tickMillis = 20;
unsigned long lastTickMillis = 0;
CRGB left_leds[PIXEL_COUNT];
CRGB right_leds[PIXEL_COUNT];

//~~~~~~~~~~~~~~~~~~~~~~~~~
// pattern variables
//~~~~~~~~~~~~~~~~~~~~~~~~~

// encodes id of current pattern
// to view list of IDs, look at BLYNK_WRITE(V1)
byte pattern = 2;

// variables for fixed light speed-dependent pattern
CRGB color = 0x1144ff;

float position = 0;
//float inter_light_distance = inter_pixel_distance * PIXEL_COUNT / 2; // mm - distance for the logical lights to display
float inter_light_distance = 500; // mm - distance for the logical lights to display
float logical_light_distance = inter_light_distance / inter_pixel_distance; // what the program uses to draw the lights

// fixed rainbow
float hue_position = 0;
//float hue_per_meter = 255;
float hue_per_meter = 64;
float hue_per_pixel = inter_pixel_distance * hue_per_meter / 1000;

// speed = brightness
CRGB color_p2 = 0xff0000;
float bright_speed = 3; // m/s - the speed at which the lights are brightest

// speed = hue
byte hue_at_zero = 0;
float rainbow_speed = 5; // speed to go all the way around the hue cycle

// tail lights
float last_speed = 0;
float decel_threshold = 0.01; // should be in m/s2 but it isn't
int brake_trail_time = 500; // ms - time to leave lights on after stopped slowing down (flickers without this)
unsigned long last_brake_time = 0;

// footstep hue
//byte hue_delta = 51; //51->5 distinct colors
byte hue_delta = 17; //51->5 distinct colors, 17->15
float left_hue = 0;
float right_hue = 0;

//step = increase brightness
const byte step_brightness_delta = 128;
const byte step_brightness_fade_delta = 2;
byte left_step_brightness = 0;
byte right_step_brightness = 0;

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
void recordData();
CRGB scaleColor(CRGB, byte);
float compute_aggregate_wheel_speed(float, float);
void button_pressed();
void button_released();
void connect_to_wifi();


void setup() {
  Serial.begin(115200);

  button_prev_state = digitalRead(BUTTON_PIN);
  
  FastLED.addLeds<NEOPIXEL, LEFT_NEOP>(left_leds, PIXEL_COUNT);
  FastLED.addLeds<NEOPIXEL, RIGHT_NEOP>(right_leds, PIXEL_COUNT);

  //set color of neopixel strips for debugging purposes
  fill_solid(left_leds, PIXEL_COUNT, CRGB(0, 255, 0));
  fill_solid(right_leds, PIXEL_COUNT, CRGB(0, 255, 0));
  FastLED.show();

  /*
   * prepare for blynk connection
   * 
   * there's a 4 second grace period where if the button is held down for >1 sec it will not try to connect to wifi
   */
  connect_to_wifi();

  timer.setInterval(1000L, recordData);

  // pull up some pins
  pinMode(RIGHT_HALL_SENSOR, INPUT_PULLUP);
  pinMode(LEFT_HALL_SENSOR, INPUT_PULLUP);
  pinMode(LEFT_PRESSURE_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_PRESSURE_SENSOR, INPUT_PULLUP);
}

void loop() {
  unsigned long curr_time = millis();
  
  // check the wheel speeds
  left_wheel.update();
  right_wheel.update();

  //update some statistics
  float left_wheel_speed = left_wheel.get_speed();
  float right_wheel_speed = right_wheel.get_speed();
  float aggregate_wheel_speed = compute_aggregate_wheel_speed(left_wheel_speed, right_wheel_speed);
  update_speed_stats(aggregate_wheel_speed);

//  Serial.print("left speed: ");
//  Serial.print(left_wheel_speed);
//  Serial.print(" right speed: ");
//  Serial.println(right_wheel_speed);

  //detach the interupt after the wifi connection part is done if need be
  if(interrupts_attached) {
    detachInterrupt(BUTTON_PIN);
    interrupts_attached = false;
  }

  //hold physical button for 3 seconds to enable non-wifi mode
  //cycle demos by clicking physical button on chip
  byte button_curr_state = digitalRead(BUTTON_PIN);

  //if the button was pressed
  if(button_curr_state==LOW && button_prev_state==HIGH && curr_time > last_time_button_pressed+100) {
    last_time_button_pressed = curr_time;
  }
  //if the buttom was released
  else if(button_curr_state==HIGH && button_prev_state==LOW && curr_time > last_time_button_released+100) {
    can_toggle_blynk = true;
    
    if(!enable_blynk) {
      //tap button to change demos
      pattern = (pattern+1) % 11;
      if(pattern == 0)
        pattern++;
    }
    
    last_time_button_released = curr_time;
  }
  button_prev_state = button_curr_state;


  //hold button to toggle blynk
  if(button_curr_state==LOW && curr_time-last_time_button_pressed >= 2000 && can_toggle_blynk) {
    enable_blynk = !enable_blynk;
    can_toggle_blynk = false;

    if(enable_blynk) {
      tone(BUZZER_PIN, 4000);
      delay(200);
      tone(BUZZER_PIN, 5000);
      delay(200);
      tone(BUZZER_PIN, 0);
    }
    else {
      tone(BUZZER_PIN, 5000);
      delay(200);
      tone(BUZZER_PIN, 4000);
      delay(200);
      tone(BUZZER_PIN, 0);
    }

    connect_to_wifi();
  }

  

  //blynk can be disabled for performance purposes
  if(enable_blynk) {
    // update blynk stuff
    Blynk.run();
//    timer.run();

    // do temboo stuff if needed
    if(should_execute_temboo_task) {
      should_execute_temboo_task = false;
  
      write_to_spreadsheet(String(avg_speed), String(max_speed), String(total_steps), String(millis()/1000));
    }
  }


  /*
   * Only dump the data to blynk if:
   * 1. the skates are stopped
   * 2. the skates have been stopped for a little bit of time
   * 3. the data hasn't been dumped for a bit of time
   */
  if(aggregate_wheel_speed <= 0.01) {
    if(curr_time-time_last_stopped > TIME_STOPPED_TIL_DATA_DUMP) {
      if(curr_time-time_last_dumped > MIN_TIME_TIL_DATA_DUMP) {
        time_last_dumped = curr_time;
        sendDataToBlynk();
      }
    }
  }
  else {
    time_last_stopped = curr_time;
  }


  // detect steps from pressure sensors
  boolean left_stepped_off = false;
  boolean left_stepped_on = false;
  boolean right_stepped_off = false;
  boolean right_stepped_on = false;

  // debounce left skate
  boolean new_left_stepping_on = (digitalRead(LEFT_PRESSURE_SENSOR) == LOW);
  if(curr_time - left_last_pressure_time > pressure_debounce_time && left_stepping_on != new_left_stepping_on) {
    left_last_pressure_time = curr_time;

    left_stepped_off = (left_stepping_on == true && new_left_stepping_on == false);
    left_stepped_on = (left_stepping_on == false && new_left_stepping_on == true);
    
    left_stepping_on = new_left_stepping_on;

    if(left_stepped_on) {
      total_steps++;
      left_hue = byte(left_hue + hue_delta);
      left_step_brightness = min(255, (int)(left_step_brightness+step_brightness_delta));
      Serial.println("left stepped");
    }
  }

  // debounce right skate
  boolean new_right_stepping_on = (digitalRead(RIGHT_PRESSURE_SENSOR) == LOW);
  if(curr_time - right_last_pressure_time > pressure_debounce_time && right_stepping_on != new_right_stepping_on) {
    right_last_pressure_time = curr_time;

    right_stepped_off = (right_stepping_on == true && new_right_stepping_on == false);
    right_stepped_on = (right_stepping_on == false && new_right_stepping_on == true);
    
    right_stepping_on = new_right_stepping_on;

    if(right_stepped_on) {
      total_steps++;
      right_hue = byte(right_hue + hue_delta);
      right_step_brightness = min(255, (int)(right_step_brightness+step_brightness_delta));
      Serial.println("right stepped");
    }
  }

  // draw patterns
  if (curr_time >= lastTickMillis + tickMillis) {
    lastTickMillis = millis();

    switch (pattern) {
      case 1: // fixed light
      {
        position += (aggregate_wheel_speed * tickMillis) / inter_pixel_distance;
        if (position >= logical_light_distance) { position -= logical_light_distance; } // can't mod floats
        
        FastLED.clear(); // clear both strips
        
        // draw lights at intervals
        for (float f = position - logical_light_distance; f < PIXEL_COUNT - PIXEL_INNER_COUNT + 1; f += logical_light_distance) {
          drawSmoothedPixel(left_leds, 0, PIXEL_INNER_COUNT, PIXEL_INNER_COUNT - f, color);
          drawSmoothedPixel(left_leds, PIXEL_INNER_COUNT, PIXEL_COUNT, f + PIXEL_INNER_COUNT, color);
  
          drawSmoothedPixel(right_leds, 0, PIXEL_INNER_COUNT, PIXEL_INNER_COUNT - f, color);
          drawSmoothedPixel(right_leds, PIXEL_INNER_COUNT, PIXEL_COUNT, f + PIXEL_INNER_COUNT, color);
        }
      
      }
      break;

      case 2: // fixed rainbow
      {
        hue_position -= (aggregate_wheel_speed * tickMillis) * hue_per_meter / 1000;
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
      }
      break;

      case 3: // speed = brightness
      {
        byte left_bright = byte(255 * _min(1.0f, left_wheel_speed / bright_speed)); // later, get the correct aggregate speed
        byte right_bright = byte(255 * _min(1.0f, right_wheel_speed / bright_speed)); // later, get the correct aggregate speed
        fill_solid(left_leds, PIXEL_COUNT, scaleColor(color_p2, left_bright));
        fill_solid(right_leds, PIXEL_COUNT, scaleColor(color_p2, right_bright));

//        unsigned int minFreq = 100;
//        unsigned int maxFreq = 2000;
//        unsigned int freq = minFreq+(maxFreq-minFreq)*aggregate_wheel_speed/3.0;
//        tone(BUZZER_PIN, freq, tickMillis);
      }
      break;
      
      case 4: // speed = hue
      {
        byte left_h = byte(255 * left_wheel_speed / rainbow_speed);
        left_h += hue_at_zero;

        byte right_h = byte(255 * right_wheel_speed / rainbow_speed);
        right_h += hue_at_zero;

        fill_solid(left_leds, PIXEL_COUNT, CHSV(left_h, 255, 255));
        fill_solid(right_leds, PIXEL_COUNT, CHSV(right_h, 255, 255));
      }
      break;

      case 5: // tail lights
      {
        FastLED.clear();
        for(int i = 1; i < 4; i++) {
          left_leds[PIXEL_INNER_COUNT-i].r = 255; // headlight
          left_leds[PIXEL_INNER_COUNT-i].g = 255; // headlight
          left_leds[PIXEL_INNER_COUNT-i].b = 255; // headlight
          right_leds[PIXEL_INNER_COUNT-i].r = 255; // headlight
          right_leds[PIXEL_INNER_COUNT-i].g = 255; // headlight
          right_leds[PIXEL_INNER_COUNT-i].b = 255; // headlight
        }

        left_leds[PIXEL_COUNT - 1].r = 64; // running light
        right_leds[PIXEL_COUNT - 1].r = 64; // running light
  
        float curr_speed = aggregate_wheel_speed;

        if (last_speed - curr_speed > decel_threshold || curr_time <= last_brake_time + brake_trail_time) {
          if (last_speed - curr_speed > decel_threshold) {
            last_brake_time = curr_time;
          }
          const int upto = 3;
          for (int i = 0; i < upto; i++) {
            left_leds[PIXEL_COUNT - 1 - i].r = 255;
            right_leds[PIXEL_COUNT - 1 - i].r = 255;
          }
        }

        last_speed = curr_speed;
      }
      break;

      case 6: // lights on when pressure
      {
        byte left_color = 255 * left_stepping_on;
        byte right_color = 255 * right_stepping_on;

        fill_solid(left_leds, PIXEL_COUNT, CRGB(0, 0, left_color));
        fill_solid(right_leds, PIXEL_COUNT, CRGB(0, 0, right_color));
      }
      break;

      case 7: // footstep hue
      {
        fill_solid(left_leds, PIXEL_COUNT, CHSV( left_hue, 255, 255 ));
        fill_solid(right_leds, PIXEL_COUNT, CHSV( right_hue, 255, 255 ));
      }
      break;

      case 8: // step = increase brightness
      {
        left_step_brightness = max(0, (int)(left_step_brightness-step_brightness_fade_delta));
        right_step_brightness = max(0, (int)(right_step_brightness-step_brightness_fade_delta));

        byte adjusted_left_brightness = mapBrightness(((float)left_step_brightness)/255.0);
        byte adjusted_right_brightness = mapBrightness(((float)right_step_brightness)/255.0);
        
        fill_solid(left_leds, PIXEL_COUNT, CRGB( 0, 0, adjusted_left_brightness ));
        fill_solid(right_leds, PIXEL_COUNT, CRGB( 0, 0, adjusted_right_brightness ));
      }
      break;

      case 9: // debug
      {
        FastLED.clear();
        byte left_bright = byte(255 * _min(1.0f, left_wheel_speed / bright_speed));
        byte right_bright = byte(255 * _min(1.0f, right_wheel_speed / bright_speed));
        
        byte left_g = 0;
        if(left_stepping_on)
          left_g = 255;

        byte right_g = 0;
        if(right_stepping_on)
          right_g = 255;

        byte b = 0;

        fill_solid(left_leds, PIXEL_COUNT, CRGB(left_bright, left_g, b));
        fill_solid(right_leds, PIXEL_COUNT, CRGB(right_bright, right_g, b));
      }
      break;

      case 10: // police
      {
        FastLED.clear();
        float flashes_per_second = 2;
        if(long(millis()/(1000/flashes_per_second)) % 2 == 0) {
          fill_solid(left_leds, PIXEL_COUNT, CRGB(255, 0, 0));
          fill_solid(right_leds, PIXEL_COUNT, CRGB(0, 0, 255));
        }
        else {
          fill_solid(left_leds, PIXEL_COUNT, CRGB(0, 0, 255));
          fill_solid(right_leds, PIXEL_COUNT, CRGB(255, 0, 0));
        }
      }
      break;

      case 11: // hybrid step brightness
      {
        
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
 * V7 - step counter
 */

//upload stats button
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt();
  boolean button_down = (pinValue == HIGH);
  if(button_down) {
//    Serial.println("Upload stats button pressed");
    Serial.println("Preparing to upload stats to spreadsheet");
//    write_to_spreadsheet("love", "joy");
    should_execute_temboo_task = true;
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
    case 10:
      Serial.println("pattern: police lights");
      break;
    default:
      Serial.println("pattern: Unknown pattern code (defaulting to fixed lights)");
      pattern = 1;
  }
}

//blynk timer called every 1 second
void recordData()
{
  float left_speed = left_wheel.get_speed();
  float right_speed = right_wheel.get_speed();
  float aggregate_wheel_speed = compute_aggregate_wheel_speed(left_speed, right_speed);

  left_speeds[speed_index] = left_speed;
  right_speeds[speed_index] = right_speed;

  speed_index = (speed_index+1) % SPEED_BUFFER_SIZE; //note: it shouldn't ever need to wrap around

  //print some debug information
  Serial.print("left: ");
  if(left_stepping_on)
    Serial.print("stepping ");
  else
    Serial.print("not stepping ");
  Serial.print("right: ");
  if(right_stepping_on)
    Serial.print("stepping ");
  else
    Serial.print("not stepping ");
  Serial.print(" left: ");
  Serial.print(left_speed);
  Serial.print(" m/s | ");
  Serial.print("right: ");
  Serial.print(right_speed);
  Serial.print(" m/s | ");
  Serial.print("avg: ");
  Serial.print(aggregate_wheel_speed);
  Serial.println(" m/s");
}

float compute_aggregate_wheel_speed(float left_wheel_speed, float right_wheel_speed) {
  float aggregate_wheel_speed = 0;
  if (left_stepping_on && right_stepping_on) {
    aggregate_wheel_speed = max(left_wheel_speed, right_wheel_speed);
  } else if (left_stepping_on) {
    aggregate_wheel_speed = left_wheel_speed;
  } else if (right_stepping_on) {
    aggregate_wheel_speed = right_wheel_speed;
  } else { // no feet are on the ground
    aggregate_wheel_speed = (left_wheel_speed + right_wheel_speed) / 2;
  }
  return aggregate_wheel_speed;
}

//called only when the skate stops moving so we don't interfer with the timing of the patterns
void sendDataToBlynk()
{
  Serial.println("starting data dump");
  
  //max speed
  Blynk.virtualWrite(V2, max_speed);

  //total time spent skating
  Blynk.virtualWrite(V3, millis() / 1000);

  //total steps
  Blynk.virtualWrite(V7, total_steps);

  //dump speeds
//  for(int i = 0; i <= speed_index; i++) {
//    float left_speed = left_speeds[i];
//    float right_speed = right_speeds[i];
//    float avg_of_both_skates = (left_speed+right_speed)/2.0;
//    Blynk.virtualWrite(V5, left_speed);
//    Blynk.virtualWrite(V6, right_speed);
//    Blynk.virtualWrite(V4, avg_of_both_skates);
//
//    //clear out the old data
//    left_speeds[i] = 0;
//    right_speeds[i] = 0;
//  }
  speed_index = 0; //reset the index to prepare for the next data dump  

  Serial.println("finished data dump");

//  //left speed
//  float left_speed = left_wheel.get_speed();
//  Blynk.virtualWrite(V5, left_speed);
//  
//  //right speed
//  float right_speed = right_wheel.get_speed();
//  Blynk.virtualWrite(V6, right_speed);
//
//  //avg speed
//  float avg_of_both_skates = (left_speed + right_speed)/2.0;
//  Blynk.virtualWrite(V4, avg_of_both_skates);
}

void update_speed_stats(float speed) {
  if (speed >= 0.01) {
    max_speed = _max(speed, max_speed);
  
    statistic_ticks++;
    avg_speed = 1.0*(avg_speed*(statistic_ticks-1) + speed)/statistic_ticks;
  }
}

void write_to_spreadsheet(String avg_speed_string, String max_speed_string, String total_steps_string, String time_spent_string) {
  if (calls <= maxCalls) {
    calls++;
//    Serial.println("Running smart skate temboo choreo friend - Run #" + String(calls));
    Serial.println("APPENDING STATS TO SPREADSHEET");
    
    
    TembooChoreo AppendToSpreadsheetChoreo(client);

    // Invoke the Temboo client
    AppendToSpreadsheetChoreo.begin();

    // Set Temboo account credentials
    AppendToSpreadsheetChoreo.setAccountName(TEMBOO_ACCOUNT);
    AppendToSpreadsheetChoreo.setAppKeyName(TEMBOO_APP_KEY_NAME);
    AppendToSpreadsheetChoreo.setAppKey(TEMBOO_APP_KEY);


    AppendToSpreadsheetChoreo.addInput("RefreshToken", "1/L56ybiyPZP9BG_P6P1eQaV1_LMI1mlCCOlx6o5GJAdo");
    AppendToSpreadsheetChoreo.addInput("ClientSecret", "nkSiHCtY4uhJw5ElWiZsImWL");
    
    time_t now = time(nullptr);
    String time_string = ctime(&now);
    time_string.trim();

//    String appended_row = "[[\""+time_string+"\",\""+value_string+"\"]]";
//    String appended_row = "[[\""+time_string+"\",\""+avg_speed_string+"\",\""+max_speed_string+"\",\""+time_spent_string+"\"]]";
    String appended_row = "[[\""+avg_speed_string+"\",\""+max_speed_string+"\",\""+total_steps_string+"\",\""+time_spent_string+"\"]]";
    Serial.println(appended_row);
    AppendToSpreadsheetChoreo.addInput("Values", appended_row);
    
    AppendToSpreadsheetChoreo.addInput("ClientID", "243948719335-78difeijcp952a4tu3d20lsrfue5cvu7.apps.googleusercontent.com");
    AppendToSpreadsheetChoreo.addInput("SpreadsheetID", "1M3QXqTQ0ka-VfH5SVIgdJAq2QSrUy2FLGo-ffGr55G0");
    
    // Identify the Choreo to run
    AppendToSpreadsheetChoreo.setChoreo("/Library/Google/Sheets/AppendValues");
    
    // Run the Choreo; when results are available, print them to serial
    AppendToSpreadsheetChoreo.run();
    
    while(AppendToSpreadsheetChoreo.available()) {
      char c = AppendToSpreadsheetChoreo.read();
      Serial.print(c);
    }
    AppendToSpreadsheetChoreo.close();
  }
  Serial.println("");
}

//called via interrupt on the button pin
void button_pressed() {
   last_time_button_pressed = millis();
}

//called via interrupt on the button pin
void button_released() {
  last_time_button_released = millis();
  if(last_time_button_released-last_time_button_pressed > 1000) {
    //disable blynk
    if(can_toggle_blynk) {
      enable_blynk = !enable_blynk;
      can_toggle_blynk = false;
    }
  }
}

void connect_to_wifi() {
  delay(100);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_pressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_released, RISING);
  interrupts_attached = true;
  delay(4000);

  if(enable_blynk) {
    Serial.println("trying to connect to wifi");
    Blynk.begin(auth, ssid, pass);
    Serial.println("wifi connected");
  }
  else {
    Serial.println("Blynk disabled");
  }
}

