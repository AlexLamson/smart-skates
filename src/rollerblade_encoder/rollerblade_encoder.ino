#define REED_SENSOR 2

void setup() {
  Serial.begin(115200);
  pinMode(REED_SENSOR, INPUT_PULLUP);
}

const float circum = 251.327412; //in millimeters

int big_queue[5]; //history of raw sensor values
int i = 0;

long last_high_to_low = 0;
int last_value = 0;

float last_speed = 0;

class WheelSpeed {
  public:
    WheelSpeed(int);
    void update();
    float get_speed();
    
  private:
    int pin;
    float speed;
    unsigned long last_fall;
    bool hall_last_value;

    void update_speed();
};

WheelSpeed right_wheel = WheelSpeed(REED_SENSOR);

void loop() {
  right_wheel.update();
  
  Serial.print(right_wheel.get_speed());

  Serial.print(" ");
  Serial.print(-0.1);
  Serial.print(" ");
  Serial.println(5);
//  Serial.println("");
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

float WheelSpeed::get_speed() {
  long time_diff = millis() - last_fall; //inter-valley duration
  
  if (time_diff > circum / speed) { // the wheel is slowing down since we haven't seen it in a while
    update_speed();
  }

  return speed;
}

