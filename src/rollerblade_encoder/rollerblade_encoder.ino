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

void loop() {
  int reading = digitalRead(REED_SENSOR);
//  Serial.print(reading);
  long curr_time = millis();

  //if the wheel stays still for a while, you probably aren't moving
  if(curr_time - last_high_to_low > 1000) {
    last_speed = 0;
  }

  //measure time between magnet passes
  if(reading == LOW && last_value == HIGH) {
    long time_diff = curr_time - last_high_to_low; //inter-valley duration

    //compute the speed in meters per second
    float speed = circum/time_diff; //(in m/s) they are both in milli- units, so they cancel each other out
    last_speed = speed;
    
    last_high_to_low = curr_time;
  }
  
  Serial.print(last_speed );

  last_value = reading;


  Serial.print(" ");
  Serial.print(-0.1);
  Serial.print(" ");
  Serial.println(5);
//  Serial.println("");
}

