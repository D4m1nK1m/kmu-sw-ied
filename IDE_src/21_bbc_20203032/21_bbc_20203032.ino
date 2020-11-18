#include <Servo.h>

#define PIN_LED 9     
#define PIN_SERVO 10
#define PIN_IR A0    

#define _DIST_TARGET 255 
#define _DIST_MIN 100   
#define _DIST_MAX 410   

#define _DIST_ALPHA 0.3   

#define _DUTY_MIN 1000    
#define _DUTY_NEU 1420    
#define _DUTY_MAX 2000   

#define _SERVO_ANGLE 30   
#define _SERVO_SPEED 30    
#define _INTERVAL_SERVO 20

#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20    
#define _INTERVAL_SERIAL 100  

#define _KP 0.0  

Servo myservo;

float dist_target; 
float dist_raw, dist_ema, dist_min, dist_max;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;   


void setup() { 
  pinMode(PIN_LED,OUTPUT);          
  myservo.attach(PIN_SERVO); 
  dist_raw=0.0;
  // initialize global variables
  dist_min = _DIST_MIN;     
  dist_max= _DIST_MAX;  
  dist_ema= 0;
  // move servo to neutral position
  myservo.writeMicroseconds(1400);
  
  // initialize serial port
  Serial.begin(57600);  //[3039]
  
  // convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * ( _INTERVAL_SERVO / 1000.0); //[3039] 
}
  

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;

        if(duty_target > duty_curr) {
            duty_curr += duty_chg_per_interval;
            if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else {
            duty_curr -= duty_chg_per_interval;
            if(duty_curr < duty_target) duty_curr = duty_target;
        }
          myservo.writeMicroseconds(duty_curr);
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
    
  if(event_dist) {
    event_dist = false;
    // get a distance reading from the USS
    dist_raw = ir_distance_filtered();
    if(dist_raw < 100)
      dist_raw = 100;
    if(dist_raw > 410)
      dist_raw = 410;
  }
  if(event_servo) {
    event_servo = false;
// adjust servo position according to the USS read value
    if(dist_raw > 225) duty_target = 1100;
    else if(dist_raw <= 225) duty_target = 1700;
  }
  if(event_serial) {
    event_serial = false;
// output the read value to the serial port
    Serial.print("Min:100,Low:180,raw:");
    Serial.print(dist_raw);
    Serial.print(",servo:");
    Serial.print(myservo.read());  
    Serial.println(",High:220,Max:300");
  }
}

float ir_distance(void){ // return value unit: mm
  float val; //[3031] 
  float volt = float(analogRead(PIN_IR)); 
  val = (((6762.0/(volt-9.0))-4.0) * 10.0-24.9286)/0.68143; 
  return val; //[3031]
}

float ir_distance_filtered(void){ // return value unit: mm
   dist_ema = ir_distance()+ dist_ema*_DIST_ALPHA;
  return dist_ema; // for now, just use ir_distance() without noise filter.
}
