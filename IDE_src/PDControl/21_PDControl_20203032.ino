#include <Servo.h>

#define PIN_LED 9     
#define PIN_SERVO 10
#define PIN_IR A0    

#define _DIST_TARGET 255 
#define _DIST_MIN 100   
#define _DIST_MAX 410   

#define _DIST_ALPHA 0.3   

#define _DUTY_MIN 1100    
#define _DUTY_NEU 1400    
#define _DUTY_MAX 1700   

#define _SERVO_ANGLE 30  
#define _SERVO_SPEED 30    
#define _INTERVAL_SERVO 20

#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20    
#define _INTERVAL_SERIAL 100  

#define _KP 0.8
#define _KD -20
Servo myservo;

float dist_target; 
float dist_raw, dist_ema, dist_min, dist_max, duty_ema;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

// Servo speed control
int duty_chg_per_interval; 
float duty_target, duty_curr;   

float dterm, error_curr, error_prev=0, control, pterm;

void setup() { 
  pinMode(PIN_LED,OUTPUT);          
  myservo.attach(PIN_SERVO); 
  dist_raw=0.0;
  // initialize global variables
  dist_min = _DIST_MIN;     
  dist_max= _DIST_MAX;  
  dist_ema= 0;
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  
  // initialize serial port
  Serial.begin(57600);  
  
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
    dist_raw = ir_distance_filtered();
    
    if(dist_raw < 100)
      dist_raw = 100;
    if(dist_raw > 410)
      dist_raw = 410;
      
    pterm = _KP *(255-dist_raw);
    control = pterm;
    duty_target = _DUTY_NEU + control; 
      
    error_curr = 225 - dist_raw;
     
    dterm = _KD * (error_prev - error_curr);
    control = dterm;
    duty_target += control;  
    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
    // update error_prev
    error_prev = error_curr;
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
      
    if(duty_target > 1700)
       duty_target = 1700;
      if(duty_target <1100)
       duty_target = 1100;
  }
  if(event_serial) {
    event_serial = false;
// output the read value to the serial port
     Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
  Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    //Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

    
    Serial.println(",High:220,Max:300");
  }
}

float ir_distance(void){ // return value unit: mm
  float val; 
  float volt = float(analogRead(PIN_IR)); 
  val = (((6762.0/(volt-9.0))-4.0) * 10.0-24.9286)/0.68143+100; 
  return val; 
}
float ir_distance_filtered(void){ // return value unit: mm
   dist_ema = (1-_DIST_ALPHA)*ir_distance()+ dist_ema*_DIST_ALPHA;
  return dist_ema; // for now, just use ir_distance() without noise filter.
}
