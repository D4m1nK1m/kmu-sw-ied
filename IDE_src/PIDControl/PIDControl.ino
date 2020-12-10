#include <Servo.h>

#define PIN_LED 9     
#define PIN_SERVO 10
#define PIN_IR A0    

#define _DIST_TARGET 255 
#define _DIST_MIN 100   
#define _DIST_MAX 410   

#define _DIST_ALPHA 0.3

#define _DUTY_MIN 1000    
#define _DUTY_NEU 1370    
#define _DUTY_MAX 1600   

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30    
#define _INTERVAL_SERVO 20

#define _INTERVAL_DIST 20   
#define _INTERVAL_SERVO 20    
#define _INTERVAL_SERIAL 100  

#define LENGTH 30
#define k_LENGTH 8
#define Horizontal_Angle 2160
#define Max_Variable_Angle 100

#define _KP 1
#define _KD -40
#define _KI 0.003

#define _RAMPUP_TIME 100 
#define INTERVAL 20  // servo update interval

#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100


Servo myservo;

float dist_target; 
float dist_raw, dist_ema, dist_min, dist_max, duty_ema;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; 

int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
int duty_chg_per_interval; // current speed (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec

float duty_target, duty_curr;   

float dterm, error_curr, error_prev, control, pterm, iterm, d_ema;

int a, b; // unit: mm
int correction_dist, iter;
float dist_list[LENGTH], sum, alpha;

void setup() { 
  pinMode(PIN_LED,OUTPUT);          
  myservo.attach(PIN_SERVO); 
  dist_raw=0.0;
  // initialize global variables
  dist_min = _DIST_MIN;     
  dist_max= _DIST_MAX;  
  dist_ema= 150;
  duty_ema = 1400;
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  iterm=0;
  d_ema =150;
  a = 68;
  b = 245;
  correction_dist = 0;
  iter = 0; sum = 0;
  alpha = _DIST_ALPHA;
  error_prev=155;
  duty_curr = 1650;
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * ( _INTERVAL_SERVO / 700.0);
  duty_chg_adjust = (float) duty_chg_max * INTERVAL / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.
  
  // initialize serial port
  Serial.begin(57600);  
  
  // convert angle speed into duty change per interval.
  //duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * ( _INTERVAL_SERVO / 300.0);
}
  
float ir_distance_filter() {
  sum = 0;
  iter = 0;
  while (iter < LENGTH)
  {
    dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }

  for (int i = 0; i < LENGTH-1; i++){
    for (int j = i+1; j < LENGTH; j++){
      if (dist_list[i] > dist_list[j]) {
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    }
  }
  
  for (int i = 0; i < k_LENGTH; i++) {
    sum -= dist_list[i];
  }
  for (int i = 1; i <= k_LENGTH; i++) {
    sum -= dist_list[LENGTH-i];
  }

  float dist_cali = sum/(LENGTH-2*k_LENGTH);
  
  return _DIST_ALPHA*dist_cali + (1-_DIST_ALPHA)*dist_ema;
}
void loop() {
  unsigned long time_curr = millis();
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;

        if(duty_target > duty_curr) {
          if(duty_chg_per_interval < duty_chg_max) {
            duty_chg_per_interval += duty_chg_adjust;
            if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
          }
          //duty_curr += duty_chg_per_interval;
          duty_curr += duty_chg_max;
          if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else if(duty_target < duty_curr) {
          if(duty_chg_per_interval > -duty_chg_max) {
            duty_chg_per_interval -= duty_chg_adjust;
            if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
        }
          //duty_curr += duty_chg_per_interval;
          duty_curr += -duty_chg_max;
          if(duty_curr < duty_target) duty_curr = duty_target;
        }
  else {
    duty_chg_per_interval = 0;
  }
       myservo.writeMicroseconds(duty_curr);
       
    
        //Serial.print(duty_curr);
        
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
    dist_raw = ir_distance_filter();
    
    if(dist_raw < 10)
      dist_raw = 10;
    if(dist_raw > 410)
      dist_raw = 410;
    pterm = _KP *(255-dist_raw);

    error_curr = 255 - dist_raw;
    dterm = _KD * (error_prev - error_curr);
    d_ema = 0.2*dterm + (1-0.2)*d_ema;

    
    iterm += _KI * error_curr;
    control = pterm + d_ema  + iterm;
    duty_target = _DUTY_NEU + control; 
    
    /*Serial.print(dist_raw);
    Serial.print(" ");
    Serial.print(pterm);
    Serial.print(" ");
    Serial.println(dterm);*/
    
    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; // lower limit
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX; // upper limit
    
    // update error_prev
    error_prev = error_curr;
    event_dist = false;
  }
  if(event_servo) {
    event_servo = false;
  }
  if(event_serial) {
    event_serial = false;
    int zero = 1400;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    //Serial.print(map(d_ema,-1000,1000,510,610));
   
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.print(",zero:");
    Serial.print(map(zero,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");

  }
}

float ir_distance(void){ // return value unit: mm
  float val; 
  float volt = float(analogRead(PIN_IR)); 
  val = ((((6762.0/(volt-9.0))-4.0) * 10.0)+5)/0.46-105; 
  return val; 
 /*150 52
  200 89
  250 113
  300 131
  350 152
  400 179*/
  
  
}
