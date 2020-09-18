int ledPin = 7;
int Period;
void setup() {
  pinMode(ledPin, OUTPUT);
  set_period(10000);
  Serial.begin(115200);
}

void loop() {
  Serial.println("Hello World!");
  for (int Duty=0; Duty<100;Duty++) {
    set_duty(Duty);
  }
  for (int Duty=100; Duty>0;Duty--) {
    set_duty(Duty);
  }
}

void set_period(int period){ // period: 100 to 10000 (unit: us)
    Period = period;
}
void set_duty(int duty){ // duty: 0 to 100 (unit: %)
  int cnt = 5000.0/Period;
  if(Period == 10000)
    if(duty%2==0)
      return;
    else
    cnt=1;
  for(int i=0;i<cnt;i++){
    digitalWrite(ledPin, HIGH);
    delayMicroseconds((float)Period*(float)duty/100.0);
    digitalWrite(ledPin, LOW);
    delayMicroseconds(Period-(float)Period*(float)duty/100.0);
  }
}
