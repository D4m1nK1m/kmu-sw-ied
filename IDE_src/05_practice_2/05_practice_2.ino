#define PIN_LED 7
unsigned int count, toggle, cnt;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = cnt =0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  if(toggle == 1){
    if(cnt == 5){
      toggle = toggle_state(toggle);
      while(1){}
    }
    toggle = toggle_state(toggle);
    delay(100);
    digitalWrite(PIN_LED, toggle);
    toggle = toggle_state(toggle);
    delay(100);
    cnt++;
  }else{
    toggle = toggle_state(toggle); //toggle LED value.
    delay(1000);
  }
  digitalWrite(PIN_LED, toggle); // update LED status.
}

int toggle_state(int toggle) {
  return 1-toggle;
}
