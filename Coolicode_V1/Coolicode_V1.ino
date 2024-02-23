  #include <AccelStepper.h>
  #include <Automaton.h>
  
  // variables
    int pos = 0;

    //stepper
      bool step_enable_A = false;
      bool step_enable_B = false;
      bool step_enable_C = false;

  // State 1 = idle, 2 = enabled, 3 = error / stopped
    int state = 0;

  // IO
    // Button pins
      #define rdy_btn_pin 2
      #define jog_fw_pin 6
      #define jog_bw_pin 5
      #define reset_btn 4
      #define stop_btn_pin 4
      #define test_btn_pin 7

    // Leds
      #define rdy_led 12
      #define error_led 13
      #define reset_led 11

    // Button values
      //int rdy_btn = 0;
      int jog_fw = 0;
      int jog_bw = 0;
      int stop_btn = 0;
      int test_btn = 0;

      int rdyState = 0;
      int stopState = 0;
      int testState = 0;

    // stepper
      int step_enable_A_pin = 8;
      int step_enable_B_pin = 11;
      int step_enable_C_pin = 12;

      int roll_stepPin = 10;
      int roll_dirPin = 19;
      int motorInterfaceType = 1;
    
    // stepper declaration
      AccelStepper RollStepper = AccelStepper(motorInterfaceType, roll_stepPin, roll_dirPin);

    //state machines from automaton
    Atm_led RdyLatch;
    Atm_button rdy_btn;
      
void setup() {
  // Modes
    // Inputs
      //pinMode(rdy_btn, INPUT);
      pinMode(jog_fw, INPUT);
      pinMode(jog_bw, INPUT);
      pinMode(test_btn, INPUT);

    // Outputs
      pinMode(rdy_led, OUTPUT);
      pinMode(error_led, OUTPUT);
      pinMode(roll_stepPin, OUTPUT);
      pinMode(roll_dirPin, OUTPUT);
   Serial.begin(9600);

    RdyLatch.begin(7); 
    rdy_btn.begin(2);
    rdy_btn.onPress(rdy_btn, RdyLatch.EVT_ON);

}

void read(){
  bool latch = false;
  //rdy_btn = digitalRead(rdy_btn_pin);
  stop_btn = digitalRead(stop_btn_pin);
  test_btn = digitalRead(test_btn_pin);

  if(stop_btn != stopState && stop_btn == 1){
    if(!latch){
      latch = true;
      stopState = 1;
    }
    else{
      latch = false;
      stopState = 0;
    }
  }
  if(stop_btn != stopState && stop_btn == 1){
    if(!latch){
      latch = true;
      stopState = 1;
    }
    else{
      latch = false;
      stopState = 0;
    }
  }

  if(test_btn != testState && test_btn == 1){
    if(!latch){
      latch = true;
      testState = 1;
    }
    else{
      latch = false;
      testState = 0;
    }
  }

  Serial.print("Ready: ");
  Serial.print(rdyState);
  Serial.print(" Stopped: ");
  Serial.print(stopState);
  Serial.print(" test: ");
  Serial.print(testState);
  Serial.print(" State: ");
  Serial.println(state);
}

void loop() {
  automaton.run();
  read();
 //roll();
 
  if (rdyState == 1){
    read();
    state = 1; 
    ready();
    
  }
  else if (stopState == 1){
    read();
    state = 3;
    stop();
  }

  if(state = 1 && test_btn == 1){
    roll();
  }
}

void stepperEnable(){
  if (state == 1 && testState == 1){

  }
}

void ready(){
  if (state == 1){
   read();
    digitalWrite(rdy_led, HIGH);
    digitalWrite(error_led, LOW);
  }
  return 0;
}

void run(){
}

void stop(){

  if(state == 3 & stop_btn == 1 ){
    //Serial.println(state);
    step_enable_A = false;
    step_enable_B = false;
    step_enable_C = false;

    digitalWrite(error_led, HIGH);
    digitalWrite(rdy_led, LOW);
    //Serial.println("here");
    read();
  }
  return 0;
}

//Stepper controller for x


void roll(){
  //Serial.println(state);
  int i;
  bool BTN = digitalRead(test_btn_pin);
  Serial.println(BTN);
  Serial.println("step");
  step_enable_A = true;
  while  (BTN == 1 && i < 1000){
    RollStepper.setMaxSpeed(20000);
    RollStepper.setSpeed(1000); 
    RollStepper.runSpeed();
    i = i+1;
    Serial.println(i);
  }
  i=0;
 return 0;
}

void SC_pursotin_A(){

}
