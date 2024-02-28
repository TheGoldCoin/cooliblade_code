#include <Automaton.h>
#include <AccelStepper.h>
#include <Servo.h>

Atm_button RdyBTN, ResetBTN, StopBTN, Calibration, TestBTN;
Atm_led ledRdy, ledErr, ledRes;
Atm_comparator cmp;

  // variables
    int pos = 0;

    //stepper
      bool step_enable_A = false;
      bool step_enable_B = false;
      bool step_enable_C = false;

  // State 1 = reset, 2 = rdy, 3 = error / stopped
    int state = 0;

  // IO
    // Button pins
      #define rdy_btn_pin 2
      #define jog_fw_pin 6
      #define jog_bw_pin 5
      #define reset_btn 3
      #define stop_btn_pin 4
      #define test_btn_pin 7
      #define pot_pin A8

    // Leds
      #define rdy_led 12
      #define error_led 1
      #define reset_led 11

      #define servoPWM 8
     Servo RollServo;
    // stepper
      int step_enable_A_pin = 8;
      int step_enable_B_pin = 16;
      int step_enable_C_pin = 12;

      int roll_stepPin = 10;
      int roll_dirPin = 9;
      int motorInterfaceType = 1;

      int B_stepPin = 14;
      int B_dirPin = 15;

      int RollSpeed = 200;
    
    // stepper declaration
      AccelStepper RollStepper = AccelStepper(motorInterfaceType, roll_stepPin, roll_dirPin);
      AccelStepper BStepper = AccelStepper(motorInterfaceType, B_stepPin, B_dirPin);

    // IO: rajakytkimet

    #define A_Yraj_pin 23
    #define A_Araj_pin 24

    #define B_Yraj_pin 25
    #define B_Araj_pin 26

    #define Mag_raj_pin 27

    int AY_raja = 0;

    bool confirmed = false;

static uint16_t threshold_list[] = 
    { 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000 }; 

void Reset( int idx, int v, int up ) {
  if ( idx == 1 ) ledErr.trigger( ledErr.EVT_OFF );  
  if ( idx == 1 ) ledRdy.trigger( ledRdy.EVT_OFF );
  if ( idx == 1 ) ledRes.trigger( ledRes.EVT_ON );  
  state = 1;
  Serial.println(state);

}
void Ready( int idx, int v, int up ) {
  if(state == 1){
    if ( idx == 1 ) ledRes.trigger( ledRes.EVT_OFF );  
    if ( idx == 1 ) ledRdy.trigger( ledRdy.EVT_ON ); 
    state = 2; 
    Serial.println(state);
    }
}
void Stop( int idx, int v, int up ) {
  if ( idx == 1 ) ledRes.trigger( ledRes.EVT_OFF );   
  if ( idx == 1 ) ledRdy.trigger( ledRdy.EVT_OFF );  
  if ( idx == 1 ) ledErr.trigger( ledErr.EVT_ON );  
  state = 3;
  Serial.println(state);
}

void Test( int idx, int v, int up ){
  confirmed = true;
}

void Calib( int idx, int v, int up ) {
  if (state != 2){
    RollSpeed = analogRead(pot_pin);
    state = 4;
    Serial.println(state);
    Serial.println(RollSpeed);
  }

}
void cmp_callback( int idx, int v, int up ) {
  RollSpeed = analogRead(pot_pin);
  Serial.println(RollSpeed);
}

void setup() {
 //Serial.begin(9600);
  ledRdy.begin(12);
  ledErr.begin(1);
  ledRes.begin(11);
  Calibration.begin(7)
    .onPress(Calib, 1);

  RdyBTN.begin(2)
    .onPress(Ready, 1);

  ResetBTN.begin(3)
    .onPress(Reset, 1);

  StopBTN.begin(4)
    .onPress(Stop, 1);
  

  cmp.begin( A8, 50 )
    .threshold( threshold_list, sizeof( threshold_list ) )
    .onChange( Calib, 99 );

  BStepper.setMaxSpeed(2000);
  BStepper.setAcceleration(100000);
  RollStepper.setMaxSpeed(20000);
  RollStepper.setSpeed(RollSpeed); 

  RollServo.attach(servoPWM);
}

void loop() {

  if (state == 1){
    ResetLog();
  }

  if(state == 2){
    //roll();
    //BRoll();
    ServoFW();
  }

  automaton.run();
}

void ResetLog(){

  //Pursottimen nollaus
  AY_raja = digitalRead(A_Yraj_pin);

  if (AY_raja == 0){
    digitalWrite(step_enable_B_pin, HIGH);
    BStepper.setSpeed(1000);
    BStepper.run();
  }

  //Kuittaus (testinappi)
  TestBTN.begin(7)
    .onPress(Test, 1);
  // Jos raja = 1, confirm = true, luukku = 1
  /*
  if (Mag_raj = 0){
    MagStepper.setSpeed(1000);
    MagStepper.run();
  }
  */
}

void RunLog(){

}

void ServoFW(){

  int pos = 0;
if (state == 2){
  for (pos = 0; pos <= 360; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    RollServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 360; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    RollServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
}

void ServoBW(){

}

void BRoll(){
  step_enable_B = true;
  BStepper.setSpeed(1000);
  BStepper.runSpeed();
}

void roll(){
  step_enable_A = true;
  RollStepper.setSpeed(RollSpeed); 
  RollStepper.runSpeed();
 return 0;
}

