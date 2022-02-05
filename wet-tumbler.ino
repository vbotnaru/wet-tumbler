#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <StateMachine.h>

#define CLK  2 // LCD I2C
#define DT   3 // LCD I2C
#define SW   4 // No interrupt encoder button
#define LOAD 5 // Relay

LiquidCrystal_I2C lcd(0x27, 16, 2);

byte lastStateCLK;
byte direction = 0;   // 0 - no dir, 1 - CW, 2 - CCW 
byte lvl = 0;         // current level of the menu (should not have more than 255 levels ;))
bool running = false; // indicates motor state

unsigned long lastButtonPress = 0;

const unsigned long STATE_DELAY = 100;

StateMachine machine = StateMachine();

State* S0 = machine.addState(&state0); 
State* S1 = machine.addState(&state1);
State* S2 = machine.addState(&state2);
// State* S3 = machine.addState(&state3);
// State* S4 = machine.addState(&state4);
// State* S5 = machine.addState(&state5);

void setup() {
  Serial.begin(115200);

  // Setup LCD
  lcd.init();
  lcd.backlight();

  // Set encoder pins as inputs
  pinMode(CLK,  INPUT);
  pinMode(DT,   INPUT);
  pinMode(SW,   INPUT_PULLUP);
  pinMode(LOAD, OUTPUT);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

  // Set relay to OFF
  digitalWrite(LOAD, LOW);
  
  // Uno, Nano has only two interrupt pins 2,3
  // Use other model to enable interupt for press button
  attachInterrupt(digitalPinToInterrupt(2), handleEncoderDirChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), handleEncoderDirChange, CHANGE);

  S0->addTransition(&transitionS0S1,S1);
  
  S1->addTransition(&transitionS1S0,S0);
  S1->addTransition(&transitionS1S2,S2);
  
  S2->addTransition(&transitionS2S1,S1);
//  S2->addTransition(&transitionS2S3,S3);
//  S2->addTransition(&transitionS2S3,S3);
//  S3->addTransition(&transitionS3S4,S4);
//  S4->addTransition(&transitionS4S5,S5);
//  S5->addTransition(&transitionS5S0,S0);
//  S5->addTransition(&transitionS5S2,S2);
}

void handleMotorState() {
  if (running) {
    digitalWrite(LOAD, HIGH);
  } else {
    digitalWrite(LOAD, LOW);
  }
}

void handleEncoderDirChange() {
  byte curStateCLK = digitalRead(CLK);
  if (curStateCLK != lastStateCLK  && curStateCLK == HIGH){
    direction = digitalRead(DT) != curStateCLK ? 2 : 1;
 }
  lastStateCLK = curStateCLK;
}

void handleEncoderClickChange() {
  int btnState = digitalRead(SW);
  if (btnState == LOW) {
    if (millis() - lastButtonPress > 500) {
      switch (machine.currentState) {
        case 0:
          handleState0Click();
          break;
        case 1:
          // statements
          break;
        default:
          // statements
          break;
      }
    }
    lastButtonPress = millis();
  }
  delay(1);
}

bool next() {
  bool tr = false;
  if (direction == 1) {
    direction = 0;
    tr = true;
  }
  return tr;
}

bool prev() {
  bool tr = false;
  if (direction == 2) {
    direction = 0;
    tr = true;
  }
  return tr;
}

void renderItem0() {
  if (running) {
    lcd.print("Stop");
  } else {
    lcd.print("Start");
  }
}

void renderItem1() {
  lcd.print("Menu 2");
}

void renderItem2() {
  lcd.print("Menu 3");
}

//***************STATE 0**********//
void state0(){
  if(machine.executeOnce){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(">");
    renderItem0();
    lcd.setCursor(1,1);
    renderItem1();  
  }
}

bool transitionS0S1(){
  return next() && lvl == 0;
}

void handleState0Click() {
  running = !running;
  handleMotorState();
  // TO RERENDER CURRENT STATE 
  machine.transitionTo(machine.currentState);
}

//***************STATE 1**********//
void state1(){
  if(machine.executeOnce){
    lcd.clear();
    lcd.setCursor(1,0);
    renderItem0();
    lcd.setCursor(0,1);
    lcd.print(">");
    renderItem1();
  }
}

bool transitionS1S0(){
  return prev() && lvl == 0;
}

bool transitionS1S2(){
  return next() && lvl == 0;
}

//***************STATE 2**********//
void state2(){
  if(machine.executeOnce){
    lcd.clear();
    lcd.setCursor(1,0);
    renderItem1();
    lcd.setCursor(0,1);
    lcd.print(">");
    renderItem2();
  }
}

bool transitionS2S1(){
  return prev() && lvl == 0;
}

// bool transitionS2S3(){
//   return next() && lvl == 0;
// }

//------------------------
// void state3(){
//   //Serial.println("State 3");
//   lcd.setCursor(0,1);
//   lcd.print("State 3");
// }

// bool transitionS3S4(){
//   return true;
// }

//-------------------------
// void state4(){
//   //Serial.println("State 4");
//   lcd.setCursor(0,1);
//   lcd.print("State 4");
// }

// bool transitionS4S5(){
//   return true;
// }

//-------------------------
// void state5(){
//   //Serial.println("State 5");
//   lcd.setCursor(0,1);
//   lcd.print("State 5");
// }

// bool transitionS5S0(){
//   return random(0,2);
// }

// bool transitionS5S2(){
//   return true;
// }

// MAIN LOOP
void loop() {
  machine.run();
  //TODO: move to use interrupts for the supported boards
  handleEncoderClickChange();  
  delay(STATE_DELAY);
}