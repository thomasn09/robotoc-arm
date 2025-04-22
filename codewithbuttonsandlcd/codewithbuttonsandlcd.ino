#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_PWMServoDriver.h>

// ***** LCD & Button Setup *****
hd44780_I2Cexp lcd;             // LCD connected via I2C expander
const int button1 = 5;          // Button to trigger teabag pickup (active LOW)
const int button2 = 4;          // (Optional) Another button
const int button3 = 2;  

// ***** Servo Driver Setup *****
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150  // Minimum pulse length for 0°
#define SERVOMAX 750  // Maximum pulse length for 180°

#define LED_MODE1 33    // LED indicator (if used)
#define LED_MODE2 32 
#define LED_MODE3 35 

// ***** Joint Definitions *****
#define BASE_JOINT      0

#define SHOULDER_JOINT  1//these two move together
#define SHOULDER_JOINT2 2

#define SECOND_JOINT    3
#define THIRD_JOINT     4

#define GRIPPER_JOINT   6
#define PUMP1_JOINT     7
#define PUMP2_JOINT     8

// ***** Current Angles Array (for servo tracking) *****
int currentAngles[9] = {45, 15, 90 - 15, 60, 45, 10, 45, 0, 0};

// ***** Servo Functions *****
void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
  currentAngles[channel] = angle;
}

void setPumpAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 360, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void slowSetServoAngle(uint8_t channel, int targetAngle, int stepDelay = 25) {
  int startAngle = currentAngles[channel];
  if (startAngle == targetAngle) return;
  int step = (targetAngle > startAngle) ? 1 : -1;
  for (int angle = startAngle; angle != targetAngle; angle += step) {
    setServoAngle(channel, angle);
    delay(stepDelay);
  }
  setServoAngle(channel, targetAngle);
}

void slowSetTwoServoAngles(uint8_t channel1, int targetAngle1, uint8_t channel2, int targetAngle2, int stepDelay = 25) {
  int start1 = currentAngles[channel1];
  int start2 = currentAngles[channel2];
  int diff1 = targetAngle1 - start1;
  int diff2 = targetAngle2 - start2;
  int steps1 = abs(diff1);
  int steps2 = abs(diff2);
  int steps = (steps1 > steps2) ? steps1 : steps2;
  if (steps == 0) return;
  
  for (int i = 1; i <= steps; i++) {
    int newAngle1 = start1 + (diff1 * i) / steps;
    int newAngle2 = start2 + (diff2 * i) / steps;
    setServoAngle(channel1, newAngle1);
    setServoAngle(channel2, newAngle2);
    delay(stepDelay);
  }
  setServoAngle(channel1, targetAngle1);
  setServoAngle(channel2, targetAngle2);
}

void releaseobject() {
  slowSetServoAngle(GRIPPER_JOINT, 45, 10);//gripper is open
  
}
void grabobject() {
  slowSetServoAngle(GRIPPER_JOINT, 0, 10);//gripper is closed
  
}


void pump1mode() {
  //PoringPostion();
 setPumpAngle(PUMP1_JOINT, 180);
  delay(pumpduration);
}
void pump2mode() {
  //PoringPostion();
 setPumpAngle(PUMP2_JOINT, 180);
  delay(pump2duration);
}

// homemode: positions the arm in its home state.

void homemode() {
  // Turn pumps off
  setPumpAngle(PUMP1_JOINT, 90);
  setPumpAngle(PUMP2_JOINT, 90);
  digitalWrite(LED_MODE1, HIGH);

  slowSetServoAngle(BASE_JOINT, 55, 30);
  slowSetTwoServoAngles(SHOULDER_JOINT, 20, SHOULDER_JOINT2, 90 - 20, 40);
  slowSetServoAngle(SECOND_JOINT, 45, 30);
  slowSetServoAngle(THIRD_JOINT, 50, 30);
  slowSetServoAngle(FOURTH_JOINT, 10, 30);
  slowSetServoAngle(GRIPPER_JOINT, 50, 20); // Open gripper
  
  digitalWrite(LED_MODE1, LOW);
  
  // Update LCD to show Home Mode
  lcd.clear();
  lcd.print("Home Mode");
}

// pickingupteabag mode: executes the teabag pickup routine.

void testmode(){
 
slowSetServoAngle(BASE_JOINT, 80, 30);
delay(100);
  slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 25, SHOULDER_JOINT2, 90 - 25, 30);//move into teabag psotion
 delay(1000);
slowSetServoAngle(SECOND_JOINT, 50, 30);
delay(100);
delay(100);
slowSetServoAngle(THIRD_JOINT, 75,30);
delay(100);
grabobject();
delay(100);
  slowSetTwoServoAngles(SHOULDER_JOINT, 20, SHOULDER_JOINT2, 90 - 20, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
slowSetServoAngle(SECOND_JOINT, 45, 30);
slowSetServoAngle(SECOND_JOINT, 35, 30);
delay(100);
slowSetServoAngle(THIRD_JOINT, 55,30);
slowSetServoAngle(THIRD_JOINT, 45,30);
delay(100);
slowSetServoAngle(BASE_JOINT, 60, 30);
delay(100);

slowSetServoAngle(THIRD_JOINT, 55,30);
releaseobject();

}
void teabagremoval(){
slowSetServoAngle(BASE_JOINT, 60, 30);
 slowSetTwoServoAngles(SHOULDER_JOINT, 20, SHOULDER_JOINT2, 90 - 20, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
slowSetServoAngle(SECOND_JOINT, 45, 30);
slowSetServoAngle(SECOND_JOINT, 35, 30);
delay(100);
slowSetServoAngle(THIRD_JOINT, 55,30);
slowSetServoAngle(THIRD_JOINT, 45,30);
delay(100);
slowSetServoAngle(BASE_JOINT, 60, 30);
delay(100);

slowSetServoAngle(THIRD_JOINT, 55,30);

  slowSetServoAngle(FOURTH_JOINT,45, 30);
grapobject();


slowSetServoAngle(SECOND_JOINT, 45, 40);
delay(100);
slowSetServoAngle(THIRD_JOINT, 55,30);
slowSetServoAngle(BASE_JOINT, 20, 30);
 slowSetTwoServoAngles(SHOULDER_JOINT, 30, SHOULDER_JOINT2, 90 - 30, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 35, SHOULDER_JOINT2, 90 - 35, 30);//move into teabag psotion
 delay(100);

releaseobject();



}



void milkmode (){
  slowSetServoAngle(FOURTH_JOINT, 0, 30);
   slowSetServoAngle(SECOND_JOINT, 35, 30);
slowSetServoAngle(SECOND_JOINT, 55, 30);
  slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 25, SHOULDER_JOINT2, 90 - 25, 30);//move into teabag psotion
 delay(100);

delay(100);
slowSetServoAngle(THIRD_JOINT, 65,30);

 
 setPumpAngle(PUMP2_JOINT, 180);
 delay(5000);
setPumpAngle(PUMP2_JOINT, 90);

}


void watermode (){
  slowSetServoAngle(FOURTH_JOINT, 0, 30);
   slowSetServoAngle(SECOND_JOINT, 35, 30);
slowSetServoAngle(SECOND_JOINT, 55, 30);
  slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 25, SHOULDER_JOINT2, 90 - 25, 30);//move into teabag psotion
 delay(100);

delay(100);
slowSetServoAngle(THIRD_JOINT, 65,30);


 setPumpAngle(PUMP1_JOINT, 180);
 delay(5000);
setPumpAngle(PUMP1_JOINT, 90);


}
void sugarcubemode(){
   slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
slowSetServoAngle(BASE_JOINT, 90, 30);
delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 25, SHOULDER_JOINT2, 90 - 25, 30);//move into teabag psotion
 delay(1000);
slowSetServoAngle(SECOND_JOINT, 50, 30);
delay(100);
slowSetServoAngle(THIRD_JOINT, 60,30);
slowSetServoAngle(THIRD_JOINT, 75,30);
delay(100);
grabobject();
delay(100);
  slowSetTwoServoAngles(SHOULDER_JOINT, 20, SHOULDER_JOINT2, 90 - 20, 30);//move into teabag psotion
 delay(100);
 slowSetTwoServoAngles(SHOULDER_JOINT, 15, SHOULDER_JOINT2, 90 - 15, 30);//move into teabag psotion
 delay(100);
slowSetServoAngle(SECOND_JOINT, 45, 30);
slowSetServoAngle(SECOND_JOINT, 35, 30);
delay(100);
slowSetServoAngle(THIRD_JOINT, 55,30);
slowSetServoAngle(THIRD_JOINT, 45,30);
delay(100);
slowSetServoAngle(BASE_JOINT, 60, 30);
delay(100);

slowSetServoAngle(THIRD_JOINT, 55,30);
releaseobject();


}

// ***** Setup & Loop *****
void setup() {
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  
  // Initialize buttons
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
   pinMode(button3, INPUT_PULLUP);
  
  // Initialize servo driver.
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for servos
  delay(10);
  
  // LED output.
  pinMode(LED_MODE1, OUTPUT);
  pinMode(LED_MODE2, OUTPUT);
  pinMode(LED_MODE2, OUTPUT);
  
  // Set the arm to home mode and update LCD.
  homemode();
}

void loop() {
  // Wait here until button1 is pressed.
  bool button1State = (digitalRead(button1) == LOW);
  bool button2State = (digitalRead(button2) == LOW);
   bool button3State = (digitalRead(button2) == LOW);

  lcd.print("homemode");
  if (button2State) {
    // Indicate the mode change on the LCD.
    lcd.clear();
    lcd.print("tea mode is running");
    delay(500);  // brief debounce/delay
    homemode();
    delay(1000);
  
  }

  if (button1State){
 lcd.clear();
  lcd.print("tea mode");

  testmode();
lcd.clear();
  lcd.print("is teabg in the cup?");
  
  lcd.clear();
  lcd.print("water mode");
  watermode();

  lcd.clear();
  lcd.print("more water?");
 
  if (digitalRead(button3) == LOW) { // button pressed (active LOW)
    lcd.clear();
    lcd.print("Pump2 Running");
    while(digitalRead(button3) == LOW) { // while button3 is held down
      setPumpAngle(PUMP2_JOINT, 180);
      // You could add a small delay here to avoid rapid looping:
      delay(50);
    }
    // Once the button is released, turn pump2 off:
    setPumpAngle(PUMP2_JOINT, 90);
    lcd.clear();
    lcd.print("Pump2 Stopped");
  }
  
 lcd.clear();
    lcd.print("stewing....");
    delay(5000);

  teabagremoval();

  milkmode();

  lcd.clear();
  lcd.print("sugar mode");
  sugarcubemode();
  
  
  }
  delay(200);
}


