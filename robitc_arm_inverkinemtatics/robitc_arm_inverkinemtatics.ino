#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

hd44780_I2Cexp lcd;
const int buttonPickup = 5;   

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  150 
#define SERVOMAX  750 

#define BASE_JOINT    0
#define SHOULDER_JOINT 1
#define ELBOW_JOINT   2
#define WRIST_JOINT   3
#define GRIPPER_JOINT 4

// Track current angles to enable smooth movement
int currentAngles[5] = {90, 90, 90, 90, 40};


float L0 = 8.5;   // shoulder pivot height above base
float L1 = 12.5;  // shoulder to elbow
float L2 = 5.0;   // elbow to wrist

struct Angles {
  float base;
  float shoulder;
  float elbow;
  float wrist;
};


float clampVal(float x, float lo, float hi) {
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}


Angles inverseKinematics(float x, float y, float z) {
  Angles result;

 
  result.base = atan2(y, x) * 180.0 / M_PI; // degrees


  float R = sqrt(x*x + y*y);

 
  float dz = z - L0;        
  float d  = sqrt(R*R + dz*dz);

  // 4) Elbow angle
  float numerator   = L1*L1 + L2*L2 - d*d;
  float denominator = 2.0 * L1 * L2;
  float cElbow = clampVal(numerator / denominator, -1.0, 1.0);
  float elbowRad = acos(cElbow);          // radians
  result.elbow   = elbowRad * 180.0 / M_PI; // convert to deg

// Shoulder angle
  float phi = atan2(dz, R); // angle from horizontal plane
  float cShoulder = clampVal((d*d + L1*L1 - L2*L2) / (2.0 * L1 * d), -1.0, 1.0);
  float psi  = acos(cShoulder);
  float shoulderRad = phi + psi;
  result.shoulder   = shoulderRad * 180.0 / M_PI; // deg

// Wrist angle (fixed)
  
  result.wrist = 75.0;

  return result;
}


void setServoAngle(uint8_t channel, int angle) {
  if(angle < 0)   angle = 0;
  if(angle > 180) angle = 180;
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
  currentAngles[channel] = angle;
}

void slowSetServoAngle(uint8_t channel, int targetAngle, int stepDelay = 15) {
  int startAngle = currentAngles[channel];
  int step = (targetAngle > startAngle) ? 1 : -1;
  while(startAngle != targetAngle) {
    startAngle += step;
    setServoAngle(channel, startAngle);
    delay(stepDelay);
  }
}

void openGripper() {
  slowSetServoAngle(GRIPPER_JOINT, 45, 10); // or whatever open angle is
}
void closeGripper() {
  slowSetServoAngle(GRIPPER_JOINT, 0, 10);  // or whatever closed angle is
}

//  Tea Bag Pickup Routine 
void pickUpTeabag(float xObj, float yObj, float zObj) {
  //  Compute the IK angles for the tea bag position
  Angles ik = inverseKinematics(xObj, yObj, zObj);

  // 2) Move base
  slowSetServoAngle(BASE_JOINT, (int)ik.base, 20);
  // 3) Move shoulder
  slowSetServoAngle(SHOULDER_JOINT, (int)ik.shoulder, 20);
  // 4) Move elbow
  slowSetServoAngle(ELBOW_JOINT, (int)ik.elbow, 20);
  // 5) Move wrist
  slowSetServoAngle(WRIST_JOINT, (int)ik.wrist, 20);

  // 6) Close gripper
  closeGripper();
  delay(500);

  // Optionally lift the arm a bit so the bag is picked up
  slowSetServoAngle(ELBOW_JOINT, (int)(ik.elbow - 20), 20);
}

void setup() {
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  
  // Initialize servo driver
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Initialize button
  pinMode(buttonPickup, INPUT_PULLUP);

  // Home position
  setServoAngle(BASE_JOINT, 90);
  setServoAngle(SHOULDER_JOINT, 90);
  setServoAngle(ELBOW_JOINT, 90);
  setServoAngle(WRIST_JOINT, 90);
  setServoAngle(GRIPPER_JOINT, 45); // Open

  lcd.clear();
  lcd.print("Ready");
}

void loop() {
  // If button is pressed (active LOW), pick up
  if(digitalRead(buttonPickup) == LOW) {
    lcd.clear();
    lcd.print("Picking up...");

    // Example coordinates for the tea bag
    float teaX = 10.0; // cm
    float teaY = 5.0;  // cm
    float teaZ = 2.0;  // cm (above table)

    pickUpTeabag(teaX, teaY, teaZ);

    lcd.clear();
    lcd.print("Done");
    delay(2000);
  }
}
