/*
This code is used for testing four utrasonic sensors reading without RTOS.
You can add if-statements to test each sensor response.
*/

#define TRIG_LEFT 14
#define ECHO_LEFT 27
#define TRIG_RIGHT 25
#define ECHO_RIGHT 26
#define TRIG_FRONT 32
#define ECHO_FRONT 33
#define TRIG_BOTTOM 15
#define ECHO_BOTTOM 2
#define vibrationMotorPin 12       // Vibration motor pin for obstacle alert

float distanceLeft = 0;
float distanceRight = 0;
float distanceFront = 0;
float distanceBottom = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BOTTOM, OUTPUT);
  pinMode(ECHO_BOTTOM, INPUT);
}

void loop() {
  distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);    // Left sensor
  delay(20);
  distanceRight = getDistance(TRIG_RIGHT, ECHO_RIGHT); // Right sensor
  delay(20);
  distanceFront = getDistance(TRIG_FRONT, ECHO_FRONT); // Front sensor
  delay(20);
  distanceBottom = getDistance(TRIG_BOTTOM, ECHO_BOTTOM); // Bottom sensor

  Serial.print("Left Distance: ");
  Serial.print(distanceLeft);
  Serial.println(" cm");

  Serial.print("Right Distance: ");
  Serial.print(distanceRight);
  Serial.println(" cm");

  Serial.print("Front Distance: ");
  Serial.print(distanceFront);
  Serial.println(" cm");

  Serial.print("Bottom Distance: ");
  Serial.print(distanceBottom);
  Serial.println(" cm");

  delay(100); 
}


float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// // Function to activate the vibration motor
// void activateVibrationMotor() {
//   digitalWrite(vibrationMotorPin, HIGH); // Turn on vibration motor
// }

// // Function to deactivate the vibration motor
// void deactivateVibrationMotor() {
//   digitalWrite(vibrationMotorPin, LOW);  // Turn off vibration motor
// }

// // Function to check for obstacles and activate vibration if conditions are met
// void obstacleAlert() {
//   // Measure distance from the front ultrasonic sensor
//   float frontDistance = getDistance(TRIG_FRONT, ECHO_FRONT);
//   // Measure distance from the bottom ultrasonic sensor
//   float belowDistance = getDistance(TRIG_BOTTOM, ECHO_BOTTOM);

//   // Front Sensor Logic: Activate vibration if obstacle is within 5 cm
//   bool frontAlert = (frontDistance < 5);

//   // Activate vibration motor if either frontAlert or belowAlert is true
//   if (frontAlert == true) {
//     activateVibrationMotor();
//   } else {
//     deactivateVibrationMotor();
//   }
// }
