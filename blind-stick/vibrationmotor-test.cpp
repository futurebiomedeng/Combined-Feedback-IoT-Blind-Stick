/*
This code is used for testing vibration motor response upon a ultrasonic sensor reading. 
If the ultrasonic sensor reading is below a defined threshold, the vibration motor will starts vibrating and if the sensor reads the distance above threshold, the motor will be disabled.
*/

const int trigPin = 23;
const int echoPin = 32;
const int vibrationMotorPin = 4; 

#define SOUND_SPEED 0.034

long duration;
float distance;

void setup() {
  Serial.begin(115200); 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  pinMode(vibrationMotorPin, OUTPUT); 
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distance = duration * SOUND_SPEED / 2;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distance);
  
  if (distance <= 30) {
    activateVibrationMotor();  // Trigger the vibration motor if the distance is below the threshold
  } else {
    deactivateVibrationMotor();  // Turn off the vibration motor if the distance is above the threshold
  }

  delay(1000); // Delay before the next measurement
}

void activateVibrationMotor() {
  digitalWrite(vibrationMotorPin, HIGH); // Turn on the vibration motor
}

void deactivateVibrationMotor() {
  digitalWrite(vibrationMotorPin, LOW);  // Turn off the vibration motor
}
