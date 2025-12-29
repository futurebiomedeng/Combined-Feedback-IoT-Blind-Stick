#include <Arduino.h>
#include <TinyGPS++.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Set your own pins
#define SIM800_RX 22
#define SIM800_TX 23
#define GPS_RX 16
#define GPS_TX 17
#define PUSH_BUTTON 4
#define VIBRATION_MOTOR 12
#define MOTOR_PWM_LEFT 18
#define MOTOR_PWM_RIGHT 19
#define BUZZER 13

#define TRIG_LEFT 14 
#define ECHO_LEFT 27
#define TRIG_RIGHT 25
#define ECHO_RIGHT 26
#define TRIG_FRONT 32
#define ECHO_FRONT 33
#define TRIG_BOTTOM 15
#define ECHO_BOTTOM 2

#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define PWM_RESOLUTION 8
#define PWM_FREQUENCY 5000


const float FRONT_THRESHOLD = 50.0;
const float SIDE_THRESHOLD = 30.0;
const float BOTTOM_THRESHOLD = 60.0;
const int BUZZER_DURATION = 5000;
const int GPS_UPDATE_INTERVAL = 5000;

// Set your own PID constant
const double Kp = 0.00; 
const double Ki = 0.00;
const double Kd = 0.00; 
double SETPOINT = 30.0;
const int PWM_MAX = 255;      
const int PWM_MIN = 0;        

// Motor control states
enum MotorState {
    STOP,
    TURN_BACK,
    TURN_FORWARD,
    EMERGENCY_STOP
};

HardwareSerial sim(1);
HardwareSerial neogps(2);
TinyGPSPlus gps;

// PID Controllers
double pidOutputLeft, pidOutputRight;
double integralLeft = 0, integralRight = 0;
double previousErrorLeft = 0, previousErrorRight = 0;
double dt = 0.05; // 50 ms time interval

String phoneNumber = "00000000000000000"; // set your own phone number
volatile bool gpsMode = true;
volatile bool buttonInterrupted = false;

struct GPSData {
    double latitude;
    double longitude;
    bool valid;
};

struct DistanceData {
    float front;
    float left;
    float right;
    float bottom;
};
DistanceData distanceData;

void IRAM_ATTR buttonISR();
float measureDistance(int trigPin, int echoPin);
void setupPins();
void switchToGPSMode();
void switchToGSMMode();
bool sendSMSMessage(const String& message);
void controlVibrationMotor(const DistanceData& distances);
void controlMotors(const DistanceData& distances);

void IRAM_ATTR buttonISR() {
    buttonInterrupted = true;
}

// Setup pin modes and interrupts
void setupPins() {
    ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_LEFT, PWM_CHANNEL_LEFT);
    ledcAttachPin(MOTOR_PWM_RIGHT, PWM_CHANNEL_RIGHT);

    pinMode(PUSH_BUTTON, INPUT_PULLUP);
    pinMode(VIBRATION_MOTOR, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_BOTTOM, OUTPUT);
    pinMode(ECHO_BOTTOM, INPUT);

    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON), buttonISR, FALLING);
}

// Ultrasonic distance measurement
float measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

void taskDistanceSensors(void *pvParameters) {
    while (true) {
        distanceData.front = measureDistance(TRIG_FRONT, ECHO_FRONT);
        delay(20);
        distanceData.left = measureDistance(TRIG_LEFT, ECHO_LEFT);
        delay(20);
        distanceData.right = measureDistance(TRIG_RIGHT, ECHO_RIGHT);
        delay(20);
        distanceData.bottom = measureDistance(TRIG_BOTTOM, ECHO_BOTTOM);

        controlVibrationMotor(distanceData);
        controlMotors(distanceData);

        Serial.print("Front: "); Serial.print(distanceData.front);
        Serial.print(" cm, Left: "); Serial.print(distanceData.left);
        Serial.print(" cm, Right: "); Serial.print(distanceData.right);
        Serial.print(" cm, Bottom: "); Serial.println(distanceData.bottom);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Vibration motor control based on distance thresholds
void controlVibrationMotor(const DistanceData& distances) {
    if (distances.front < FRONT_THRESHOLD || distances.bottom > BOTTOM_THRESHOLD) {
        digitalWrite(VIBRATION_MOTOR, HIGH);
    } else {
        digitalWrite(VIBRATION_MOTOR, LOW);
    }
}

// Motor control based on side distances with PID
void controlMotors(const DistanceData& distances) {
    // Right motor control (clockwise rotation if close to an obstacle on the right)
    if (distances.right < SIDE_THRESHOLD) {
        double error = SETPOINT - distances.right;
        integralRight += error * dt;
        double derivative = (error - previousErrorRight) / dt;
        pidOutputRight = Kp * error + Ki * integralRight + Kd * derivative;
        previousErrorRight = error;
        ledcWrite(PWM_CHANNEL_RIGHT, constrain(pidOutputRight, PWM_MIN, PWM_MAX));
    }
    // Left motor control (counterclockwise rotation if close to an obstacle on the left)
    else if (distances.left < SIDE_THRESHOLD) {
        double error = SETPOINT - distances.left;
        integralLeft += error * dt;
        double derivative = (error - previousErrorLeft) / dt;
        pidOutputLeft = Kp * error + Ki * integralRight + Kd * derivative;
        previousErrorLeft = error;
        ledcWrite(PWM_CHANNEL_LEFT, constrain(pidOutputLeft, PWM_MIN, PWM_MAX));
    } else {
        ledcWrite(PWM_CHANNEL_LEFT, 0);
        ledcWrite(PWM_CHANNEL_RIGHT, 0);
    }
}

// Task to read GPS data
void taskReadGPS(void *pvParameters) {
    GPSData gpsData;
    while (true) {
        if (gpsMode) {
            while (neogps.available()) {
                if (gps.encode(neogps.read())) {
                    if (gps.location.isValid()) {
                        gpsData.latitude = gps.location.lat();
                        gpsData.longitude = gps.location.lng();
                        gpsData.valid = true;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(GPS_UPDATE_INTERVAL));
    }
}

// Task to send SMS with GPS location on button press
void taskSendSMS(void *pvParameters) {
    GPSData gpsData;

    while (true) {
        if (buttonInterrupted) {
            buttonInterrupted = false;

            // Activate the buzzer
            digitalWrite(BUZZER, HIGH);
            Serial.println("Buzzer ON, preparing to send SMS...");

            // Switch to GSM mode for SMS
            switchToGSMMode();

            // Retrieve the current GPS location
            if (gps.location.isValid()) {
                double latitude = gps.location.lat();
                double longitude = gps.location.lng();
                
                // Create the Google Maps link with the current GPS coordinates
                String message = "http://maps.google.com/maps?q=" + 
                                 String(latitude, 6) + "," + 
                                 String(longitude, 6);

                // Send the SMS
                if (sendSMSMessage(message)) {
                    Serial.println("SMS sent with GPS location.");
                } else {
                    Serial.println("Failed to send SMS.");
                }
            } else {
                Serial.println("GPS location is not valid.");
            }

            delay(BUZZER_DURATION);

            digitalWrite(BUZZER, LOW);
            Serial.println("Buzzer OFF.");

            switchToGPSMode();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// SMS sending function
bool sendSMSMessage(const String& message) {
    sim.println("AT+CMGF=1");  // Set SMS to text mode
    delay(200);
    sim.println("AT+CMGS=\"" + phoneNumber + "\"");  // Send SMS command
    delay(200);
    sim.print(message);  // SMS content
    sim.write(26);  // Send CTRL+Z to end message
    return true;
}

// Switching modes between GPS and GSM
void switchToGPSMode() {
    sim.end();
    neogps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

void switchToGSMMode() {
    neogps.end();
    sim.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);
}

void setup() {
    Serial.begin(115200);
    setupPins();
    xTaskCreate(taskDistanceSensors, "Distance", 4096, NULL, 5, NULL);
    xTaskCreate(taskReadGPS, "GPS", 4096, NULL, 2, NULL);
    xTaskCreate(taskSendSMS, "SMS", 4096, NULL, 1, NULL);
}

void loop() {
    // Empty as tasks manage all operations
}
