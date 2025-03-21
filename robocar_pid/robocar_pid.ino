/**
 * PID Distance Controller for Robot Car
 * MIT 1.104 Lab 7 PID Control Systems
 * 
 * This program implements a PID controller to maintain a fixed distance
 * between the robot car and an object using an ultrasonic sensor.
 * The distance and control data are logged to an SD card.
 * 
 * Hardware:
 * - ESP32 development board
 * - HC-SR04 ultrasonic sensor
 * - L298N motor driver or similar
 * - SD card module
 * - Robot car chassis with DC motors
 */

#include <SD.h>
#include <SPI.h>

// Pin Definitions
const int TRIGGER_PIN = 27;    // HC-SR04 trigger pin
const int ECHO_PIN = 26;      // HC-SR04 echo pin
const int MOTOR_IN1 = 17;     // L298N Input 1
const int MOTOR_IN2 = 16;     // L298N Input 2
const int MOTOR_IN3 = 14;     // L298N Input 3
const int MOTOR_IN4 = 13;     // L298N Input 4
const int MOTOR_ENA = 25;     // L298N Enable A (PWM for right motor)
const int MOTOR_ENB = 4;     // L298N Enable B (PWM for left motor)
const int SD_CS = 5;          // SD card module CS pin

// PWM Configuration
const int PWM_FREQ = 30000;    // PWM frequency for motor control
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

// PID Control Constants (adjusted for meter units)
float Kp = 150.0;       // Proportional gain (reduced from 500.0)
float Ki = 0.0;        // Integral gain (reduced from 100.0)
float Kd = 100.0;        // Derivative gain (reduced from 200.0)

// PID Variables
const float TARGET_DISTANCE = 0.20;  // Target distance in m (was 20 cm)
float prevError = 0.0;              // Previous error
float integral = 0.0;               // Integral accumulator
float derivative = 0.0;             // Derivative term
float previousDistance = 0.0;       // Previous distance reading
unsigned long prevTime = 0;         // Previous time for dt calculation

// Control Variables
const int BASE_SPEED = 150;           // Base motor speed (0-255)
const int MAX_SPEED = 200;            // Maximum motor speed (reduced from 255)
const int MIN_SPEED = 0;              // Minimum motor speed
const int SAMPLE_TIME = 50;           // PID sample time in milliseconds

// Double Integrator Model Variables
float currentSpeed = 0.0;             // Current speed of the car in m/s (can be negative)
const float MAX_ACCEL = 0.3;          // Maximum acceleration in m/s² (reduced from 0.5)
const float FRICTION = 0.0;           // Friction coefficient to simulate real-world damping
const float MAX_REAL_SPEED = 0.8;     // Maximum speed in m/s (reduced from 1.5)
const float PWM_TO_SPEED = MAX_REAL_SPEED / MAX_SPEED;  // Conversion factor from PWM to m/s
const float SPEED_TO_PWM = MAX_SPEED / MAX_REAL_SPEED;  // Conversion factor from m/s to PWM

// SD Card Variables
File dataFile;
const String FILENAME = "/pid_data" + String(millis()) + ".csv";
bool sdCardAvailable = false;

// Function declarations
float getDistance();
void moveForward(int speedLeft, int speedRight);
void moveBackward(int speedLeft, int speedRight);
void stopMotors();
float computePID(float distance);
void logData(unsigned long time, float distance, float error, float acceleration);
void setupMotors();
bool setupSDCard();

void setup() {
    Serial.begin(115200);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Configure motor driver pins
    setupMotors();
    
    // Initialize SD card
    sdCardAvailable = setupSDCard();
    
    if (sdCardAvailable) {
        // Create or open the data file and write header
        dataFile = SD.open(FILENAME, FILE_WRITE);
        if (dataFile) {
            dataFile.println("Time(ms),Distance(m),Error(m),Acceleration,Speed,Kp,Ki,Kd");
            dataFile.println(String(0) + "," + String(0) + "," + String(0) + "," + String(0) + "," + String(0) + "," + 
                            String(Kp) + "," + String(Ki) + "," + String(Kd));
            dataFile.close();
            Serial.println("Setup SD card successfully");
        } else {
            Serial.println("Error opening data file!");
        }
    }

    // Wait a moment before starting
    delay(1000);
    
    Serial.println("PID Distance Controller Starting");
    Serial.println("Target distance: 0.20 m");
    Serial.println("PID parameters: Kp=" + String(Kp) + ", Ki=" + String(Ki) + ", Kd=" + String(Kd));
    
    // Initialize timing variables
    prevTime = millis();
}

void loop() {
    // Get current time
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - prevTime;
    
    // Check if it's time to run the PID loop
    if (elapsedTime >= SAMPLE_TIME) {
        // Measure the current distance
        float distance = getDistance();
        
        if (distance > 0) {  // Valid distance reading
            // Compute PID output as acceleration command
            float acceleration = computePID(distance);

            if (abs(acceleration) < 0.6) {
              acceleration = 0.0;
            }
            
            // Calculate dt in seconds
            float dt = elapsedTime / 1000.0;
            
            // Update speed based on acceleration (first integration)
            currentSpeed += acceleration * dt;
            
            // Apply simple friction model
            // currentSpeed -= FRICTION * currentSpeed * dt;
            
            // Cap the speed to ensure the car doesn't move too fast
            currentSpeed = constrain(currentSpeed, -MAX_REAL_SPEED, MAX_REAL_SPEED);
            
            // Convert speed in m/s to motor PWM values
            int motorSpeed = abs(round(currentSpeed * SPEED_TO_PWM));
            motorSpeed = constrain(motorSpeed, 0, MAX_SPEED);
            
            // Apply control to motors based on speed direction
            if (currentSpeed > 0.05) {  // Small deadband to prevent jitter
                // Move forward (car is too far from object)
                moveForward(motorSpeed, motorSpeed);
            } else if (currentSpeed < -0.05) {
                // Move backward (car is too close to object)
                moveBackward(motorSpeed, motorSpeed);
            } else {
                // Stop motors if speed is very close to zero
                stopMotors();
            }
            
            // Log the data
            float error = TARGET_DISTANCE - distance;
            logData(currentTime, distance, error, acceleration);
            
            // Print data to serial monitor
            Serial.print("Time: ");
            Serial.print(currentTime);
            Serial.print(" ms, Distance: ");
            Serial.print(distance);
            Serial.print(" m, Error: ");
            Serial.print(error);
            Serial.print(" m, Speed: ");
            Serial.print(motorSpeed);
            Serial.print(" acceleration: ");
            Serial.print(acceleration);
            Serial.println(" ");
            
            // Update previous time
            prevTime = currentTime;
        } else {
            Serial.println("Invalid distance reading!");
            stopMotors();
        }
    }
}

float getDistance() {
    // Send ultrasonic pulse
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    // Measure the echo pulse duration
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout after 30ms
    
    // Calculate distance in meters (was in centimeters)
    float distance = duration * 0.000344 / 2;  // 0.034/100 = 0.000344
    
    // Filter out invalid readings
    if (distance > 4.0 || distance <= 0) {  // 400cm = 4.0m
        return -1;  // Invalid reading
    }
    
    return distance;
}

float computePID(float distance) {
    // Calculate error
    float error = TARGET_DISTANCE - distance;
    
    // Calculate dt in seconds
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    
    // Ensure dt is reasonable (in case of overflow or too much time elapsed)
    if (dt <= 0 || dt > 1.0) {
        dt = SAMPLE_TIME / 1000.0;  // Use expected sample time as fallback
    }
    
    // Calculate integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, -30, 30);  // Reduced integral limit
    
    // Calculate derivative term (using filter to reduce noise)
    // Using rate of change of measurement instead of rate of change of error
    float distanceChange = distance - previousDistance;
    derivative = -distanceChange / dt;  // Negative because distance increasing means error decreasing
    
    // Apply simple low-pass filter to derivative term
    derivative = 0.0 * derivative + 1.0 * (error - prevError) / dt;
    
    // Calculate PID output (acceleration command)
    float control = Kp * error + Ki * integral + Kd * derivative;
    
    // Limit acceleration command
    // control = constrain(control, -MAX_ACCEL, MAX_ACCEL);
    
    // Update previous values
    prevError = error;
    previousDistance = distance;
    
    return control;
}

void setupMotors() {
    // Configure motor control pins
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    
    // Configure PWM channels for motor speed control
    ledcAttach(MOTOR_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_ENB, PWM_FREQ, PWM_RESOLUTION);
    
    // Initially stop motors
    stopMotors();
}

bool setupSDCard() {
    Serial.println("Initializing SD card...");
    
    if (!SD.begin(SD_CS)) {
        Serial.println("SD card initialization failed!");
        return false;
    }
    
    Serial.println("SD card initialized successfully.");
    return true;
}

void moveForward(int speedLeft, int speedRight) {
    // Set direction pins for forward movement
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    digitalWrite(MOTOR_IN3, HIGH);
    digitalWrite(MOTOR_IN4, LOW);
    
    // Set motor speeds
    ledcWrite(MOTOR_ENA, speedLeft);
    ledcWrite(MOTOR_ENB, speedRight);
}

void moveBackward(int speedLeft, int speedRight) {
    // Set direction pins for backward movement
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, HIGH);
    
    // Set motor speeds
    ledcWrite(MOTOR_ENA, speedLeft);
    ledcWrite(MOTOR_ENB, speedRight);
}

void stopMotors() {
    // Set all direction pins LOW or set them to brake configuration
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);
    
    // Set speeds to 0
    ledcWrite(MOTOR_ENA, 0);
    ledcWrite(MOTOR_ENB, 0);
}

void logData(unsigned long time, float distance, float error, float control) {
    if (!sdCardAvailable) return;
    
    dataFile = SD.open(FILENAME, FILE_APPEND);
    if (dataFile) {
        dataFile.println(String(time) + "," + String(distance) + "," + 
                        String(error) + "," + String(control) + "," + String(currentSpeed));
        dataFile.close();
    } else {
        Serial.println("Error opening data file for logging!");
    }
}