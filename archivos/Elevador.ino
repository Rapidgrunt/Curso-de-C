// Include necessary libraries
#include <Arduino.h>

// Define pins for sensors, buttons, display, and motor
const int sensorPins[] = {2, 3, 4, 5};                   // Replace with your actual sensor pin numbers
const int buttonPins[] = {6, 7, 8, 9};                   // Replace with your actual button pin numbers
const int motorEnablePin = 10;                             // Replace with your actual motor enable pin
const int motorAPin = 11;                                  // Replace with your actual motor A pin
const int motorBPin = 12;                                  // Replace with your actual motor B pin
const int displayPins[] = {A0, A1, A2, A3, A4, A5, A6};  // Replace with your actual display segment pin numbers
const int emergencyButtonPin = 13;                         // Replace with your actual emergency button pin

// Define floor constants
const int NUM_FLOORS = 4;
const int GROUND_FLOOR = 0;
const int TOP_FLOOR = NUM_FLOORS - 1;

// Variables
int currentFloor = GROUND_FLOOR;
int GlobalTarget = GROUND_FLOOR;

// Function prototypes
void stopMotor();
void updateDisplay();
void displayfloor1();
void displayfloor2();
void displayfloor3();
void displayfloor4();
void moveToFloor(int targetFloor);
void reachedFloor(int floor);
void moveUp();
void moveDown();
void emergencyStop();

// Variable to store the last time a button state change was detected
unsigned long lastButtonChangeTime = 0;
// Debounce interval in milliseconds
const int debounceInterval = 50;  // Adjust this based on your requirements

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  // Initialize pins
  for (int i = 0; i < NUM_FLOORS; i++) {
    pinMode(sensorPins[i], INPUT);
    pinMode(buttonPins[i], INPUT);
  }
  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorAPin, OUTPUT);
  pinMode(motorBPin, OUTPUT);

  for (int i = 0; i < 7; i++) {
    pinMode(displayPins[i], OUTPUT);
  }

  pinMode(emergencyButtonPin, INPUT);

  // Set initial state
  stopMotor();
  updateDisplay();
}

void loop() {
  // Check for emergency stop
  if (digitalRead(emergencyButtonPin) == LOW) {
    emergencyStop();
  }

  // Check for button presses
  for (int i = 0; i < NUM_FLOORS; i++) {
    if (digitalRead(buttonPins[i]) == HIGH) {
      if (millis() - lastButtonChangeTime > debounceInterval) {
        moveToFloor(i);
        lastButtonChangeTime = millis();
      }
    }
  }

  // Check for sensors
  for (int i = 0; i < NUM_FLOORS; i++) {
   // if (digitalRead(sensorPins[i]) == LOW && i == GlobalTarget) {
    if (digitalRead(sensorPins[i]) == LOW && i == GlobalTarget && currentFloor != i)
      reachedFloor(i);
      updateDisplay();
    
  }
}

void moveToFloor(int targetFloor) {


  // Move the elevator to the target floor
  GlobalTarget = targetFloor;
  if (targetFloor > currentFloor) {
    moveUp();
    updateDisplay();
  } else if (targetFloor < currentFloor) {
    moveDown();
    updateDisplay();
  }
  updateDisplay();
}

void reachedFloor(int floor) {
  // Update current floor
  currentFloor = floor;

  // Stop the motor
  stopMotor();

  // Display the current floor on 7-segment display
  updateDisplay();
}

void moveUp() {
  // Code to move the elevator up
  digitalWrite(motorAPin, HIGH);
  digitalWrite(motorBPin, LOW);
  analogWrite(motorEnablePin, 20);  // Adjust PWM value for desired speed
}

void moveDown() {
  // Code to move the elevator down
  digitalWrite(motorAPin, LOW);
  digitalWrite(motorBPin, HIGH);
  analogWrite(motorEnablePin, 20);  // Adjust PWM value for desired speed
}

void stopMotor() {
  // Stop the motor
  digitalWrite(motorEnablePin, LOW);
}

void updateDisplay() {
  int displayValue = currentFloor;  // Local variable

  if (digitalRead(sensorPins[0]) == LOW) {
    displayfloor1();
  }
    if (digitalRead(sensorPins[1]) == LOW) {
    displayfloor2();
  }
    if (digitalRead(sensorPins[2]) == LOW) {
    displayfloor3();
  }
    if (digitalRead(sensorPins[3]) == LOW) {
    displayfloor4();
  }

}

void emergencyStop() {
  while (digitalRead(sensorPins[0]) == HIGH) { //mientras el sensor no se active que siga bajando el levador
    // Serial.println("Moving down...");
    moveDown();
    delay(50);

    if (digitalRead(sensorPins[0]) == LOW) { // cuando el sensor se active que detenga el elevador
       // Serial.println("Reached ground floor. Stopping motor...");
      stopMotor();
      currentFloor = GROUND_FLOOR;
      updateDisplay();
      break; // Exit the loop
    }
  }

  // Serial.println("Exiting emergency stop loop.");
}

void displayfloor1() {
  digitalWrite(displayPins[0], LOW); // segmento A
  digitalWrite(displayPins[1], HIGH); // segmento B
  digitalWrite(displayPins[2], HIGH); // segmento C
  digitalWrite(displayPins[3], LOW); // segmento D
  digitalWrite(displayPins[4], LOW); // segmento E
  digitalWrite(displayPins[5], LOW); // segmento F
  digitalWrite(displayPins[6], LOW); // segmento G
}

// Add similar functions for displayfloor2, displayfloor3, and displayfloor4
    void displayfloor2() {
digitalWrite(displayPins[0], HIGH); //segmento A
digitalWrite(displayPins[1], HIGH); //segmento B
digitalWrite(displayPins[2], LOW); //segmento C
digitalWrite(displayPins[3], HIGH); //segmento D
digitalWrite(displayPins[4], HIGH); //segmento E
digitalWrite(displayPins[5], LOW); //segmento F
digitalWrite(displayPins[6], HIGH); //segmento G
  }

      void displayfloor3() {
digitalWrite(displayPins[0], HIGH); //segmento A
digitalWrite(displayPins[1], HIGH); //segmento B
digitalWrite(displayPins[2], HIGH); //segmento C
digitalWrite(displayPins[3], HIGH); //segmento D
digitalWrite(displayPins[4], LOW); //segmento E
digitalWrite(displayPins[5], LOW); //segmento F
digitalWrite(displayPins[6], HIGH); //segmento G
  }

        void displayfloor4() {
digitalWrite(displayPins[0], LOW); //segmento A
digitalWrite(displayPins[1], HIGH); //segmento B
digitalWrite(displayPins[2], HIGH); //segmento C
digitalWrite(displayPins[3], LOW); //segmento D
digitalWrite(displayPins[4], LOW); //segmento E
digitalWrite(displayPins[5], HIGH); //segmento F
digitalWrite(displayPins[6], HIGH); //segmento G
  }