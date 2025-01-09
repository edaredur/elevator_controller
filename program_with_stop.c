#include <Servo.h>

// Library Servo
Servo myServo;

// Pin sensor ultrasonik
const int trigPin = 10;
const int echoPin = 11;

// Pin push button
const int buttonPins[] = {2, 3, 4, 5}; // Button untuk lantai 1, 2, 3, 4
const int numFloors = 4;

// Jarak untuk masing-masing lantai (dalam cm)
const int floorHeights[] = {47, 33, 19, 6};

// Variabel PID
double Kp = 1.0, Ki = 0.1, Kd = 0.5;
double previousError = 0, integral = 0;

// Variabel sistem
int targetFloor = -1;
double targetHeight = 0;
double initialHeight = 0;  // Ketinggian awal sebelum gerakan

// Variabel analisis waktu respons
unsigned long startTime = 0;  // Waktu mulai respons
bool steadyStateReached = false;
bool delayTimeFound = false;
bool riseTimeStartFound = false;
bool riseTimeEndFound = false;
bool peakTimeFound = false;
double peakValue = 0;
unsigned long delayTime = 0;
unsigned long riseTimeStart = 0;
unsigned long riseTimeEnd = 0;
unsigned long peakTime = 0;
unsigned long settlingTime = 0;

void setup() {
  myServo.attach(9);
  myServo.write(90);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  for (int i = 0; i < numFloors; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  Serial.begin(9600);
}

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

double computePID(double setpoint, double currentHeight) {
  double error = setpoint - currentHeight;
  integral += error;
  double derivative = error - previousError;
  previousError = error;
  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

void moveServo(double output) {
  int servoSignal = 90 - (int)output;
  servoSignal = constrain(servoSignal, 0, 180);
  myServo.write(servoSignal);
}

void resetResponseVariables() {
  steadyStateReached = false;
  delayTimeFound = false;
  riseTimeStartFound = false;
  riseTimeEndFound = false;
  peakTimeFound = false;
  peakValue = 0;
  delayTime = 0;
  riseTimeStart = 0;
  riseTimeEnd = 0;
  peakTime = 0;
  settlingTime = 0;
}

void loop() {
  long currentHeight = getDistance();
  
  // Cek input tombol
  for (int i = 0; i < numFloors; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      targetFloor = i;
      targetHeight = floorHeights[targetFloor];
      initialHeight = currentHeight;  // Simpan ketinggian awal
      startTime = millis();
      resetResponseVariables();
      Serial.println("\nMoving to new target floor...");
      Serial.print("Initial Height: ");
      Serial.println(initialHeight);
      Serial.print("Target Height: ");
      Serial.println(targetHeight);
      break;
    }
  }

  if (targetFloor != -1) {
    double pidOutput = computePID(targetHeight, currentHeight);
    moveServo(pidOutput);
    
    if (!steadyStateReached) {
      unsigned long currentTime = millis() - startTime;
      double heightDifference = abs(targetHeight - initialHeight);
      double currentProgress = abs(currentHeight - initialHeight);
      
      // Delay Time (10% dari target)
      if (!delayTimeFound && currentProgress >= 0.1 * heightDifference) {
        delayTime = currentTime;
        delayTimeFound = true;
        Serial.print("Delay Time (Td): ");
        Serial.print(delayTime);
        Serial.println(" ms");
      }
      
      // Rise Time Start (10% dari target)
      if (!riseTimeStartFound && currentProgress >= 0.1 * heightDifference) {
        riseTimeStart = currentTime;
        riseTimeStartFound = true;
      }
      
      // Rise Time End (90% dari target)
      if (!riseTimeEndFound && currentProgress >= 0.9 * heightDifference) {
        riseTimeEnd = currentTime;
        riseTimeEndFound = true;
        Serial.print("Rise Time (Tr): ");
        Serial.print(riseTimeEnd - riseTimeStart);
        Serial.println(" ms");
      }
      
      // Peak Time
      if (currentHeight > peakValue) {
        peakValue = currentHeight;
        peakTime = currentTime;
        if (!peakTimeFound) {
          peakTimeFound = true;
          Serial.print("Peak Time (Tp): ");
          Serial.print(peakTime);
          Serial.println(" ms");
        }
      }
      
      // Print current status
      Serial.print("SetPoint:");
      Serial.print(targetHeight);
      Serial.print(",OutputSensor:");
      Serial.println(currentHeight);
      
      // Settling Time (±5% dari setpoint)
      if (abs(currentHeight - targetHeight) <= 0.05 * abs(targetHeight)) {
        settlingTime = currentTime;
        Serial.println("\nTarget position reached!");
        Serial.print("Settling Time (Ts): ");
        Serial.print(settlingTime);
        Serial.println(" ms");
        Serial.print("Final Height: ");
        Serial.println(currentHeight);
        steadyStateReached = true;
        moveServo(0);
        targetFloor = -1;
        integral = 0;
        previousError = 0;
      }
    }
  }
  
  delay(100);
}
