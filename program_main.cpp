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

// Variabel untuk plotting
bool isMoving = false;
unsigned long lastPlotTime = 0;
const unsigned long PLOT_INTERVAL = 100; // Interval plotting (ms)

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

void printSerialPlotter(double setpoint, double currentHeight) {
  // // Format untuk Serial Plotter: "SetPoint:value,OutputSensor:value"
  // Serial.print("SetPoint:");
  // Serial.print(setpoint);
  // Serial.print(",OutputSensor:");
  // Serial.println(currentHeight);
  // write them all to console with tabs in between them
    Serial.print(setpoint);   // second variable is sin(t)
    Serial.print("\t");      // this last "\t" isn't required, but doesn't hurt
    Serial.println(currentHeight); // third varible is cos(t). make sure to finish with a println!
}

void printSerialMonitor(const char* message) {
  if (!steadyStateReached) {
    Serial.println(message);
  }
}

void loop() {
  long currentHeight = getDistance();
  unsigned long currentMillis = millis();
  
  // Cek input tombol
  for (int i = 0; i < numFloors; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      targetFloor = i;
      targetHeight = floorHeights[targetFloor];
      initialHeight = currentHeight;
      startTime = currentMillis;
      resetResponseVariables();
      isMoving = true;
      printSerialMonitor("\nMoving to new target floor...");
      printSerialMonitor(("Initial Height: " + String(initialHeight)).c_str());
      printSerialMonitor(("Target Height: " + String(targetHeight)).c_str());
      break;
    }
  }

  // Serial Plotter selalu berjalan
  if (currentMillis - lastPlotTime >= PLOT_INTERVAL) {
    printSerialPlotter(targetHeight, currentHeight);
    lastPlotTime = currentMillis;
  }

  if (targetFloor != -1) {
    double pidOutput = computePID(targetHeight, currentHeight);
    moveServo(pidOutput);
    
    if (!steadyStateReached) {
      unsigned long currentTime = currentMillis - startTime;
      double heightDifference = abs(targetHeight - initialHeight);
      double currentProgress = abs(currentHeight - initialHeight);
      
      // Delay Time (10% dari target)
      if (!delayTimeFound && currentProgress >= 0.1 * heightDifference) {
        delayTime = currentTime;
        delayTimeFound = true;
        printSerialMonitor(("Delay Time (Td): " + String(delayTime) + " ms").c_str());
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
        printSerialMonitor(("Rise Time (Tr): " + String(riseTimeEnd - riseTimeStart) + " ms").c_str());
      }
      
      // Peak Time
      if (currentHeight > peakValue) {
        peakValue = currentHeight;
        peakTime = currentTime;
        if (!peakTimeFound) {
          peakTimeFound = true;
          printSerialMonitor(("Peak Time (Tp): " + String(peakTime) + " ms").c_str());
        }
      }
      
      // Settling Time (±5% dari setpoint)
      if (abs(currentHeight - targetHeight) <= 0.05 * abs(targetHeight)) {
        settlingTime = currentTime;
        printSerialMonitor("\nTarget position reached!");
        printSerialMonitor(("Settling Time (Ts): " + String(settlingTime) + " ms").c_str());
        printSerialMonitor(("Final Height: " + String(currentHeight)).c_str());
        steadyStateReached = true;
        moveServo(0);
        targetFloor = -1;
        integral = 0;
        previousError = 0;
        isMoving = false;
      }
    }
  }
  
  delay(50); // Reduced delay for smoother plotting
}
