const int enPin = 8;
const int stepXPin = 2; // X.STEP
const int dirXPin = 5;  // X.DIR

const int stepsPerRevolution = 200; // Update this baased on your stepper motor's specifications

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(enPin, OUTPUT);
  pinMode(stepXPin, OUTPUT);
  pinMode(dirXPin, OUTPUT);

  digitalWrite(enPin, LOW); // Enable the motor drivers
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();  // Read the incoming byte
    
    if (command == 'a') {
      int steps = stepsPerRevolution;  // 360 degrees = 1 full revolution
      rotateMotor(stepXPin, dirXPin, HIGH, steps);  // Rotate X-axis forward
    } 
    else if (command == 'b') {
      int steps = stepsPerRevolution;  // 360 degrees = 1 full revolution
      rotateMotor(stepXPin, dirXPin, LOW, steps);  // Rotate X-axis reverse (opposite direction)
    } 
    else {
      Serial.println("Invalid command");
    }
  }
}

void rotateMotor(int stepPin, int dirPin, int direction, int steps) {
  digitalWrite(dirPin, direction);  // Set rotation direction
  
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);  // Adjust pulse width for speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);  // Adjust pulse width for speed
  }
}