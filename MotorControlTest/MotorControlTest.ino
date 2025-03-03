/* Example sketch to control a stepper motor with TB6600 stepper motor driver 
  and Arduino without a library: continuous rotation. 
  More info: https://www.makerguides.com */

// Define stepper motor connections:
#define PRESSURE_MOTOR_DIR_PIN 2
#define PRESSURE_MOTOR_STEP_PIN 3

const float DEG_PER_STEP = 1.8;
const int STEPS_PER_REV = 200;
const int MAX_ROTATIONS = 7;  // of needle valve

int pressureMotorStepNumber = 0;  // 0 steps by default, assume valve is fully open

void setup() {
  // Declare pins as output:
  pinMode(PRESSURE_MOTOR_STEP_PIN, OUTPUT);
  pinMode(PRESSURE_MOTOR_DIR_PIN, OUTPUT);

  Serial.begin(9600);

  // Motor testing (press RESET on Arduino to re-run this)
  rotateMotorByAngle(360); // one rotations CW
  delayMicroseconds(10000000);
  rotateMotorByAngle(-360); // one rotation CCW
}

void loop() { }

/**
 * Moves the stepper motor to a specific step position.
 * Calls rotateMotorByStep() to execute the movement.
 * @param targetStep - The absolute step number to move to.
 */
void rotateMotorToStep(int targetStep) {
    int stepsToMove = targetStep - pressureMotorStepNumber;
    rotateMotorByStep(stepsToMove);
}

/**
 * Moves the stepper motor by a specific number of steps.
 * Updates the pressureMotorStepNumber to reflect the new position.
 * @param steps - The number of steps to move (positive for CW, negative for CCW).
 */
void rotateMotorByStep(int steps) {
    digitalWrite(PRESSURE_MOTOR_DIR_PIN, steps > 0 ? HIGH : LOW);  // Choose direction based on sign of "steps"

    for (int i = 0; i < abs(steps); i++) {
        digitalWrite(PRESSURE_MOTOR_STEP_PIN, HIGH);
        delayMicroseconds(350);
        digitalWrite(PRESSURE_MOTOR_STEP_PIN, LOW);
        delayMicroseconds(3000);
    }
    pressureMotorStepNumber += steps;
}

/**
 * Moves the stepper motor to a specific angle.
 * Converts the angle to step count and calls rotateMotorToStep().
 * @param targetAngle - The absolute angle to move to.
 */
void rotateMotorToAngle(float targetAngle) {
    int targetStep = round((targetAngle / 360.0) * STEPS_PER_REV);
    rotateMotorToStep(targetStep);
}

/**
 * Moves the stepper motor by a specific angle.
 * Converts the angle to step count and calls rotateMotorByStep().
 * @param degrees - The number of degrees to rotate (positive for CW, negative for CCW).
 */
void rotateMotorByAngle(float degrees) {
    int steps = round((degrees / 360.0) * STEPS_PER_REV);
    rotateMotorByStep(steps);
}
