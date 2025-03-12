/*
  Cardiac Catheterization Testing Apparatus - Flow & Pressure Sensor Data Collection
  This script reads data from two flow sensors and three pressure sensors.
  The data is processed and transmitted over serial communication to an external system.
  - Flow sensors are connected via digital pins and use interrupts for accurate pulse counting.
  - Pressure sensors are connected via analog pins and use an interpolation function to convert raw ADC values to mmHg.
*/

// TODO: Check that AREF is connected to something
// TODO: Rename the flow/pressure sensor pins based on the specific locations (e.g. IVC/SVC)? Depends on how we implement modularity

#define PI 3.14159265358979323846

// Pin definitions
const unsigned char FLOW_SENSOR_PIN1 = 2;         // First flow sensor input pin
const unsigned char FLOW_SENSOR_PIN2 = 3;         // Second flow sensor input pin
const unsigned char PRESSURE_MOTOR_DIR_PIN = 4;   // Pressure valve control motor direction pin
const unsigned char PRESSURE_MOTOR_STEP_PIN = 5;  // Pressure valve control motor step pin
const unsigned char PUMP_PWM_PIN = 9;             // H-bridge enable/PWM pin
const unsigned char PRESSURE_SENSOR_PIN1 = A1;    // First pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN2 = A2;    // Second pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN3 = A3;    // Third pressure sensor input pin

// Other constants
const unsigned long PRESSURE_SENSOR_READ_INTERVAL = 50.;  // Interval for pressure readings (ms)
const unsigned long FLOW_SENSOR_READ_INTERVAL = 2000.;    // Interval for flow readings (ms); assumed to be many times larger than PRESSURE_SENSOR_READ_INTERVAL
const unsigned long PRESSURE_MOTOR_SPEED_NORMAL = 3500;   // delay for motor when operating at normal speed
const unsigned long EXPECTED_MATLAB_MESSAGE_LENGTH = 50;  // Length of message (# chars) expected from MATLAB for pump control

const float DEG_PER_STEP = 1.8;  // For pressure valve control motor
const int STEPS_PER_REV = 200;   // assuming # of microsteps is 1
const int MAX_ROTATIONS = 7;     // of needle valve
const int MAX_STEPS = STEPS_PER_REV * MAX_ROTATIONS;

const bool SIMULATE_VALUES = true;        // whether simulated values should be sent over serial (for testing without setup)
const bool SIMULATE_OSCILLATIONS = true;  // whether simulated values should oscillate sinusoidally over time

// Variables
volatile int flow_frequency1 = 0;  // First flow sensor pulse count
volatile int flow_frequency2 = 0;  // Second flow sensor pulse count
unsigned long currentTime;
unsigned long flowSensorTime = 0;
unsigned long pressureSensorTime = 0;

float flowRate1 = 0, flowRate2 = 0;
int pressureValueRaw1 = 0, pressureValueRaw2 = 0, pressureValueRaw3 = 0;
float pressureValue1 = 0, pressureValue2 = 0, pressureValue3 = 0;
char newFlowIndicator = 'N';  // Indicates whether a new flow reading is available

int pressureMotorStepNumber = 0;     // For controlling the pressure valve motor
int pressureMotorDelayMicroseconds;  // time off for pressure motor per stepper motor pulse;  3500 (normal) / 25000 (resetting)
int pumpAnalogWrite = 128;           // For controlling the pump
String inputString = "";             // a String to hold incoming data
bool stringComplete = false;         // whether the string is complete

/**
 * Interrupt service routines for the flow sensors.
 * Increment the pulse counts when pulses are detected.
 */
void flow1() {
  flow_frequency1++;
}
void flow2() {
  flow_frequency2++;
}

/**
 * Sets up the hardware configuration for sensors, serial communication, and interrupts.
 */
void setup() {
  // === PIN AND INTERRUPT SETUP ===
  pinMode(FLOW_SENSOR_PIN1, INPUT);
  pinMode(FLOW_SENSOR_PIN2, INPUT);
  pinMode(PRESSURE_MOTOR_DIR_PIN, OUTPUT);
  pinMode(PRESSURE_MOTOR_STEP_PIN, OUTPUT);
  digitalWrite(FLOW_SENSOR_PIN1, HIGH);
  digitalWrite(FLOW_SENSOR_PIN2, HIGH);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN1), flow1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN2), flow2, RISING);

  // === SERIAL COMMUNICATION SETUP ===
  Serial.begin(115200);
  inputString.reserve(200);  // reserve 200 bytes for the inputString

  // === MISCELLANEOUS SETUP ===
  analogReference(EXTERNAL);
  // sei();  // Enable global interrupts for flow sensor pulse counting

  currentTime = millis();
  flowSensorTime = currentTime;
  pressureSensorTime = currentTime;

  pressureMotorDelayMicroseconds = 10000;  // speed up for normal use

  analogWrite(PUMP_PWM_PIN, pumpAnalogWrite);
}

/**
 * Main loop function that reads sensor values at defined intervals
 * and sends the processed data over serial communication.
 */
void loop() {
  currentTime = millis();

  // Check whether a complete string has come in over Serial
  if (stringComplete) {
    processSerial(inputString);
    inputString = "";
    stringComplete = false;
  }

  if (SIMULATE_VALUES) {
    simulateDataValues(SIMULATE_OSCILLATIONS);
  } else {
    // Read pressure sensor values at defined intervals
    if (currentTime >= (pressureSensorTime + PRESSURE_SENSOR_READ_INTERVAL)) {
      pressureSensorTime = currentTime;
      pressureValueRaw1 = readPressureSensor(PRESSURE_SENSOR_PIN1);
      pressureValue1 = interpolatePressure(pressureValueRaw1, 1);
      pressureValueRaw2 = readPressureSensor(PRESSURE_SENSOR_PIN2);
      pressureValue2 = interpolatePressure(pressureValueRaw2, 2);
      pressureValueRaw3 = readPressureSensor(PRESSURE_SENSOR_PIN3);
      pressureValue3 = interpolatePressure(pressureValueRaw3, 3);
    }

    // Read flow sensor values and update new flow indicator
    if (currentTime >= (flowSensorTime + FLOW_SENSOR_READ_INTERVAL)) {
      flowSensorTime = currentTime;
      flowRate1 = readFlowSensor(flow_frequency1, 1);
      flowRate2 = readFlowSensor(flow_frequency2, 2);
      newFlowIndicator = 'Y';
    } else {
      newFlowIndicator = 'N';
    }
  }

  printSensorValues(newFlowIndicator, flowRate1, flowRate2,
                    pressureValueRaw1, pressureValue1, pressureValueRaw2, pressureValue2,
                    pressureValueRaw3, pressureValue3, pumpAnalogWrite);
}

// Function to simulate data values with sinusoidal variations
void simulateDataValues(bool oscillate) {
  unsigned long currentTime = millis();

  // Convert time to radians for a 2-second period
  float timeInSeconds = (currentTime % 2000) / 1000.0;  // Loops every 2 seconds
  float angle = timeInSeconds * PI;                     // Convert to radians

  // Sinusoidal variation
  float sinValue;
  if (oscillate) {
    sinValue = sin(angle);  // Ranges from -1 to 1
  } else {
    sinValue = 0;
  }

  // Assign values with oscillations
  flowRate1 = 3.0 + sinValue * 0.5;  // Oscillates between 2.5 and 3.5
  flowRate2 = 4.0 + sinValue * 0.5;  // Oscillates between 3.5 and 4.5

  pressureValueRaw1 = 255 + sinValue * 50;  // Oscillates around 255
  pressureValue1 = 20 + sinValue * 10;      // Oscillates between 10 and 30

  pressureValueRaw2 = 255 + sinValue * 50;
  pressureValue2 = 40 + sinValue * 10;  // Oscillates between 30 and 50

  pressureValueRaw3 = 255 + sinValue * 50;
  pressureValue3 = 60 + sinValue * 10;  // Oscillates between 50 and 70
}

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
    delayMicroseconds(pressureMotorDelayMicroseconds);
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

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

/*
 * Processes incoming serial commands to control relevant outputs. Currently supports just the motor for the pressure valve. 
 * Assumes Example string: "Pressure Motor Step Number: (0-1400), Flow Motor Voltage: (0-255)"
*/
void processSerial(String inputString) {
  // Parse each line
  int stepNumber;
  if (sscanf(inputString.c_str(), "MOT: %d, PMP: %d", &stepNumber, &pumpAnalogWrite) == 2) {
    rotateMotorToStep(stepNumber);
    analogWrite(PUMP_PWM_PIN, pumpAnalogWrite);
  }
}

/**
 * Reads the flow sensor pulse count, converts it to flow rate, and resets the pulse count.
 * @param flow_frequency - The pulse count of the flow sensor.
 * @return The calculated flow rate in L/min.
 */
float readFlowSensor(volatile int &flow_frequency, int sensorNum) {
  float flowRate = (flow_frequency / 9.68) * (1000.0 / FLOW_SENSOR_READ_INTERVAL);
  flow_frequency = 0;

  if (sensorNum == 1) {
    if (flowRate < 1) {
      flowRate = flowRate * 1.4214;  // Adjusted slope for continuity close to flowRate = 0
    } else {
      flowRate = flowRate * 0.8133 + 0.6081;
    }
  } else if (sensorNum == 2) {
    if (flowRate < 1) {
      flowRate = flowRate * 1.5773;  // Adjusted slope for continuity close to flowRate = 0
    } else {
      flowRate = flowRate * 0.7923 + 0.785;
    }
  }
  return flowRate;
}

/**
 * Reads the raw ADC value from the pressure sensor.
 * @param pin - The analog pin connected to the pressure sensor.
 * @return The raw ADC reading (0-1023).
 */
int readPressureSensor(unsigned char pin) {
  return analogRead(pin);
}

/**
 * Performs linear interpolation to convert raw ADC values to pressure in mmHg.
 * @param adc_value - The raw ADC reading from the pressure sensor.
 * @return The interpolated pressure value in mmHg.
 */
float interpolatePressure(float adc_value, int sensor) {
  const float *voltage_V;
  const float *pressure_mmHg;
  int size = 0;

  static const float voltage_V1[12] = { 54, 145, 214, 315, 404, 515, 620, 709, 852, 887, 935, 938 };
  static const float pressure_mmHg1[12] = { 0.7, 12, 20.2, 33.0, 44.2, 57.7, 70.5, 81.7, 94.5, 104.2, 115.5, 130.5 };

  static const float voltage_V2[14] = { 16, 98, 193, 258, 340, 423, 495, 596, 690, 763, 846, 915, 937, 939 };
  static const float pressure_mmHg2[14] = { 0, 10.5, 21.7, 29.2, 39.7, 49.5, 58.5, 70.5, 82.5, 90.7, 102, 111, 123.7, 135 };

  // Copied from sensor 2, should recalibrate it
  static const float voltage_V3[8] = { 117, 175, 363, 576, 700, 856, 960, 967 };
  static const float pressure_mmHg3[8] = { 2.2, 24.0, 48.7, 75.0, 91.5, 111.0, 128.2, 151.5 };

  // Select calibration curve
  if (sensor == 1) {
    voltage_V = voltage_V1;
    pressure_mmHg = pressure_mmHg1;
    size = 12;  // TODO: automatically determine size
  } else if (sensor == 2) {
    voltage_V = voltage_V2;
    pressure_mmHg = pressure_mmHg2;
    size = 14;
  } else {
    voltage_V = voltage_V3;
    pressure_mmHg = pressure_mmHg3;
    size = 8;
  }

  if (adc_value <= voltage_V[0]) {
    return pressure_mmHg[0] + (adc_value - voltage_V[0]) * (pressure_mmHg[1] - pressure_mmHg[0]) / (voltage_V[1] - voltage_V[0]);
  }
  if (adc_value >= voltage_V[size - 1]) {
    return pressure_mmHg[size - 2] + (adc_value - voltage_V[size - 2]) * (pressure_mmHg[size - 1] - pressure_mmHg[size - 2]) / (voltage_V[size - 1] - voltage_V[size - 2]);
  }

  for (int i = 0; i < size - 1; i++) {
    if (adc_value >= voltage_V[i] && adc_value <= voltage_V[i + 1]) {
      return pressure_mmHg[i] + (adc_value - voltage_V[i]) * (pressure_mmHg[i + 1] - pressure_mmHg[i]) / (voltage_V[i + 1] - voltage_V[i]);
    }
  }
  return -999;
}


/**
 * Sends flow and pressure sensor values over serial communication.
 * @param newFlowIndicator - 'Y' if new flow data is available, 'N' otherwise.
 * @param flowValue1 - Processed flow rate from the first flow sensor.
 * @param flowValue2 - Processed flow rate from the second flow sensor.
 * @param pressureValueRaw1 - Raw ADC value from the first pressure sensor.
 * @param pressureValue1 - Processed pressure from the first pressure sensor.
 * @param pressureValueRaw2 - Raw ADC value from the second pressure sensor.
 * @param pressureValue2 - Processed pressure from the second pressure sensor.
 * @param pressureValueRaw3 - Raw ADC value from the third pressure sensor.
 * @param pressureValue3 - Processed pressure from the third pressure sensor.
 * @param pumpAnalogWrite - Duty cycle value sent to pump via PWM (0-255)
 */
void printSensorValues(char newFlowIndicator, float flowValue1, float flowValue2,
                       int pressureValueRaw1, float pressureValue1, int pressureValueRaw2, float pressureValue2,
                       int pressureValueRaw3, float pressureValue3, int pumpAnalogWrite) {

  Serial.print("NF: ");
  Serial.print(newFlowIndicator);
  Serial.print(", F1: ");
  Serial.print(flowValue1);
  Serial.print(" L/min");
  Serial.print(", F2: ");
  Serial.print(flowValue2);
  Serial.print(" L/min");

  Serial.print(", P1R: ");
  Serial.print(pressureValueRaw1);
  Serial.print(", P1: ");
  Serial.print(pressureValue1);
  Serial.print(" mmHg");

  Serial.print(", P2R: ");
  Serial.print(pressureValueRaw2);
  Serial.print(", P2: ");
  Serial.print(pressureValue2);
  Serial.print(" mmHg");

  Serial.print(", P3R: ");
  Serial.print(pressureValueRaw3);
  Serial.print(", P3: ");
  Serial.print(pressureValue3);
  Serial.print(" mmHg");

  Serial.print(", Pump: ");
  Serial.print(pumpAnalogWrite);
  Serial.println("/255");
}
