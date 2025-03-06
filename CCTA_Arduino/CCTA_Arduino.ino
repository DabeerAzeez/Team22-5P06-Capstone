/*
  Cardiac Catheterization Testing Apparatus - Flow & Pressure Sensor Data Collection
  This script reads data from two flow sensors and three pressure sensors.
  The data is processed and transmitted over serial communication to an external system.
  - Flow sensors are connected via digital pins and use interrupts for accurate pulse counting.
  - Pressure sensors are connected via analog pins and use an interpolation function to convert raw ADC values to mmHg.
*/

// TODO: Check that AREF is connected to something
// TODO: Rename the flow/pressure sensor pins based on the specific locations (e.g. IVC/SVC)? Depends on how we implement modularity

// Pin definitions
const unsigned char FLOW_SENSOR_PIN1 = 1;         // First flow sensor input pin
const unsigned char FLOW_SENSOR_PIN2 = 2;         // Second flow sensor input pin
const unsigned char PRESSURE_MOTOR_DIR_PIN = 4;   // Pressure valve control motor direction pin
const unsigned char PRESSURE_MOTOR_STEP_PIN = 5;  // Pressure valve control motor step pin
const unsigned char PUMP_IN_PIN1 = 8;  // H-bridge input 1
const unsigned char PUMP_IN_PIN2 = 7;  // H-bridge input 2
const unsigned char PUMP_ENA_PIN = 9;  // H-bridge enable/PWM pin
const unsigned char PRESSURE_SENSOR_PIN1 = A1;    // First pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN2 = A2;    // Second pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN3 = A3;    // Third pressure sensor input pin

// Other constants
const unsigned long PRESSURE_SENSOR_READ_INTERVAL = 50.;  // Interval for pressure readings (ms)
const unsigned long FLOW_SENSOR_READ_INTERVAL = 2000.;    // Interval for flow readings (ms); assumed to be many times larger than PRESSURE_SENSOR_READ_INTERVAL
const unsigned long PRESSURE_MOTOR_SPEED_NORMAL = 3500;  // delay for motor when operating at normal speed

const float DEG_PER_STEP = 1.8;  // For pressure valve control motor
const int STEPS_PER_REV = 200;   // assuming # of microsteps is 1
const int MAX_ROTATIONS = 7;     // of needle valve
const int MAX_STEPS = STEPS_PER_REV * MAX_ROTATIONS;

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

int pressureMotorStepNumber = 0;  // For controlling the pressure valve motor
int pressureMotorDelayMicroseconds;  // time off for pressure motor per stepper motor pulse;  3500 (normal) / 25000 (resetting)
int pumpAnalogWrite = 0;              // For controlling the pump

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
  Serial.begin(9600);

  // === MISCELLANEOUS SETUP ===
  analogReference(EXTERNAL);
  // sei();  // Enable global interrupts for flow sensor pulse counting

  currentTime = millis();
  flowSensorTime = currentTime;
  pressureSensorTime = currentTime;

  // pressureMotorDelayMicroseconds = 10000;     // slow down while resetting so motor isn't damaged when valve locks 
  // rotateMotorByStep(-MAX_STEPS);              // reset valve position to 100% closed
  // rotateMotorByStep(round(MAX_STEPS * 0.7));  // Move to 30% closed (tough to spin valve at values lower than this)
  pressureMotorDelayMicroseconds = 10000;      // speed up for normal use
}

/**
 * Main loop function that reads sensor values at defined intervals
 * and sends the processed data over serial communication.
 */
void loop() {
  currentTime = millis();

  // Check serial comms for control messages
  if (Serial.available()) {
    processSerial();
  }

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
    flowRate1 = readFlowSensor(flow_frequency1);
    flowRate2 = readFlowSensor(flow_frequency2);
    newFlowIndicator = 'Y';
  } else {
    newFlowIndicator = 'N';
  }

  printSensorValues(newFlowIndicator, flow_frequency1, flowRate1, flow_frequency2, flowRate2,
                    pressureValueRaw1, pressureValue1, pressureValueRaw2, pressureValue2,
                    pressureValueRaw3, pressureValue3);
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
 * Processes incoming serial commands to control relevant outputs. Currently supports just the motor for the pressure valve. 
 * Assumes Example string: "Pressure Motor Step Number: (0-1400), Flow Motor Voltage: (0-255)"
*/
void processSerial() {
  String command = Serial.readStringUntil("\n");

  // Manually split the string by newline since readBytesUntil doesn't seem to work properly
  int newlineIndex = command.indexOf('\n');
  while (newlineIndex != -1) {
    // Extract the line up to the newline
    String line = command.substring(0, newlineIndex);
    line.trim();

    // Parse each line
    int stepNumber;
    if (sscanf(line.c_str(), "Pressure Motor Step Number: %d, Pump Duty Cycle: %d", &stepNumber, &pumpAnalogWrite) == 2) {
      rotateMotorToStep(stepNumber);
      analogWrite(PUMP_ENA_PIN,pumpAnalogWrite);
    }

    // Remove this line (and the newline character) from 'command'
    command.remove(0, newlineIndex + 1);

    // Check if there's another newline in the remaining string
    newlineIndex = command.indexOf('\n');
  }
}

/**
 * Reads the flow sensor pulse count, converts it to flow rate, and resets the pulse count.
 * @param flow_frequency - The pulse count of the flow sensor.
 * @return The calculated flow rate in L/min.
 */
float readFlowSensor(volatile int &flow_frequency) {
  float flowRate = (flow_frequency / 9.68) * (1000.0 / FLOW_SENSOR_READ_INTERVAL);
  flow_frequency = 0;
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

  static const float voltage_V1[11] = { 74, 250, 344, 479, 591, 704, 777, 871, 961, 969, 971 };
  static const float pressure_mmHg1[11] = { 0.7, 21.7, 33.0, 49.5, 63.0, 76.5, 85.5, 96.7, 111.0, 138.7, 149.2 };

  static const float voltage_V2[8] = { 117, 175, 363, 576, 700, 856, 960, 967 };
  static const float pressure_mmHg2[8] = { 2.2, 24.0, 48.7, 75.0, 91.5, 111.0, 128.2, 151.5 };

  // Copied from sensor 2, should recalibrate it
  static const float voltage_V3[8] = { 117, 175, 363, 576, 700, 856, 960, 967 };
  static const float pressure_mmHg3[8] = { 2.2, 24.0, 48.7, 75.0, 91.5, 111.0, 128.2, 151.5 };

  if (sensor == 1) {
    voltage_V = voltage_V1;
    pressure_mmHg = pressure_mmHg1;
    size = 11;
  } else if (sensor == 2) {
    voltage_V = voltage_V2;
    pressure_mmHg = pressure_mmHg2;
    size = 8;
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
 * @param flowValueRaw1 - Raw pulse count from the first flow sensor.
 * @param flowValue1 - Processed flow rate from the first flow sensor.
 * @param flowValueRaw2 - Raw pulse count from the second flow sensor.
 * @param flowValue2 - Processed flow rate from the second flow sensor.
 * @param pressureValueRaw1 - Raw ADC value from the first pressure sensor.
 * @param pressureValue1 - Processed pressure from the first pressure sensor.
 * @param pressureValueRaw2 - Raw ADC value from the second pressure sensor.
 * @param pressureValue2 - Processed pressure from the second pressure sensor.
 * @param pressureValueRaw3 - Raw ADC value from the third pressure sensor.
 * @param pressureValue3 - Processed pressure from the third pressure sensor.
 */
void printSensorValues(char newFlowIndicator, int flowValueRaw1, float flowValue1, int flowValueRaw2, float flowValue2,
                       int pressureValueRaw1, float pressureValue1, int pressureValueRaw2, float pressureValue2,
                       int pressureValueRaw3, float pressureValue3) {
  Serial.print("New Flow?: ");
  Serial.print(newFlowIndicator);
  Serial.print(", Flow Raw 1: ");
  Serial.print(flowValueRaw1);
  Serial.print(", Flow 1: ");
  Serial.print(flowValue1);
  Serial.print(" L/min");
  Serial.print(", Flow Raw 2: ");
  Serial.print(flowValueRaw2);
  Serial.print(", Flow 2: ");
  Serial.print(flowValue2);
  Serial.print(" L/min");
  Serial.print(", Pressure Raw 1: ");
  Serial.print(pressureValueRaw1);
  Serial.print(", Pressure 1: ");
  Serial.print(pressureValue1);
  Serial.print(" mmHg");
  Serial.print(", Pressure Raw 2: ");
  Serial.print(pressureValueRaw2);
  Serial.print(", Pressure 2: ");
  Serial.print(pressureValue2);
  Serial.print(" mmHg");
  Serial.print(", Pressure Raw 3: ");
  Serial.print(pressureValueRaw3);
  Serial.print(", Pressure 3: ");
  Serial.print(pressureValue3);
  Serial.println(" mmHg");
}
