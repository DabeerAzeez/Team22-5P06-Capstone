/*
  Cardiac Catheterization Testing Apparatus (CCTA) - Flow & Pressure Sensor Data Collection
  This script communicates bidirectionally with the CCTA MATLAB GUI 
  
  In terms of data transmitted:
  - There are two flow sensors and three pressure sensors
  - This script processes raw values and transmits calibrated values over serial communication to the CCTA GUI
  - Flow sensors are connected via digital pins and use interrupts for accurate pulse counting.
  - Pressure sensors are connected via analog pins and use an interpolation function to convert raw ADC values to mmHg.

  In terms of data received:
  - A serial interrupt event triggers on receipt of commands from the CCTA GUI in order to set the PWM value for the pump
*/

// TODO: Investigate whether newFlowIndicator is working as expected

// ====================================================
// HARDWARE CONFIGURATION
// ====================================================

// --- Pin Definitions ---
const unsigned char FLOW_SENSOR_PIN1 = 2;       // Flow sensor 1 input pin
const unsigned char FLOW_SENSOR_PIN2 = 3;       // Flow sensor 2 input pin
const unsigned char PUMP_PWM_PIN = 9;           // H-bridge enable/PWM pin
const unsigned char PRESSURE_SENSOR_PIN1 = A1;  // Pressure sensor 1 input pin
const unsigned char PRESSURE_SENSOR_PIN2 = A2;  // Pressure sensor 2 input pin
const unsigned char PRESSURE_SENSOR_PIN3 = A3;  // Pressure sensor 3 input pin

// --- System Constants ---
const unsigned long PRESSURE_SENSOR_READ_INTERVAL = 50;   // Interval for pressure readings (ms)
const unsigned long FLOW_SENSOR_READ_INTERVAL = 2000;     // Interval for flow readings (ms)
const unsigned long EXPECTED_MATLAB_MESSAGE_LENGTH = 50;  // Length of expected message from MATLAB
const unsigned long MAX_DUTY_CYCLE = 255;                 // 8-bit PWM resolution
const bool SIMULATE_VALUES = false;                       // Toggle value simulation
const bool SIMULATE_OSCILLATIONS = true;                  // Toggle oscillating simulated values


// ====================================================
// SYSTEM STATE VARIABLES
// ====================================================

// --- Sensor Readings ---
volatile int flow1_pulses = 0;
volatile int flow2_pulses = 0;
float flowRate1 = 0, flowRate2 = 0;

int pressureValueRaw1 = 0, pressureValueRaw2 = 0, pressureValueRaw3 = 0;
float pressureValue1 = 0, pressureValue2 = 0, pressureValue3 = 0;
char newFlowIndicator = 'N';  // Whether new flow data is available

// --- Timing Variables ---
unsigned long currentTime = 0;
unsigned long flowSensorTime = 0;
unsigned long pressureSensorTime = 0;

// --- Pump & Serial Control ---
int pumpDutyCycle = 0;
String inputString = "";
bool stringComplete = false;
String pumpControlMode = "Manual";  // Default mode
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// --- Pulsatile Mode Settings ---
int pulsatileBPM = 60;
int pulsatileAmplitude = 0.8 * MAX_DUTY_CYCLE;


// ====================================================
// PID CONTROL CONFIGURATION
// ====================================================

// --- PID Parameters & State ---
String pidSetpointId = "None";
float pidSetpoint = 0;
float Kp = 0.0, Ki = 0.0, Kd = 0.0;

char pidSetpointIdBuffer[20];              // temporary buffer for sscanf (e.g. "Pressure1")
long pidSetpointInt, KpInt, KiInt, KdInt;  // scaled integer values received

float pidError = 0.0;
float pidIntegral = 0.0;
float pidPrevError = 0.0;
float pidDerivative = 0.0;

// --- Rolling Error History ---
const int PID_HISTORY_LEN = 40;
float rollingErrors[PID_HISTORY_LEN] = { 0 };
int rollingIndex = 0;

// --- PID Timing ---
unsigned long lastPIDTime = 0;
long pidIntervalMS = PRESSURE_SENSOR_READ_INTERVAL;


/**
 * Interrupt service routines for the flow sensors.
 * Increments pulse counts for each sensor when pulses are detected.
 */
void flow1() {
  flow1_pulses++;
}
void flow2() {
  flow2_pulses++;
}

/**
 * Sets up the hardware configuration for sensors, serial communication, and interrupts.
 */
void setup() {
  // === PIN AND INTERRUPT SETUP ===
  pinMode(FLOW_SENSOR_PIN1, INPUT);
  pinMode(FLOW_SENSOR_PIN2, INPUT);
  digitalWrite(FLOW_SENSOR_PIN1, HIGH);
  digitalWrite(FLOW_SENSOR_PIN2, HIGH);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN1), flow1, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN2), flow2, RISING);

  // === SERIAL COMMUNICATION SETUP ===
  Serial.begin(115200);
  inputString.reserve(200);  // reserve 200 bytes for the inputString

  // === MISCELLANEOUS SETUP ===
  analogReference(EXTERNAL);
  // sei();  // TODO: Check if this is necessary -- Enable global interrupts for flow sensor pulse counting

  currentTime = millis();
  flowSensorTime = currentTime;
  pressureSensorTime = currentTime;

  analogWrite(PUMP_PWM_PIN, pumpDutyCycle);
}

/**
 * Main loop function that reads sensor values at defined intervals
 * and sends the processed data over serial communication.
 */
void loop() {
  currentTime = millis();

  if (SIMULATE_VALUES) {
    simulateDataValues(SIMULATE_OSCILLATIONS);
  } else {
    readSensorValues();
  }

  if (pumpControlMode == "Auto") {
    // Update PID Control if it's been long enough
    if (millis() - lastPIDTime >= pidIntervalMS) {
      lastPIDTime = millis();
      runPIDControl();
    }
  } else if (pumpControlMode == "Pulsatile") {
    setPulsatileFlow();
  }

  sendSystemData();
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

  pressureValueRaw2 = 255 + sinValue * 50;  // Oscillates around 255
  pressureValue2 = 40 + sinValue * 10;      // Oscillates between 30 and 50

  pressureValueRaw3 = 255 + sinValue * 50;  // Oscillates around 255
  pressureValue3 = 60 + sinValue * 10;      // Oscillates between 50 and 70
}

// Reads pressure and flow sensor values at defined intervals
void readSensorValues() {
  // Read pressure sensor values if enough time has passed
  if (currentTime >= (pressureSensorTime + PRESSURE_SENSOR_READ_INTERVAL)) {
    pressureSensorTime = currentTime;
    pressureValueRaw1 = readPressureSensor(PRESSURE_SENSOR_PIN1);
    pressureValue1 = interpolatePressure(pressureValueRaw1, 1);
    pressureValueRaw2 = readPressureSensor(PRESSURE_SENSOR_PIN2);
    pressureValue2 = interpolatePressure(pressureValueRaw2, 2);
    pressureValueRaw3 = readPressureSensor(PRESSURE_SENSOR_PIN3);
    pressureValue3 = interpolatePressure(pressureValueRaw3, 3);
  }

  // Read flow sensor values if enough time has passed
  if (currentTime >= (flowSensorTime + FLOW_SENSOR_READ_INTERVAL)) {
    flowSensorTime = currentTime;
    flowRate1 = readFlowSensor(flow1_pulses, 1);
    flowRate2 = readFlowSensor(flow2_pulses, 2);
    newFlowIndicator = 'Y';
  } else {
    newFlowIndicator = 'N';
  }
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
    // if the incoming character is a newline, process the full line then reset the string
    if (inChar == '\n') {
      processSerial(inputString);
      inputString = "";
    }
    // TODO: Add error in case new line hasn't been received in a while, or if the input string is absurdly long
  }
}

/*
 * Processes serial command strings sent from GUI
*/
void processSerial(String inputString) {

  // Extract pump duty cycle and write to analog pin
  if (sscanf(inputString.c_str(), "PMP: %d", &pumpDutyCycle) == 1) {
    Serial.print("RECEIVED pump duty cycle: ");
    Serial.print(pumpDutyCycle);
    Serial.println("/255");

    pumpControlMode = "Manual";

    analogWrite(PUMP_PWM_PIN, pumpDutyCycle);
  } else if (sscanf(inputString.c_str(), "PID: %[^,], %ld, %ld, %ld, %ld",
                    pidSetpointIdBuffer, &pidSetpointInt, &KpInt, &KiInt, &KdInt)
             == 5) {

    // Convert char buffer to String
    pidSetpointId = String(pidSetpointIdBuffer);

    // Check the setpoint identifier and act accordingly
    if (pidSetpointId.indexOf("Flow") >= 0) {
      pidIntervalMS = FLOW_SENSOR_READ_INTERVAL;
    } else if (pidSetpointId.indexOf("Pressure") >= 0) {
      pidIntervalMS = PRESSURE_SENSOR_READ_INTERVAL;
    } else {
      Serial.println("Error: Unrecognized setpoint ID. Reverting to MANUAL mode.");
      pumpControlMode = "Manual";
      return;
    }

    // Convert ints back to float (assumes scaling by 10000)
    pidSetpoint = pidSetpointInt / 10000.0;
    Kp = KpInt / 10000.0;
    Ki = KiInt / 10000.0;
    Kd = KdInt / 10000.0;

    Serial.print("RECEIVED PID SETUP -> Target: ");
    Serial.print(pidSetpointId);
    Serial.print(" | Setpoint: ");
    Serial.print(pidSetpoint);
    Serial.print(" | Kp: ");
    Serial.print(Kp);
    Serial.print(" | Ki: ");
    Serial.print(Ki);
    Serial.print(" | Kd: ");
    Serial.println(Kd);

    // Reset PID error calculations
    pidIntegral = 0.0;
    pidPrevError = 0.0;
    rollingErrors[PID_HISTORY_LEN] = { 0 };
    rollingIndex = 0;

    // Set pump control mode to auto so main loop can handle it from there
    pumpControlMode = "Auto";

  } else if (sscanf(inputString.c_str(), "PULSE: %d, %d", &pulsatileBPM, &pulsatileAmplitude) == 2) {
    Serial.print("RECEIVED pulsatile flow settings (BPM: ");
    Serial.print(pulsatileBPM);
    Serial.print(", amplitude: ");
    Serial.print(pulsatileAmplitude);
    Serial.println(")");

    pumpControlMode = "Pulsatile";
  } else {
    Serial.print("ERROR: Unrecognized message: ");
    Serial.println(inputString);
  }

  inputString = "";  // Clear input string
}

/**
 * Reads the flow sensor pulse count, converts it to flow rate, and resets the pulse count.
 * @param flow_pulses - The pulse count of the flow sensor.
 * @param sensorNum - sensor number
 * @return The calculated flow rate in L/min.
 */
float readFlowSensor(volatile int &flow_pulses, int sensorNum) {
  float flowRate = (flow_pulses / 9.68) * (1000.0 / FLOW_SENSOR_READ_INTERVAL);
  flow_pulses = 0;

  // Adjust flow rate using calibrated piecewise linear functions
  if (sensorNum == 1) {
    if (flowRate < 1) {
      flowRate = flowRate * 1.5773;  // Adjusted slope for continuity close to flowRate = 0
    } else {
      flowRate = flowRate * 0.7923 + 0.785;
    }
  } else if (sensorNum == 2) {
    if (flowRate < 1) {
      flowRate = flowRate * 1.4214;  // Adjusted slope for continuity close to flowRate = 0
    } else {
      flowRate = flowRate * 0.8133 + 0.6081;
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
 * @param sensorNum - sensor number
 * @return The interpolated pressure value in mmHg.
 */
float interpolatePressure(float adc_value, int sensorNum) {
  const float *adc_values;
  const float *pressure_mmHg;
  int size;

  // Pressure calibration curves (adc value vs. pressure in mmHg)
  static const float adc_values1[17] = { 62, 155, 232, 340, 433, 530, 640, 719, 810, 887, 945, 965, 970, 974, 979, 985, 990 };
  static const float pressure_mmHg1[17] = { 1.5, 12.7, 21.7, 34.5, 44.0, 57.0, 69.7, 78.7, 90.0, 108.0, 128.2, 150.7, 180.7, 207.0, 234.7, 270.0, 297.7 };

  static const float adc_values2[19] = { 20, 110, 177, 252, 321, 427, 541, 638, 734, 845, 944, 955, 958, 961, 966, 971, 976, 979, 983 };
  static const float pressure_mmHg2[19] = { 0.0, 10.5, 18.7, 27.7, 36.0, 48.0, 61.5, 73.5, 84.7, 98.2, 111.7, 126.7, 141.0, 165.0, 193.5, 223.5, 253.5, 273.0, 294.7 };

  static const float adc_values3[18] = { 9, 61, 131, 227, 317, 434, 563, 683, 786, 895, 963, 965, 969, 974, 979, 983, 987, 991 };
  static const float pressure_mmHg3[18] = { 0.7, 11.2, 19.5, 30.7, 41.2, 55.5, 70.5, 84.7, 97.5, 111, 128.2, 144.7, 170.2, 198.7, 224.2, 248.2, 271.5, 296.2 };

  // Select calibration curve
  if (sensorNum == 1) {
    adc_values = adc_values1;
    pressure_mmHg = pressure_mmHg1;
    size = 17;
  } else if (sensorNum == 2) {
    adc_values = adc_values2;
    pressure_mmHg = pressure_mmHg2;
    size = 19;
  } else if (sensorNum == 3) {
    adc_values = adc_values3;
    pressure_mmHg = pressure_mmHg3;
    size = 18;
  } else {
    // use sensor 1 values
    adc_values = adc_values1;
    pressure_mmHg = pressure_mmHg1;
    size = 17;
  }

  // int size = sizeof(adc_values) / sizeof(adc_values[0]);

  // Extrapolate below curve (using first two data points)
  if (adc_value <= adc_values[0]) {
    return pressure_mmHg[0] + (adc_value - adc_values[0]) * (pressure_mmHg[1] - pressure_mmHg[0]) / (adc_values[1] - adc_values[0]);
  }

  // Extrapolate above curve (using last two data points)
  if (adc_value >= adc_values[size - 1]) {
    return pressure_mmHg[size - 2] + (adc_value - adc_values[size - 2]) * (pressure_mmHg[size - 1] - pressure_mmHg[size - 2]) / (adc_values[size - 1] - adc_values[size - 2]);
  }

  // Interpolate within curve
  for (int i = 0; i < size - 1; i++) {
    if (adc_value >= adc_values[i] && adc_value <= adc_values[i + 1]) {
      return pressure_mmHg[i] + (adc_value - adc_values[i]) * (pressure_mmHg[i + 1] - pressure_mmHg[i]) / (adc_values[i + 1] - adc_values[i]);
    }
  }
  return -999;  // Error value
}

/**
 * runPIDControl
 * 
 * Executes one iteration of the PID control algorithm. It reads the current
 * value from the specified sensor, computes the error, updates integral and
 * derivative terms, calculates the PID output, and applies it to the pump
 * using analogWrite. The output is constrained within the PWM range.
 */
void runPIDControl() {
  // Prevent integral from accumulating infinitely by removing oldest error term's contribution
  int oldestIndex = (rollingIndex + 1) % PID_HISTORY_LEN;
  float oldestError = rollingErrors[oldestIndex];
  pidIntegral -= oldestError * (pidIntervalMS / 1000.0);

  // Get new error value and calculate PID output
  float currentValue = getSensorValue(pidSetpointId);
  pidError = pidSetpoint - currentValue;
  rollingErrors[rollingIndex] = pidError;
  rollingIndex = (rollingIndex + 1) % PID_HISTORY_LEN;

  pidIntegral += pidError * (pidIntervalMS / 1000.0);
  pidDerivative = (pidError - pidPrevError) / (pidIntervalMS / 1000.0);
  pidPrevError = pidError;

  float pidOutput = Kp * pidError + Ki * pidIntegral + Kd * pidDerivative;
  pumpDutyCycle = constrain(pumpDutyCycle + (int)pidOutput, PWM_MIN, PWM_MAX);
  analogWrite(PUMP_PWM_PIN, pumpDutyCycle);

  // Serial print commands (for debugging)
  // Serial.print(">>> PID CONTROL ");
  // Serial.print("| Setpoint: ");
  // Serial.print(pidSetpoint);
  // Serial.print(" | Current: ");
  // Serial.print(currentValue);
  // Serial.print(" | Error: ");
  // Serial.print(pidError);
  // Serial.print(" | Integral: ");
  // Serial.print(pidIntegral);
  // Serial.print(" | Derivative: ");
  // Serial.print(pidDerivative);
  // Serial.print(" | Output: ");
  // Serial.print(pidOutput);
  // Serial.println(" <<<");
}

/**
 * getSensorValue
 *
 * Returns the current reading of a sensor specified by its ID string.
 * The ID must match one of the predefined sensor names (e.g., "Pressure1").
 * If the ID does not match, 0 is returned by default.
 *
 * @param id The sensor name as a String
 * @return The latest value of the requested sensor
 */
float getSensorValue(String id) {
  if (id == "Pressure1") return pressureValue1;
  else if (id == "Pressure2") return pressureValue2;
  else if (id == "Pressure3") return pressureValue3;
  else if (id == "Flow1") return flowRate1;
  else if (id == "Flow2") return flowRate2;
  return 0;
}

/*
* Enables pulsatile flow (i.e. sinusoidal oscillations in pump power) according to system BPM and amplitude for pulsatile flow
*/
void setPulsatileFlow() {
  float beatsPerSecond = pulsatileBPM / 60.0;
  float periodMs = 1000.0 / beatsPerSecond;  // period of one beat in ms

  float timeInSeconds = (currentTime % (unsigned long)periodMs) / 1000.0;
  float angle = timeInSeconds * 2 * PI * beatsPerSecond;

  // Generate sine wave from 0 to pulsatileAmplitude
  float rawSine = 0.5 * (1 + sin(angle));
  pumpDutyCycle = (int)round(pulsatileAmplitude * rawSine);

  analogWrite(PUMP_PWM_PIN, pumpDutyCycle);
}

/*
 * Sends system data over serial, including flow readings, raw/converted pressure readings, 
 * pump duty cycle, and pulsatile flow parameters.
 * 
 * Sample message: 
 * "NF: N, F1: 0.00, F2: 0.00, P1R: 783, P1: 86.65, P2R: 9, P2: -1.28, P3R: 449, P3: 57.24 | PMP: 200 | PULSE: N, BPM: 30, AMP: 200"
 */
void sendSystemData() {
  // Flow data
  Serial.print("NF: ");
  Serial.print(newFlowIndicator);
  Serial.print(", F1: ");
  Serial.print(flowRate1);
  Serial.print(", F2: ");
  Serial.print(flowRate2);

  // Pressure sensor 1
  Serial.print(", P1R: ");
  Serial.print(pressureValueRaw1);
  Serial.print(", P1: ");
  Serial.print(pressureValue1);

  // Pressure sensor 2
  Serial.print(", P2R: ");
  Serial.print(pressureValueRaw2);
  Serial.print(", P2: ");
  Serial.print(pressureValue2);

  // Pressure sensor 3
  Serial.print(", P3R: ");
  Serial.print(pressureValueRaw3);
  Serial.print(", P3: ");
  Serial.print(pressureValue3);

  // Control mode
  Serial.print(" | MODE: ");
  Serial.print(pumpControlMode);

  // Manual pump power
  Serial.print(" | PMP: ");
  Serial.print(pumpDutyCycle);

  // PID settings
  Serial.print(" | PID_ID: ");
  Serial.print(pidSetpointId);
  Serial.print(", SP: ");
  Serial.print(pidSetpoint);
  Serial.print(", Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.print(Kd);

  // Pulsatile flow parameters
  Serial.print(" | BPM: ");
  Serial.print(pulsatileBPM);
  Serial.print(", AMP: ");
  Serial.println(pulsatileAmplitude);
}
