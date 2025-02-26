/*
  Cardiac Catheterization Testing Apparatus - Flow & Pressure Sensor Data Collection
  This script reads data from two flow sensors and three pressure sensors.
  The data is processed and transmitted over serial communication to an external system.
  - Flow sensors are connected via digital pins and use interrupts for accurate pulse counting.
  - Pressure sensors are connected via analog pins and use an interpolation function to convert raw ADC values to mmHg.
*/

// TODO: Check that AREF is connected to something
// TODO: Rename the flow/pressure sensor pins based on the specific locations (e.g. IVC/SVC)? Depends on how we implement modularity

const unsigned char FLOW_SENSOR_PIN1 = 2;  // First flow sensor input pin
const unsigned char FLOW_SENSOR_PIN2 = 3;  // Second flow sensor input pin
const unsigned char PRESSURE_SENSOR_PIN1 = A0;  // First pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN2 = A1;  // Second pressure sensor input pin
const unsigned char PRESSURE_SENSOR_PIN3 = A2;  // Third pressure sensor input pin

const unsigned long PRESSURE_SENSOR_READ_INTERVAL = 50.;  // Interval for pressure readings (ms)
const unsigned long FLOW_SENSOR_READ_INTERVAL = 2000.;  // Interval for flow readings (ms); assumed to be many times larger than PRESSURE_SENSOR_READ_INTERVAL

volatile int flow_frequency1 = 0;  // First flow sensor pulse count
volatile int flow_frequency2 = 0;  // Second flow sensor pulse count
unsigned long currentTime;
unsigned long flowSensorTime = 0;
unsigned long pressureSensorTime = 0;

float flowRate1 = 0, flowRate2 = 0;
int pressureValueRaw1 = 0, pressureValueRaw2 = 0, pressureValueRaw3 = 0;
float pressureValue1 = 0, pressureValue2 = 0, pressureValue3 = 0;
char newFlowIndicator = 'N';  // Indicates whether a new flow reading is available

/**
 * Interrupt service routines for the flow sensors.
 * Increment the pulse counts when pulses are detected.
 */
void flow1() { flow_frequency1++; }
void flow2() { flow_frequency2++; }

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
    Serial.begin(9600);

    // === MISCELLANEOUS SETUP ===
    analogReference(EXTERNAL);
    // sei();  // Enable global interrupts for flow sensor pulse counting

    currentTime = millis();
    flowSensorTime = currentTime;
    pressureSensorTime = currentTime;

    Serial.println("RESET");
}

/**
 * Main loop function that reads sensor values at defined intervals
 * and sends the processed data over serial communication.
 */
void loop() {
    currentTime = millis();

    // Read pressure sensor values at defined intervals
    if (currentTime >= (pressureSensorTime + PRESSURE_SENSOR_READ_INTERVAL)) {
        pressureSensorTime = currentTime;
        pressureValueRaw1 = readPressureSensor(PRESSURE_SENSOR_PIN1);
        pressureValue1 = interpolatePressure(pressureValueRaw1);
        pressureValueRaw2 = readPressureSensor(PRESSURE_SENSOR_PIN2);
        pressureValue2 = interpolatePressure(pressureValueRaw2);
        pressureValueRaw3 = readPressureSensor(PRESSURE_SENSOR_PIN3);
        pressureValue3 = interpolatePressure(pressureValueRaw3);
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
float interpolatePressure(float adc_value) {
    const float voltage_V[] = {94, 336, 503, 700, 880, 996, 1020};
    const float pressure_mmHg[] = {2.2, 30.0, 49.5, 72.7, 93.7, 114.0, 249.7};
    const int size = sizeof(voltage_V) / sizeof(voltage_V[0]);

    if (adc_value <= voltage_V[0]) {
        return pressure_mmHg[0] + (adc_value - voltage_V[0]) * (pressure_mmHg[1] - pressure_mmHg[0]) / (voltage_V[1] - voltage_V[0]);
    }
    if (adc_value >= voltage_V[size - 1]) {
        return pressure_mmHg[size - 2] + (adc_value - voltage_V[size - 2]) * (pressure_mmHg[size - 1] - pressure_mmHg[size - 2]) / (voltage_V[size - 1] - voltage_V[size - 2]);
    }

    for (int i = 0; i < size - 1; i++) {
        if (adc_value >= voltage_V[i] && adc_value <= voltage_V[i + 1]) {
            return pressure_mmHg[i] + (adc_value - voltage_V[i]) * 
                   (pressure_mmHg[i + 1] - pressure_mmHg[i]) / 
                   (voltage_V[i + 1] - voltage_V[i]);
        }
    }
    return 0;
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
