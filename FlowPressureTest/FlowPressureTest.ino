/*
  YF‐S201 Water Flow Sensor & Analog Pressure Sensor
  Reads from both the water flow sensor and an analog pressure sensor
*/

// Pin connections
const unsigned char FLOW_SENSOR_PIN = 2; // Flow sensor input pin
const unsigned char PRESSURE_SENSOR_PIN = A0; // Pressure sensor input pin

// Interval for reading sensors in milliseconds
const unsigned long SENSOR_READ_INTERVAL = 50;

volatile int flow_frequency;  // Measures flow sensor pulses
float l_min;                  // Calculated litres/hour as a float
unsigned long currentTime;
unsigned long cloopTime;
unsigned long analogSensorTime = 0; // Timing for analog sensor readings

void flow()  // Interrupt function for flow sensor
{
  flow_frequency++;
}

void setup() {
  // Initialize flow sensor
  pinMode(FLOW_SENSOR_PIN, INPUT);
  digitalWrite(FLOW_SENSOR_PIN, HIGH); // Optional Internal Pull-Up
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flow, RISING); // Setup Interrupt for flow sensor
  sei(); // Enable interrupts

  // Set initial times
  currentTime = millis();
  cloopTime = currentTime;
  analogSensorTime = currentTime;
  Serial.println("RESET");
}

void loop() {
  currentTime = millis();

  // Read and print flow and pressure sensor data
  if (currentTime >= (cloopTime + SENSOR_READ_INTERVAL)) {
    cloopTime = currentTime; // Updates cloopTime

    float flowRate = readFlowSensor();
    int pressureValue = readPressureSensor();

    printSensorValues(flowRate, pressureValue);
  }
}

float readFlowSensor() {
  // Calculate flow rate in L/min
  float flowRate = (flow_frequency / 9.68) * 4; // Multiplied by 4 for 250 ms timing
  flow_frequency = 0; // Reset Counter
  return flowRate;
}

int readPressureSensor() {
  // Read the analog value from the pressure sensor
  return analogRead(PRESSURE_SENSOR_PIN);
}

void printSensorValues(float flowRate, int pressureValue) {
  // Print both flow and pressure values on one line
  Serial.print("Flow: ");
  Serial.print(flowRate, 2); // Print flow rate with 2 decimal places
  Serial.print(" L/min ; Pressure: ");
  Serial.println(pressureValue); // Print pressure value
}
