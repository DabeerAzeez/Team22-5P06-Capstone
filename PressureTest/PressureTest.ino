void setup() {
  // Start the serial communication
  Serial.begin(9600);
  
  // Set the analog reference to external if you’re using a custom AREF voltage
  analogReference(INTERNAL);
}

void loop() {
  // Read the value from analog pin 2
  int sensorValue = analogRead(A0);
  
  // Print the sensor value to the Serial Monitor
  Serial.print("Analog Value: ");
  Serial.println(sensorValue);
  
  // Wait 100 milliseconds before the next reading
  delay(100);
}
