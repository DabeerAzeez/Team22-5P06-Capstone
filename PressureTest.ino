const float OffSet = 0.450;
const int AVG_LENGTH = 3; // Length of the rolling average array

float V, P;
float rolling_avg = 0.0;
float pressure_data[AVG_LENGTH] = {0.0}; // Array to hold the last AVG_LENGTH pressure readings
int data_index = 0;                      // Index for current data point

void setup() 
{ 
  Serial.begin(9600);        // open serial port, set the baud rate to 9600 bps 
  Serial.println("/** Water pressure sensor demo **/"); 
} 

void loop() 
{ 
  // Connect sensor to Analog 0 
  V = analogRead(0) * 5.00 / 1024;     // Sensor output voltage 
  P = (V - OffSet) * 400;              // Calculate water pressure 

  // Update the rolling average array
  pressure_data[data_index] = P;
  data_index = (data_index + 1) % AVG_LENGTH; // Move to the next index, wrap around every AVG_LENGTH points

  // Calculate the rolling average
  rolling_avg = 0.0;
  for (int i = 0; i < AVG_LENGTH; i++) {
    rolling_avg += pressure_data[i];
  }
  rolling_avg /= AVG_LENGTH;

  // Output both the current pressure and the rolling average to the Serial Plotter
  // Serial.print("Current Pressure: ");
  // Serial.print(P, 1);  // Current pressure
  // Serial.print("\tRolling Average: ");
  Serial.println(rolling_avg, 1); // Rolling average pressure

  delay(250); 
}
