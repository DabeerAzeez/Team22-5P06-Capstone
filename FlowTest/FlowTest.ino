/*
YF‐S201 Water Flow Sensor
Water Flow Sensor output processed to read in litres/hour
Adaptation Courtesy: www.hobbytronics.co.uk
*/

volatile int flow_frequency; // Measures flow sensor pulses

float l_min; // Calculated litres/hour as a float
unsigned char flowsensor = 2; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;

void flow() // Interrupt function
{
   flow_frequency++;
}

void setup()
{
   pinMode(flowsensor, INPUT);
   digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
   Serial.begin(9600);
   attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING); // Setup Interrupt
   sei(); // Enable interrupts
   currentTime = millis();
   cloopTime = currentTime;
   Serial.println("RESET");
}

void loop()
{
   currentTime = millis();
   // Every 250 ms, calculate and print litres/hour as a float
   if (currentTime >= (cloopTime + 250))
   {
      cloopTime = currentTime; // Updates cloopTime

      Serial.println(flow_frequency, 2); // Print litres/hour with 2 decimal places

      // Pulse frequency (Hz) = 9.68Q, Q is flow rate in L/min.
      // Formula: (Pulse frequency x 60 min) / 7.5Q = flowrate in L/min
      l_min = (flow_frequency / 9.68) * 4; // Multiplied by 4 to account for 250 ms timing

      flow_frequency = 0; // Reset Counter

      // Serial.print(l_min, 2); // Print litres/hour with 2 decimal places
      // Serial.println(" L/min");
   }
}
