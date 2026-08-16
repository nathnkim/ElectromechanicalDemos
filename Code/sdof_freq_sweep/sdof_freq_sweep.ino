// First iteration code for the SDoF control demo. Rough sketch to run on an Arduino Micro.
// Written Fall 2023 by Nathan Kim (nathnkim@umich.edu) for MECHENG 360 with Dr. Jeffrey Koller at the University of Michigan - Ann Arbor.

const int potentiometerPin = 18; // Arduino pin connected to the potentiometer. Be careful about how ground and Vcc are connected, they could cause potentiometer reading increase with different rotation driections.
const int pwmOutputPin_pos = 9;  // Arduino PWM output pin 
const int pwmOutputPin_neg = 11; // Arduino PWM output pin

// void setup() is code that runs once on the Arduino when powered on. Pins are initialized, library preliminaries are handled.
void setup() {
  pinMode(pwmOutputPin_pos, OUTPUT);
  pinMode(pwmOutputPin_neg, OUTPUT);
  // Serial.begin(9600); // Uncomment if printing sensor value
}

//void loop() is code that runs infinitely once setup() is executed, until the Arduino is powered off.
void loop() {
  // Read the value from the potentiometer
  int sensorValue = analogRead(potentiometerPin);

  // Map the sensor value to a range of 0-255 (for PWM)
  int pwmValue = map(sensorValue, 0, 1023, 0, 255);

  /* // Print the sensor value and PWM value to the Serial Monitor
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(" | PWM Value: ");
  Serial.println(pwmValue);
  */

  // Write the PWM value to the PWM output pin
  analogWrite(pwmOutputPin_pos, pwmValue);
}
