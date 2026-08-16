// First iteration code for the SDoF control demo. Rough sketch to run on an Arduino Micro.
// Written Fall 2023 by Nathan Kim (nathnkim@umich.edu) for MECHENG 360 with Dr. Jeffrey Koller at the University of Michigan - Ann Arbor.

#include "AS5600.h" // Library for the magnetic encoders https://github.com/RobTillaart/AS5600
#include "Wire.h" // Library for using the I2C communication protocol https://www.arduino.cc/reference/en/language/functions/communication/wire/
#include "TCA9548.h" // Library for using the I2C multiplexer https://github.com/RobTillaart/TCA9548

// Instantiating peripheral components per their respective libraries
TCA9548 multiplexer(0x70); // I2C address of multiplexer is 0x70 per datasheet
AS5600 inputEncoder; 
AS5600 outputEncoder; 

uint8_t encoderAddress = 0x36; // I2C address of encoders

const int inputEncoderChannel = 7; // Input encoder channel on multiplexer
const int outputEncoderChannel = 6; // Output encoder channel on multiplexer

const int pwmOutputPin_neg = 9;  // PWM output pin on Arduino
const int pwmOutputPin_pos = 11; // PWM output pin on Arduino

const int limitSwitchPin_left = 1; //Left bound limit switch interrupt pin on Arduino
const int limitSwitchPin_right = 0; //Right bound limit switch interrupt pin on Arduino

const double inputPulleyRadius = 0.5375; // inches, for calculating linear distance of input pointer
const double outputPulleyRadius = 0.7275; // inches, for calculating linear distance of driven pointer

double inputTotalRotations = 0.0;  // Variable to keep track of total rotations of input pulleys
double outputTotalRotations = 0.0;  // Variable to keep track of total rotations of driven pulleys

double inputLastAngle = 0.0; // Variable to keep track of previous position of of input pulley
double outputLastAngle = 0.0; // Variable to keep track of previous position of of driven pulley

double inputAngle, outputAngle, inputAngleOffset, outputAngleOffset, inputPos, outputPos; // Variables to track states and some offsets

double previousError = 0.0; // Track previous error for PID computation
double integral = 0.0; // To integrate error for PID computation

unsigned long previousTime = 0; // To store low bound of dt for derivative of error de/dt

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Edit PID gains here
double Kp = 150.;
double Ki = 50.;
double Kd = 5.;

const unsigned int pwmLimit = 155; //Saturate PWM duty cycle output (on [0,255] in Arduinoland) to prevent overcurrent
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void setup() is code that runs once on the Arduino when powered on. Pins are initialized, interrupts are attached, library preliminaries are handled.
void setup() {
  // Intializing all pins that the Arduino takes input from or provides output to (I/O)
  pinMode(pwmOutputPin_neg, OUTPUT);
  pinMode(pwmOutputPin_pos, OUTPUT);
  pinMode(limitSwitchPin_left, INPUT);
  pinMode(limitSwitchPin_right, INPUT);

  // AS5600.h library preliminaries for reading the encoder (see library documentation)
  inputEncoder.begin(4);  //  set direction pin.
  inputEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  outputEncoder.begin(4);  //  set direction pin.
  outputEncoder.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  
  // The following chunk sets the encoder offsets (since the encoders are absolute) which are used to zero the position of the pointers.
  // The implication of this is that with this code, regardless of where the pointers are in real life, the Arduino will think
  // they are pointing right at each other due to this offset calculation.
  multiplexer.selectChannel(inputEncoderChannel); // set multiplexer to read input encoder
  inputAngleOffset = inputEncoder.rawAngle() * AS5600_RAW_TO_RADIANS; // read current angle, convert to radians
  multiplexer.selectChannel(outputEncoderChannel);// set multiplexer to read output encoder
  outputAngleOffset = outputEncoder.rawAngle() * AS5600_RAW_TO_RADIANS; // read current angle, convert to radians

  // "Attach" the limit switch interrupts- tell the Arduino to stop doing what its doing when a limit switch is pressed and save the demo from destroying itself.
  attachInterrupt(digitalPinToInterrupt(limitSwitchPin_left), leftLimitSwitchISR, RISING); // When the Arduino detects a rising edge on the left limit switch pin indicating it has been pressed,
                                                                                           // immediately stop what its doing and execute the `leftLimitSwitchISR` function.
  attachInterrupt(digitalPinToInterrupt(limitSwitchPin_right), rightLimitSwitchISR, RISING); // When the Arduino detects a rising edge on the right limit switch pin indicating it has been pressed,
                                                                                            // immediately stop what its doing and execute the `rightLimitSwitchISR` function.

  // Open serial communication for debugging https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-serial-monitor/
  Serial.begin(115200);
}

//*****************void loop() is code that runs infinitely once setup() is executed, until the Arduino is powered off.*******************************************
void loop() {
  inputPos = inputGetPosition(); //Read and store current position of input pointer
  outputPos = outputGetPosition(); //Read and store current position of output pointer

  int pwm = computePID(inputPos, outputPos); // Use PID controller to compute PWM duty cycle to apply to motor based on the pointer positions
  
  // If controller commands negative rotation, PWM duty cycle will be negative- the following code catches thus case and corrects PWM sign
  // while sdirecting the command to the negative rotation PWM input on the H-bridge 
  if (pwm < 0){
    if (pwm < (-1 * pwmLimit)){ // Saturate the duty cycle at the preset limit
      pwm = -1 * pwmLimit;
    }
    pwm *= -1;
    analogWrite(pwmOutputPin_neg, pwm);

    //Uncomment below to debug via Serial monitor and see what the commanded PWM duty cycle is
    //Serial.print("PWM Neg : "); 
    //Serial.println(pwm);

  }
  else{
    if (pwm > pwmLimit){ // Saturate the duty cycle at the preset limit
      pwm = pwmLimit;
    }
    analogWrite(pwmOutputPin_pos, pwm);

    //Uncomment below to debug via Serial monitor and see what the commanded PWM duty cycle is
    //Serial.print("PWM Pos : ");
    //Serial.println(pwm);
  }
}

//*********************** Function to print angles to serial monitor for debugging *****************************************************
void printAngles(){
  multiplexer.selectChannel(inputEncoderChannel);
  inputAngle = inputEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;
  multiplexer.selectChannel(outputEncoderChannel);
  outputAngle = outputEncoder.rawAngle() * AS5600_RAW_TO_DEGREES;

  Serial.print("Input Angle : ");
  Serial.print(inputAngle);
  Serial.print("            Output Angle : ");
  Serial.println(outputAngle);
  delay(100);
}

//*********************** Function to get the position of the input pointer************************************************************
double inputGetPosition() {
  multiplexer.selectChannel(inputEncoderChannel); // Set multiplexer to read input encoder
  inputAngle = inputEncoder.rawAngle() * AS5600_RAW_TO_RADIANS - inputAngleOffset; // Compute input angle displaced from 0 set by offset
  
  // Calculate the change in angle since the last call
  double angleChange = inputAngle - inputLastAngle;

  // Update the total number of rotations if there is actual rotation
  if (angleChange > M_PI) {
    inputTotalRotations -= 1.0;  // Handle case where angle goes backward across 2*pi
  } else if (angleChange < -M_PI) {
    inputTotalRotations += 1.0;  // Handle case where angle goes forward across 2*pi
  }
  
  // Convert the angle to linear position, preserving accumulated rotations
  double linearPosition = (inputAngle * inputPulleyRadius) + (inputTotalRotations * TWO_PI * inputPulleyRadius);

  inputLastAngle = inputAngle;
  return linearPosition;

  //Uncomment below to debug via Serial monitor 
  //Serial.println(linearPosition);
}

//*********************** Function to get the position of the output pointer ************************************************************
double outputGetPosition() {
  multiplexer.selectChannel(outputEncoderChannel); // Set multiplexer to read driven encoder
  outputAngle = -1.*(outputEncoder.rawAngle() * AS5600_RAW_TO_RADIANS - outputAngleOffset); // Compute input angle displaced from 0 set by offset
  
  // Calculate the change in angle since the last call
  double angleChange = outputAngle - outputLastAngle;
  // Update the total number of rotations if there is actual rotation
  if (angleChange > M_PI) {
    outputTotalRotations -= 1.0;  // Handle case where angle goes backward across 2*pi
  } else if (angleChange < -M_PI) {
    outputTotalRotations += 1.0;  // Handle case where angle goes forward across 2*pi
  }
  
  // Convert the angle to linear position, preserving accumulated rotations
  double linearPosition = (outputAngle * outputPulleyRadius) + (outputTotalRotations * TWO_PI * outputPulleyRadius);

  outputLastAngle = outputAngle;
  return linearPosition;

  //Uncomment below to debug via Serial monitor 
  //Serial.println(linearPosition);
}

//********************** Function to implement PID control law *********************************************************************
double computePID(double setpoint, double sensorValue) {
  
  unsigned long currentTime = millis(); // Get the current time

  unsigned long dt = currentTime - previousTime; // Calculate the time difference since the last iteration
  
  double error = setpoint - sensorValue;  // Calculate the error
  integral += error * dt / 1000.0; // Calculate the integral of the error, dt is converted to seconds
  double derivative = (error - previousError) / (dt / 1000.0);  // Calculate the derivative of the error, dt is converted to seconds
  double output = Kp * error + Ki * integral + Kd * derivative; // Calculate the control output

  // Update variables for the next iteration
  previousError = error;
  previousTime = currentTime;

  return output;
}

//********************** Function to implement safing routine when pointer bottoms out on leftmost limit switch *********************
void leftLimitSwitchISR(){
  // EDIT THIS- RIGHT NOW IT DOES NOTHING
  // Uncomment below to test limit switch via serial monitor
  //Serial.println("Left bound touched");
  }
//********************** Function to implement safing routine when pointer bottoms out on rightmost limit switch *********************  
void rightLimitSwitchISR(){
  // EDIT THIS- RIGHT NOW IT DOES NOTHING
  // Uncomment below to test limit switch via serial monitor
  //Serial.println("Right bound touched");
  }