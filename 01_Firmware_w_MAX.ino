//Includes necessary libraries
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MAX31856.h>

//Include Device Sepcific Header From Sketch>>Import Library (In This Case LinxArduinoMega2560.h)
//Also Include Desired LINX Listener From Sketch>>Import Library (In This Case LinxSerialListener.h)
#include <LinxArduinoMega2560.h>
#include <LinxSerialListener.h>

// Create objects for each thermocouple using the pin layout below
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 kthermo = Adafruit_MAX31856(6,7,8,9);
Adafruit_MAX31856 tthermo = Adafruit_MAX31856(2,3,4,5);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  //Creates an object to utilize the AFMS shield
//Creates an object to operate the fan (DC motor) connected in slot 2 of the AFMS shield
Adafruit_DCMotor *fan = AFMS.getMotor(2);
//Creates an object to utilize the stepper motor, initialized by the number of steps per revolution
//200 steps for the 1.8 degree to step ratio, and the channel the stepper is connected in
Adafruit_StepperMotor *stepper = AFMS.getStepper(200,2);

/***********************************************************
 timeDelay = Overall system delay (in ms)
 stepTrack = variable to monitor current stepper position (in steps)
 x = Set point (percentage) set in LabVIEW
 setStep = Set point in steps
 del = difference between setStep and stepTrack
 fanSpeed = rotation speed of fan, set in LabVIEW
 upTemp = Upstream temperature from T-type thermocouple
 downTemp = Downstream temperature from K-type thermocouple
 up/downMult = Multiplier value for thermocouple outlet
 up/downMod = Modulo value for thermocouple outlet

 Thermocouple Value (calculated in LabVIEW = (255 * multiplier) + modulo
 ***********************************************************/
int timeDelay, stepTrack, del, setStep, fanSpeed;
int upMult, upMod, downMult, downMod;
float x, upTemp, downTemp;

//Create A Pointer To The LINX Device Object We Instantiate In Setup()
LinxArduinoMega2560* LinxDevice;

int customCommand();  //Creates a custom command object

//Initialize LINX Device And Listener
void setup()
{
  //Instantiate The LINX Device
  LinxDevice = new LinxArduinoMega2560();
  
  //The LINX Listener Is Pre Instantiated, Call Start And Pass A Pointer To The LINX Device And The UART Channel To Listen On
  LinxSerialConnection.Start(LinxDevice, 0);
  LinxSerialConnection.AttachCustomCommand(0, customCommand);
  
  AFMS.begin(1600);  //Initializes the motor shield at 1.6kHz
  fan->run(RELEASE);  //Cuts power to the fan
  fan->setSpeed(0);  //Sets the fan speed to 0
  stepper->release();  //Cuts power to the stepper motor
  stepper->setSpeed(255);  //Sets the rotation speed of the stepper to 255 (maximum)
  timeDelay = 1;  //Sets the time delay to 1 ms
  stepTrack = 0;

  // Initialize thermocouples
  tthermo.begin();
  kthermo.begin();
  tthermo.setThermocoupleType(MAX31856_TCTYPE_T);
  kthermo.setThermocoupleType(MAX31856_TCTYPE_K);

  // Set Thermocouple threshold temperatures
  tthermo.setColdJunctionFaultThreshholds(-100, 100);
  tthermo.setTempFaultThreshholds(0, 150);

  kthermo.setColdJunctionFaultThreshholds(-100, 100);
  kthermo.setTempFaultThreshholds(0, 2000);
}

void loop()
{
  //Listen For New Packets From LabVIEW
  LinxSerialConnection.CheckForCommands();
  
  fan->run(FORWARD); //set the fan direction to forward
}

//Custom command that will be run when called within the block diagram
int customCommand(unsigned char numInputBytes, unsigned char* input, unsigned char* numResponseBytes, unsigned char* response){
  fanSpeed = input[0];  //sets fan speed to the first array value
  x = input[1];  //sets percent valve open to second array value
  stepTrack = input[2];  //sets step tracker to third array value
  
  *numResponseBytes = 5;  //sets the output to two bytes
    
  x = map(x, 0, 100, 0, 250);  //Maps the percent valve open value to steps
  
  setStep = (int)x;  //Sets setStep to x cast to type int
  del = setStep - stepTrack;  //Calculates the difference in the set point and actual position

  //Determines stepper rotation if the set point is different from the current position
  if(del != 0){
    if(del > 0){
      stepper->onestep(FORWARD, DOUBLE);  //If the difference is positive the stepper needs to rotate foward one step
      stepTrack += 1;  //Increments the step tracker by one step
    }else if(del < 0){
      stepper->onestep(BACKWARD, DOUBLE);  //Rotates stepper back one step
      stepTrack -= 1;  //Increments step tracker by one step
    }
  }else{
    stepper->release();  //Removes current from stepper.  Doesn't provide holding torque but also prevents stepper from overheating
  }

  upTemp = tthermo.readThermocoupleTemperature();  //Reads upstream thermocouple and records temp
  downTemp = kthermo.readThermocoupleTemperature();  //Reads downstream thermocouple and records temp

  upMult = ((int)upTemp) / 255;  //Calculates truncated multiplier of upTemp/255
  upMod = ((int)upTemp) % 255;  //Calculates the remainder of upTemp/255
  downMult = ((int)downTemp) / 255;  //Calculates truncated multiplier of downTemp/255
  downMod = ((int)downTemp) % 255;  //Calculates the remainder of downTemp/255
  
  fan->setSpeed(fanSpeed*2.5);  //Sets the fan speed to a speed between 0 and 250
  response[0] = stepTrack;  //Sets the first response array value to the step tracker
  response[1] = upMult;  //Sets the second response to the upstream temperature multiplier
  response[2] = upMod;  //Sets the third response to the upstream temperature modulo
  response[3] = downMult;  //Sets the fourth response to the downstream temperature multiplier
  response[4] = downMod;  //Sets the fifth response to the downstream temperature modulo
  
  delay(timeDelay);  //Delays the system
  return 0;  //Required value for custom command
}
