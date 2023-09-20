/*
 * @Author: ELEGOO original / Shields modified
 * @Description: Smart Robot Car V4.0
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // initialize the bot 
  Application_FunctionSet.ApplicationFunctionSet_Init();
  
  // enable watchdog timer with 25ms timeout
  wdt_enable(WDTO_2S);
}

void loop()
{
  // watchdog timer reset; note that if cumulative loop execution takes over 25ms to (configurable above), it will force a reset, so keep loop contents tidy
  wdt_reset();
  
  // read sensors that should always be monitored
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();

  // do something if the button on the control board is pressed
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  
  // control the onboard LED 
  Application_FunctionSet.ApplicationFunctionSet_RGB();

  // Checks the stop signal to put bot into standby if appropriate IR signal is received
  //Application_FunctionSet.ApplicationFunctionSet_CheckStopSignal();

  // listen to a PPM-based controller if connected
  Application_FunctionSet.ApplicationFunctionSet_RadioControl();

  
  // Use this if doing advanced work that requires analysis of sensor data, etc.
  //Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

 }
