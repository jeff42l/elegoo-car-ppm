/*
 * @Author: ELEGOO original / Shields modified
 * @Description: Smart Robot Car V4.0
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
}

void loop()
{
  //put your main code here, to run repeatedly :
  wdt_reset();
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_RadioControl();

  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

 }
