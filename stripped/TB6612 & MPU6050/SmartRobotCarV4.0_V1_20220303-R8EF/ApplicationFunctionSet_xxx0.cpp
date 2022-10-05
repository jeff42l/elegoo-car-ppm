/*
 * @Author: ELEGOO original / Shields modified
 * @Description: Smart Robot Car V4.0
 */

//TODO: add dead zone variable 


#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#include "ppm.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;
/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}


/*Mode Control List*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*Standby Mode*/
  RadioControl_mode,        /*Line Tracking Mode*/
  ObstacleAvoidance_mode, /*Obstacle Avoidance Mode*/
  Follow_mode,            /*Following Mode*/
  Rocker_mode,            /*Rocker Control Mode*/
  CMD_inspect,
  CMD_Programming_mode,                   /*Programming Mode*/
  CMD_ClearAllFunctions_Standby_mode,     /*Clear All Functions And Enter Standby Mode*/
  CMD_ClearAllFunctions_Programming_mode, /*Clear All Functions And Enter Programming Mode*/
  CMD_MotorControl,                       /*Motor Control Mode*/
  CMD_CarControl_TimeLimit,               /*Car Movement Direction Control With Time Limit*/
  CMD_CarControl_NoTimeLimit,             /*Car Movement Direction Control Without Time Limit*/
  CMD_MotorControl_Speed,                 /*Motor Speed Control*/
  CMD_ServoControl,                       /*Servo Motor Control*/
  CMD_LightingControl_TimeLimit,          /*RGB Lighting Control With Time Limit*/
  CMD_LightingControl_NoTimeLimit,        /*RGB Lighting Control Without Time Limit*/

};

/*Application Management list*/
struct Application_xxx
{
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

#define motorMax 255 //PWM(Motor speed/Speed)

/* support for PPM radio control */
const long PPMInterval = 50;
unsigned long PPMPreviousMillis = 0;

int pDir[4] = {direction_void,0,direction_void,0};

// PPM channel layout (update for your situation)
#define RIGHTX          1
#define RIGHTY          2
#define LEFTY           3
#define LEFTX           4
#define RIGHTROCK       5
#define RIGHTMOM        6
#define LEFTROCK        7
#define LEFTPOT         8


void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  Application_SmartRobotCarxxx0.Functional_Mode = RadioControl_mode;
  
  // start PPM traffic on pin A0 -- note if you have a non-standard-issue cable, you may need to adjust this pin value
  ppm.begin(A0, false);
}

/*
 Robot car update sensors' data:Partial update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{

  // AppMotor.DeviceDriverSet_Motor_Test();
  { /*Battery voltage status update*/
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10) //read and update the data per 10ms
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        if (VoltageData_number == 500) //Continuity to judge the latest voltage value multiple 
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

}
/*
  Startup operation requirement：
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void)
{
  Application_SmartRobotCarxxx0.Functional_Mode = RadioControl_mode;
}

/*RBG_LED set*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) //Act on low power state？
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
    case /* constant-expression */ 0 ... 49:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 50 ... 99:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 100 ... 149:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 150 ... 199:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 200 ... 249:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 250 ... 299:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 300 ... 349:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 350 ... 399:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 400 ... 449:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 450 ... 499:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    default:
      break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode) //Act on mode control sequence
    {
    case /* constant-expression */ Standby_mode:
      /* code */
      {
        if (VoltageDetectionStatus == true)
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay(30);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay(30);
        }
        else
        {
          static uint8_t setBrightness = 0;
          static boolean et = false;
          static unsigned long time = 0;

          if ((millis() - time) > 10)
          {
            time = millis();
            if (et == false)
            {
              setBrightness += 1;
              if (setBrightness == 100)
                et = true;
            }
            else if (et == true)
            {
              setBrightness -= 1;
              if (setBrightness == 0)
                et = false;
            }
          }
          // AppRBG_LED.leds[1] = CRGB::Blue;
          AppRBG_LED.leds[0] = CRGB::Violet;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
      }
      break;
    case /* constant-expression */ CMD_Programming_mode:
      /* code */
      {
      }
      break;
    case /* constant-expression */ RadioControl_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Green);
      }
      break;
    case /* constant-expression */ ObstacleAvoidance_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      }
      break;
    case /* constant-expression */ Follow_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
      }
      break;
    case /* constant-expression */ Rocker_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Violet);
      }
      break;
    default:
      break;
    }
  }
}

void ApplicationFunctionSet::ApplicationFunctionSet_RadioControl(void) {
  if (Application_SmartRobotCarxxx0.Functional_Mode == RadioControl_mode) {

    // Interval at which the PPM is updated
    unsigned long currentMillis = millis();
    if (currentMillis - PPMPreviousMillis >= PPMInterval) {
      PPMPreviousMillis = currentMillis;

      short drivemode = ppm.read_channel(RIGHTROCK);

      bool motA_dir = direction_void;
      bool motB_dir = direction_void;
      short motA_spd = 0;
      short motB_spd = 0;
      bool motorgo = false;
      
      if (drivemode <= 1600) {
        // Acquiring channel values
        short throttle = ppm.read_channel(LEFTY);
        short steer = ppm.read_channel(RIGHTX);
        short speedmax = ppm.read_channel(LEFTPOT);

        bool reverse = throttle > 1525;
        bool forward = throttle < 1425;
        bool left = steer < 1525;
        bool right = steer > 1475; 

        if (forward || reverse || left || right) {
          motorgo = true;

          // calculate proportional movement and max speed
          // values should center at 1500, low is 1000, high is 2000
          float max_speed_percent = (speedmax - 1000) / 1000.0;
          short requested_throttle = abs(throttle - 1500);
          short adjusted_throttle = constrain(((requested_throttle / 500.0) * max_speed_percent * motorMax), 0, motorMax);

          short turn_speed = abs(steer - 1500);

          

          if ((left || right) && !(forward || reverse)) {
            // spinning in place
            short adjusted_turn_speed = (turn_speed / 500.0) * max_speed_percent * motorMax;
            motA_spd = adjusted_turn_speed; 
            motB_spd = adjusted_turn_speed;

            if (left) {
              motA_dir = direction_just;
              motB_dir = direction_back;
            } else if (right) {
              motA_dir = direction_back;
              motB_dir = direction_just;
            }
          } else if (forward || reverse) {
            if (forward) {
              motA_dir = direction_just;
              motB_dir = direction_just;
            } else if (reverse) {
              motA_dir = direction_back;
              motB_dir = direction_back;
            }
            
            // proportional turning while motating
            float turn_ratio = 1.0 - (turn_speed / 500.0);
            if (right) {
              motA_spd = adjusted_throttle * turn_ratio;
              motB_spd = adjusted_throttle;
            } else if (left) {
              motA_spd = adjusted_throttle;
              motB_spd = adjusted_throttle * turn_ratio;
            } else {
              motA_spd = adjusted_throttle;
              motB_spd = adjusted_throttle;
            }
          }
        }
      } else {
        short leftthrottle = ppm.read_channel(LEFTY);
        short rightthrottle = ppm.read_channel(RIGHTY);

        short requested_left = abs(leftthrottle - 1500);
        short adjusted_left = constrain(((requested_left / 500.0) * motorMax), 0, motorMax);
        if (adjusted_left > 20) {
          motorgo = true;
          if (leftthrottle < 1475) {
            motB_dir = direction_just;
          } else if (leftthrottle > 1525) {
            motB_dir = direction_back;
          }        
          motB_spd = adjusted_left;
        }
        short requested_right = abs(rightthrottle - 1500);
        short adjusted_right = constrain(((requested_right / 500.0) * motorMax), 0, motorMax);
        if (adjusted_right > 20) {
          motorgo = true;
          if (rightthrottle < 1475) {
            motA_dir = direction_just;
          } else if (rightthrottle > 1525) {
            motA_dir = direction_back;
          }        
          motA_spd = adjusted_right;
        }
      }

      motA_spd = constrain(motA_spd,0,motorMax);
      motB_spd = constrain(motB_spd,0,motorMax);

      if (pDir[0] != motA_dir || pDir[1] != motA_spd || pDir[2] != motB_dir || pDir[3] != motB_spd) {
        pDir[0] = motA_dir;
        pDir[1] = motA_spd;
        pDir[2] = motB_dir;
        pDir[3] = motB_spd;

        if (motorgo) {
          AppMotor.DeviceDriverSet_Motor_control(motA_dir, motA_spd, motB_dir, motB_spd, true); //Motor control
        } else {
          AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, false); //Motor control 
        }
      }
    }
  }
}


/*Key command*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;//Serial.println(get_keyValue);
    switch (get_keyValue)
    {
    case /* constant-expression */ 1:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = RadioControl_mode;
      break;
    default:

      break;
    }
  }
}

/*Data analysis on serial port*/
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  if (Serial.available() > 0)
  {
    
  }
}
