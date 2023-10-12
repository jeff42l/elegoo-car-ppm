/*
 * @Author: ELEGOO original / Shields modified
 * @Description: Smart Robot Car V4.0
 */


/***** Build Options *****/
/* Uncomment this to build with automatic mode enabled */
//#define AUTOENABLED
/* Uncomment this to enable servo use during operator mode */
//#define SERVOENABLED
/*************************/

#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
#include "MPU6050_getdata.h"
#include "ppm.h"

#ifdef AUTOENABLED
#include <ArduinoQueue.h>
#endif

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_Voltage AppVoltage;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIR;

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

/* maximum speed value that can be sent to motor */
#define motorMax 255 //PWM(Motor speed/Speed)

/* Control dead zones */
#define deadzone_Y 75
#define deadzone_X 30

/* minimum value the throttle knob will reduce speed to, valid values float 0.0-1.0 */
#define throttleKnob_min 0.2

/* support for PPM radio control */
const long PPMInterval = 50;
unsigned long PPMPreviousMillis = 0;

/* object that sets robot movement, this defaults to stopped */
int pDir[4] = {direction_void,0,direction_void,0};

// PPM channel layout (update for your controller, if not matching)
#define RIGHTX          1
#define RIGHTY          2
#define LEFTY           3
#define LEFTX           4
#define RIGHTROCK       5
#define RIGHTMOM        6
#define LEFTROCK        7
#define LEFTPOT         8


/* Mode Control List */
enum RobotState {
  Standby,    /* Waiting to initiate automatic function */
  Automatic,  /* In automated period */
  Operator,   /* Operator is in control */
};

/* Application definition for maintaining current state, etc. */
struct ApplicationState
{
  RobotState CurrentRobotState; 
  unsigned long CurrentTeamColor;
};
ApplicationState CurrentApplication;

/* Motor control packet */
struct MotorControlPacket {
  bool motA_dir = direction_void; // Right side motor direction
  bool motB_dir = direction_void; // Left side motor direction
  short motA_spd = 0; // Right side motor speed (0-255 or as configured)
  short motB_spd = 0; // Left side motor speed (0-255 or as configured)
  bool motorgo = false; // if false, stop, if true, process 
};

/* Initialization called from RobotCode.ino at startup time */
void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);

  // Should blink yellow if battery voltage is low
  AppVoltage.DeviceDriverSet_Voltage_Init();

  // Motor control board
  AppMotor.DeviceDriverSet_Motor_Init();

  // Button on control board
  AppKey.DeviceDriverSet_Key_Init();

  // Built-in LED
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);

  // Servo control
  AppServo.DeviceDriverSet_Servo_Init(90);
  
  // Ultrasonic sensor
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();

  // Accelerometer
  bool res_error = true;
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  // Infrared receiver
  //AppIR.DeviceDriverSet_IRrecv_Init();

  // initial state of robot
  CurrentApplication.CurrentRobotState = Standby;
  CurrentApplication.CurrentTeamColor = CRGB::Purple;
  
  // start PPM traffic on pin A0 -- note if you have a non-standard-issue cable, you may need to adjust this pin value
  ppm.begin(A0, false);
}

// gets current approximate yaw (angle on the z axis) based on relative position from where robot was powered up
float GetYaw() {
  float yawn;
  if (!AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yawn)) {
    return yawn;
  } else {
    return 0;
  }
}

// gets current raw value from the ultrasonic sensor
unsigned int GetUltrasonic() {
  unsigned int ultra;
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ultra); 
  return ultra;
}

// Turns the servo - note that 0 is right, 180 is left, and 90 is center
void Move_Servo(unsigned int angle) {
  unsigned int servoint = constrain(angle,0,180);
  AppServo.DeviceDriverSet_Servo_control(servoint);
}

// Runs the motors as directed by the passed packet
void RunMotors(MotorControlPacket mcp) {
  // Don't allow controls to send values outside allowed bounds
  mcp.motA_spd = constrain(mcp.motA_spd,0,motorMax);
  mcp.motB_spd = constrain(mcp.motB_spd,0,motorMax);

  // Only send commands to motors if the effective state has changed
  if (pDir[0] != mcp.motA_dir || pDir[1] != mcp.motA_spd || pDir[2] != mcp.motB_dir || pDir[3] != mcp.motB_spd) {
    pDir[0] = mcp.motA_dir;
    pDir[1] = mcp.motA_spd;
    pDir[2] = mcp.motB_dir;
    pDir[3] = mcp.motB_spd;

    if (mcp.motorgo) {
      AppMotor.DeviceDriverSet_Motor_control(mcp.motA_dir, mcp.motA_spd, mcp.motB_dir, mcp.motB_spd, true); //Motor control
    } else {
      AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, false); //Motor control 
    }
  }
}


#ifdef AUTOENABLED

  /* support for automatic run time in game mode */
  const long automaticRunTime = 30000; // 30 second automatic mode
  unsigned long automaticStartTime = 0; // time that automatic mode started

  // Control packet for automatic movement queuing
  struct AutomovePacket {
    short turndegrees = 0; // degrees to turn (negative values counterclockwise, positive clockwise)
    short speed = 0; // max speed to move (around 10 at minimum usuable to motorMax)
    short duration = 0; // duration to move (time in milliseconds)
    short direction = 0; // direction to move (-1 is back, 1 is forward)
  }; 

  // queue to manage movement requests
  ArduinoQueue<AutomovePacket> AutomoveQ(100);

  // variables to carry current robot movement state when in automatic mode
  bool turn_inprogress = false;
  float turn_target = 0; 
  unsigned long move_end = 0;
  short move_inprogress = 0;
  short move_speed = 0;

  // Stop all motors
  void Move_Stop(bool clearall) {
    if (clearall) {
      // clear the queue
      while (!AutomoveQ.isEmpty()) {
        AutomoveQ.dequeue();
      }
    }
    move_inprogress = 0;
    turn_inprogress = false;

    MotorControlPacket mcp; // default values are stop/zero
    RunMotors(mcp);
  }

  // Turns the robot, tries to hit the specified relative angle
  void Move_Turn(float degrees) {
    AutomovePacket amp;
    float target = GetYaw() + degrees;
    if (target > 360) {
      target = target - 360;
    }
    amp.turndegrees = target;
    AutomoveQ.enqueue(amp);
  }
  // Drive forward for the requested duration at the requested speed
  void Move_Forward(short duration, short speed) {
    AutomovePacket amp;
    amp.direction = 1;
    amp.duration = duration;
    amp.speed = speed;
    AutomoveQ.enqueue(amp);
  }
  // Drive backward for the requested duration at the requested speed
  void Move_Back(short duration, short speed) {
    AutomovePacket amp;
    amp.direction = -1;
    amp.duration = duration;
    amp.speed = speed;
    AutomoveQ.enqueue(amp);
  }

  void Move_Process() {
    if (!move_inprogress && !turn_inprogress && !AutomoveQ.isEmpty()) {
      // process next movement packet
      AutomovePacket amp = AutomoveQ.dequeue();
      if (abs(amp.turndegrees) > 0) {
        turn_inprogress = true;
        turn_target = round(GetYaw() + amp.turndegrees);
      } else if (amp.direction != 0 && amp.duration > 0) {
        move_inprogress = amp.direction;
        move_end = millis() + amp.duration;
        move_speed = amp.speed;
      }
    }
    
    MotorControlPacket mcp;

    // take actions based on what's been set in this loop
    if (turn_inprogress) {
      float curyaw = GetYaw();
      float yawdue = round(turn_target - curyaw);
      short turnspeed = 50;
      short fudge = 5;

      Serial.print("turning, yaw is ");
      Serial.print(curyaw);
      Serial.print(" with target of ");
      Serial.print(turn_target);
      Serial.print(" and calculated due ");
      Serial.println(yawdue);

      if (abs(yawdue) < 30) {
        turnspeed = 25;
      } else if (abs(yawdue) < 10) {
        turnspeed = 12;
      }

      if (yawdue < -fudge) {
        // spin left
        mcp.motA_dir = direction_just;
        mcp.motB_dir = direction_back;
        mcp.motA_spd = turnspeed;
        mcp.motB_spd = turnspeed;
        mcp.motorgo = true;
      } else if (abs(yawdue) < fudge) {
        turn_inprogress = false;
      } else if (yawdue > fudge) {
        // spin right
        mcp.motA_dir = direction_back;
        mcp.motB_dir = direction_just;
        mcp.motA_spd = turnspeed;
        mcp.motB_spd = turnspeed;
        mcp.motorgo = true;
      }
    } else if (move_inprogress > 0) {
      if (millis() < move_end) {
        mcp.motA_dir = direction_just;
        mcp.motB_dir = direction_just;
        mcp.motA_spd = move_speed;
        mcp.motB_spd = move_speed;
        mcp.motorgo = true;
      } else {
        move_inprogress = 0;
      }
    } else if (move_inprogress < 0) {
      if (millis() < move_end) {
        mcp.motA_dir = direction_back;
        mcp.motB_dir = direction_back;
        mcp.motA_spd = move_speed;
        mcp.motB_spd = move_speed;
        mcp.motorgo = true;
      } else {
        move_inprogress = 0;
      }
    }
    RunMotors(mcp);
  }

  // Called in processing loop to drive automatically
  void AutomaticMode() {
    
    /* 
      Put custom automatic control code here.
      Mind that it's running in a loop, so you will need to basically "check sensors, instruct robot" 
      Consider reducing robot "jitter" by reducing your update frequency; you can do this with a loop that checks the current time against your last update (see the PPM logic in Operator control modes below)
    */
    
    // This block of code is a poor implementation of collision avoidance
    unsigned int ultra = GetUltrasonic();
    if (ultra > 22) {
      if (turn_inprogress) {
        Move_Stop(true);
        Move_Servo(90);
      }
      Move_Forward(1000,100);
    } else {
      if (move_inprogress != 0) {
        Move_Stop(true);
      }
      Move_Turn(90);
      Move_Servo(180);
    }

    // This will process all your movement (except STOP, which will happen immediately), so leave this call here
    Move_Process();  
  }
#endif

#ifndef AUTOENABLED
  void Move_Stop(bool clearall) {
    MotorControlPacket mcp; // default values are stop/zero
    RunMotors(mcp);
  }
#endif

/*
  Check any other sensors needed to determine robot health
  Currently this is just checking battery voltage to trigger blinky lights if low
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
  On-board LED control logic
  Low battery error (blinking yellow)
  Mode indication (team color, blinking if in automatic, solid in operator) 
*/
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
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      break;
    case /* constant-expression */ 50 ... 99:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 100 ... 149:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      break;
    case /* constant-expression */ 150 ... 199:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 200 ... 249:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      break;
    case /* constant-expression */ 250 ... 299:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 300 ... 349:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      break;
    case /* constant-expression */ 350 ... 399:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 400 ... 449:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      break;
    case /* constant-expression */ 450 ... 499:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    default:
      break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (CurrentApplication.CurrentRobotState) //Act on mode control sequence
    {
      case /* constant-expression */ Standby:
        {
          // fade in/out in team color
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
          AppRBG_LED.leds[0] = CurrentApplication.CurrentTeamColor;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
        break;
      case /* constant-expression */ Automatic:
        {
          // blink in team color
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 2 /*Traversal_Number*/, CurrentApplication.CurrentTeamColor);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(50 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
        }
        break;
      case /* constant-expression */ Operator:
        {
          // solid in team color
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CurrentApplication.CurrentTeamColor);
        }
        break;
      default:
        break;
    }
  }
}

// Radio control processing for arcade mode driving
MotorControlPacket OperatorControl_Arcade() {
  MotorControlPacket mcp;
  mcp.motA_dir = direction_void;
  mcp.motB_dir = direction_void;
  mcp.motA_spd = 0;
  mcp.motB_spd = 0;
  mcp.motorgo = false;
  
  // Acquiring channel values
  short throttle = ppm.read_channel(LEFTY);
  short steer = ppm.read_channel(RIGHTX);
  short speedmax = ppm.read_channel(LEFTPOT);

  bool reverse = throttle > 1500 + deadzone_Y;
  bool forward = throttle < 1500 - deadzone_Y;
  bool left = steer < 1500 - deadzone_X;
  bool right = steer > 1500 + deadzone_X; 

  /*
  Serial.print("throttle(");
  Serial.print(throttle);
  Serial.print(") steer(");
  Serial.print(steer);
  Serial.println(") servo1(");
  Serial.println(servo1);
  Serial.println(")");
  */
  
  // This block of code puts the servo control on the left stick of the controller...
  // HOWEVER - sending the command to the servo control -- AppServo.DeviceDriverSet_Servo_control(servoint) -- is a blocking call that will prevent other actions from happening
  // I've reduced the blocking delay to the bare minimum, which renders it usable; stock code had higher values, not sure why
  #ifdef SERVOENABLED
    short servo1 = ppm.read_channel(LEFTX);
    float servoangle = 2000 - servo1; 
    servoangle = servoangle / 1000 * 180;
    unsigned int servoint = constrain(round(servoangle),0,180);
    AppServo.DeviceDriverSet_Servo_control(servoint);
  #endif

  if (forward || reverse || left || right) {
    mcp.motorgo = true;

    // calculate proportional movement and max speed
    // values should center at 1500, low is 1000, high is 2000
    float max_speed_percent = constrain((speedmax - 1000) / 1000.0, throttleKnob_min, 1.0);
    short requested_throttle = abs(throttle - 1500);
    float throttle_percent = requested_throttle / 500.0;
    short adjusted_throttle = constrain(throttle_percent * max_speed_percent * motorMax, 0, motorMax);
    short turn_speed = abs(steer - 1500);
    if (turn_speed < 30) {
      turn_speed = 0;
    }

    if ((left || right) && !(forward || reverse)) {
      // spinning in place
      float steer_percent = turn_speed / 500.0;
      short adjusted_turn_speed = constrain(steer_percent * max_speed_percent * motorMax, 0, motorMax);
      
      mcp.motA_spd = adjusted_turn_speed; 
      mcp.motB_spd = adjusted_turn_speed;

      if (left) {
        mcp.motA_dir = direction_just;
        mcp.motB_dir = direction_back;
      } else if (right) {
        mcp.motA_dir = direction_back;
        mcp.motB_dir = direction_just;
      }
    } else if (forward || reverse) {
      if (forward) {
        mcp.motA_dir = direction_just;
        mcp.motB_dir = direction_just;
      } else if (reverse) {
        mcp.motA_dir = direction_back;
        mcp.motB_dir = direction_back;
      }
      
      // proportional turning while motating
      float turn_ratio = 1.0 - (turn_speed / 500.0);
      if (right) {
        mcp.motA_spd = adjusted_throttle * turn_ratio;
        mcp.motB_spd = adjusted_throttle;
      } else if (left) {
        mcp.motA_spd = adjusted_throttle;
        mcp.motB_spd = adjusted_throttle * turn_ratio;
      } else {
        mcp.motA_spd = adjusted_throttle;
        mcp.motB_spd = adjusted_throttle;
      }
    }
  }

  return mcp;
}

// Radio control processing for tank mode operator control
MotorControlPacket OperatorControl_Tank() {
  MotorControlPacket mcp;
  mcp.motA_dir = direction_void;
  mcp.motB_dir = direction_void;
  mcp.motA_spd = 0;
  mcp.motB_spd = 0;
  mcp.motorgo = false;

  short leftthrottle = ppm.read_channel(LEFTY);
  short rightthrottle = ppm.read_channel(RIGHTY);
  short speedmax = ppm.read_channel(LEFTPOT);

  float max_speed_percent = constrain((speedmax - 1000) / 1000.0, throttleKnob_min, 1.0);

  short requested_left = abs(leftthrottle - 1500);
  short adjusted_left = constrain(((requested_left / 500.0) * motorMax * max_speed_percent), 0, motorMax);
  if (adjusted_left > 20) {
    mcp.motorgo = true;
    if (leftthrottle < (1500 - deadzone_Y)) {
      mcp.motB_dir = direction_just;
    } else if (leftthrottle > (1500 + deadzone_Y)) {
      mcp.motB_dir = direction_back;
    }        
    mcp.motB_spd = adjusted_left;
  }
  short requested_right = abs(rightthrottle - 1500);
  short adjusted_right = constrain(((requested_right / 500.0) * motorMax * max_speed_percent), 0, motorMax);
  if (adjusted_right > 20) {
    mcp.motorgo = true;
    if (rightthrottle < 1475) {
      mcp.motA_dir = direction_just;
    } else if (rightthrottle > 1525) {
      mcp.motA_dir = direction_back;
    }        
    mcp.motA_spd = adjusted_right;
  }

  // This block of code puts the servo control on the left stick of the controller
  #ifdef SERVOENABLED
    short servo1 = ppm.read_channel(LEFTX);
    float servoangle = 2000 - servo1; 
    servoangle = servoangle / 1000 * 180;
    unsigned int servoint = constrain(round(servoangle),0,180);
    AppServo.DeviceDriverSet_Servo_control(servoint);
  #endif

  return mcp;
}


#ifdef AUTOENABLED
  // Radio control listener, handles processing for game mode
  void ApplicationFunctionSet::ApplicationFunctionSet_RadioControl(void) {
    if (CurrentApplication.CurrentRobotState == Standby) { // Standby mode, where robot is waiting for activation of automatic mode
      // read every loop, looking for basically instantaneous activation
      short click = ppm.read_channel(RIGHTMOM);
      if (click > 1400) {
        // set start time
        automaticStartTime = millis();

        // set robot state
        CurrentApplication.CurrentRobotState = Automatic;
      }
    } else if (CurrentApplication.CurrentRobotState == Automatic) { // Automatic control of bot, no operator input
      // run AutomaticMode() functions, which should be checking and acting
      AutomaticMode();

      // allow stoppage by holding right button
      short click = ppm.read_channel(RIGHTMOM);
      if (click > 1400) {
        Move_Stop(true);
      }

      // if timer expires, switch to operator mode
      unsigned long currentMillis = millis();
      if (currentMillis - automaticStartTime >= automaticRunTime) {
        Move_Stop(true);
        Move_Servo(90);
        CurrentApplication.CurrentRobotState = Operator;
      } 
    } else if (CurrentApplication.CurrentRobotState == Operator) { // Operator (manual) control of bot
      // Interval at which the PPM is updated
      unsigned long currentMillis = millis();
      if (currentMillis - PPMPreviousMillis >= PPMInterval) {
        PPMPreviousMillis = currentMillis;

        MotorControlPacket mcp;

        short drivemode = ppm.read_channel(RIGHTROCK);
        // Process remote controls
        if (drivemode <= 1600) {
          mcp = OperatorControl_Arcade();
        } else {
          mcp = OperatorControl_Tank();
        }

        RunMotors(mcp);
      }
    }
  }

  // Processes presses of the onboard momentary switch
  void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
  {
    uint8_t get_keyValue;
    static uint8_t temp_keyValue = keyValue_Max;
    AppKey.DeviceDriverSet_key_Get(&get_keyValue);
  
    if (temp_keyValue != get_keyValue)
    {
      temp_keyValue = get_keyValue;
      //Serial.println(get_keyValue);
      switch (get_keyValue)
      {
        case 0: 
          CurrentApplication.CurrentRobotState = Operator;
          CurrentApplication.CurrentTeamColor = CRGB::Violet;
          break;
        case 1:
          CurrentApplication.CurrentRobotState = Standby;
          CurrentApplication.CurrentTeamColor = CRGB::Red;
          break;
        case 2:
          CurrentApplication.CurrentRobotState = Standby;
          CurrentApplication.CurrentTeamColor = CRGB::Blue;
          break;
        default:
          break;
      }
      Move_Stop(true);
    }
  }
#endif

#ifndef AUTOENABLED
  // Radio control listener, handles processing for game mode
  void ApplicationFunctionSet::ApplicationFunctionSet_RadioControl(void) {
    if (CurrentApplication.CurrentRobotState == Operator) {
      // Interval at which the PPM is updated
      unsigned long currentMillis = millis();
      if (currentMillis - PPMPreviousMillis >= PPMInterval) {
        MotorControlPacket mcp;
        PPMPreviousMillis = currentMillis;
        short drivemode = ppm.read_channel(RIGHTROCK);
        // Process remote controls
        if (drivemode <= 1600) {
          mcp = OperatorControl_Arcade();
        } else {
          mcp = OperatorControl_Tank();
        }
        RunMotors(mcp);
      }
    } else {
      Move_Stop(true); // will use default mcp, which stops
    }
  }

  // Processes presses of the onboard momentary switch
  void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
  {
    uint8_t get_keyValue;
    static uint8_t temp_keyValue = keyValue_Max;
    AppKey.DeviceDriverSet_key_Get(&get_keyValue);
  
    if (temp_keyValue != get_keyValue)
    {
      temp_keyValue = get_keyValue;
      //Serial.println(get_keyValue);
      switch (get_keyValue)
      {
        case 0: 
          CurrentApplication.CurrentRobotState = Operator;
          CurrentApplication.CurrentTeamColor = CRGB::Violet;
          break;
        case 1:
          CurrentApplication.CurrentRobotState = Operator;
          CurrentApplication.CurrentTeamColor = CRGB::Red;
          break;
        case 2:
          CurrentApplication.CurrentRobotState = Operator;
          CurrentApplication.CurrentTeamColor = CRGB::Blue;
          break;
        default:
          break;
      }
      Move_Stop(true);
    }
  }
#endif


// Checks the IR sensor for a stop signal (5, the OK button on the remote)
void ApplicationFunctionSet::ApplicationFunctionSet_CheckStopSignal(void) {
  uint8_t ir;
  if (AppIR.DeviceDriverSet_IRrecv_Get(&ir) && ir == 5) {
    CurrentApplication.CurrentRobotState = Standby;
    
    //Serial.print("Current IR value: ");
    //Serial.println(ir);
  }
}

/*Data analysis on serial port*/
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  if (Serial.available() > 0)
  {
    
  }
}


