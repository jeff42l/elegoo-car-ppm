/*
 * @Description: PPM radio controlled integration for Elegoo V3 Plus bots
 * @Author: Shields, modified from Elegoo base code
 */
#include <stdio.h>
#include "ppm.h"

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define LED_Pin 13

/* Input pin for the PPM signal from the radio receiver */
#define PIN_A4 4

#define motorMax 250 //PWM(Motor speed/Speed)

// yes, direction_just is odd, it just means right (carryover from V4 code)
#define direction_back 0
#define direction_just 1
#define direction_void 2

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

/*
  Stop motor control：Turn off the motor drive
*/
void stop(bool debug = false)
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if (debug)
    Serial.println("Stop!");
}

void radio_control() {
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
      bool forward = throttle < 1475;
      bool left = steer < 1470;
      bool right = steer > 1530; 

    /*  
      Serial.print("throttle(");
      Serial.print(throttle);
      Serial.print(") steer(");
      Serial.print(steer);
      Serial.println(") ");
    */  
      
      if (forward || reverse || left || right) {
        motorgo = true;

        // calculate proportional movement and max speed
        // values should center at 1500, low is 1000, high is 2000
        float max_speed_percent = (speedmax - 1000) / 1000.0;
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
        MotorControl(motA_dir, motA_spd, motB_dir, motB_spd, true); //Motor control
      } else {
        MotorControl(direction_void, 0, direction_void, 0, false); //Motor control 
      }
    }
  }
}

void MotorControl(short right_dir, short right_spd, short left_dir, short left_spd, bool enabled) {
  if (enabled) {
    if (right_dir == direction_just) {
      analogWrite(ENB, right_spd);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else if (right_dir == direction_back) {
      analogWrite(ENB, right_spd);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (right_dir == direction_void) {
      digitalWrite(ENB, LOW);
    }

    if (left_dir == direction_just) {
      analogWrite(ENA, left_spd);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (left_dir == direction_back) {
      analogWrite(ENA, left_spd);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else if (left_dir == direction_void) {
      digitalWrite(ENA, LOW);
    }
  } else {
    digitalWrite(ENA, LOW); //Turn off the motor enable pin
    digitalWrite(ENB, LOW);
  }
}


void setup(void)
{
  Serial.begin(9600); //initialization

  pinMode(IN1, OUTPUT); //Motor-driven port configuration
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(2, INPUT);

  // start PPM traffic on pin A0 -- note if you have a non-standard-issue cable, you may need to adjust this pin value
  ppm.begin(2, false);
}

void loop(void)
{
  
  radio_control(); 

}
