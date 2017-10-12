////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// How to run it: upload to arduino with the dynamixel_serial library installed in your arduino libraries folder.             //
//                After upload, disconnect usb from arduino and add power to CrustCrawler and the arduino board.              //
//                When the arm's servos hold their torque, press the arduino restart button and the program should run. :)    //
//                                                                                                                            // 
//                PIN Setup:  Green wire to PIN 10,                                                                           //
//                            Yellow wire to PIN 11,                                                                          //
//                            Black wire to ground,                                                                           //
//                            Red wire to 5v,                                                                                 //
//                            Blue wire to PIN2                                                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Dynamixel_Serial.h>
#include <SoftwareSerial.h>


#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

SoftwareSerial mySerial(10, 11); // RX, TX

void setup(){
  pinMode(LED13, OUTPUT);
  digitalWrite(LED13, HIGH);

  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);          // Optional. Set direction control pin

  // Set the angular limits for each servo:
  //Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);    // set mode to SERVO and set angle limits

//Examples:
  //Dynamixel.setNGoalPositions(-1, 2048, 2048, -1, -1);  //Turn on all the servos at once and sets Servo 2 & 3 to 180 degree
  //Dynamixel.setGoalPosition(0x02, 2048);                //Set goal position of servo 2 to 2048 = 180 degree
  //delay(9000);
  //Dynamixel.setGoalPWM(0x01, 300);                      //Set the goal Pulse Width Modulation for each servo. (max. is 885)
  //Dynamixel.setProfileAcceleration(0x01, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  //Dynamixel.setProfileVelocity(0x01, 100);  //Set the Profile Velocity for each servo. (max. is 1023)

 
  // Set the Profile acceleration.
  Dynamixel.setProfileAcceleration(0x01, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x02, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x03, 10);  //Set the Profile Acceleration for each servo. (max. is 32767)
  /*
  Dynamixel.setProfileAcceleration(0x04, 300);  //Set the Profile Acceleration for each servo. (max. is 32767)
  Dynamixel.setProfileAcceleration(0x05, 300);  //Set the Profile Acceleration for each servo. (max. is 32767)
  */

  // Set the Profile velocity.
  Dynamixel.setProfileVelocity(0x01, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x02, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x03, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  /*
  Dynamixel.setProfileVelocity(0x04, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x05, 100);  //Set the Profile Velocity for each servo. (max. is 1023)
  */
  
  // Ball grab, lift, 180 turn and place back down.
  Dynamixel.setNGoalPositions(-1, 2048, 2048, -1, -1);  //Set goal position of all the servos
  delay(1500);
  Dynamixel.setNGoalPositions(2071, -1, -1, 2048, 2048);  //Set goal position of all the servos
  delay(1000);
  Dynamixel.setNGoalPositions(2071, 2600, 3100, 2448, 1648);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(2071, 2855, 3309, 2448, 1648);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(2071, 2855, 3309, 2180, 1980);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(2071, 1660, 3309, 2180, 1980);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(15, 2333, 794, 2180, 1980);  //Set goal position of all the servos
  delay(1000);
  Dynamixel.setNGoalPositions(15, 1278, 794, 2180, 1980);  //Set goal position of all the servos
  delay(1000);
  Dynamixel.setNGoalPositions(15, 1278, 794, 2448, 1648);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(15, 2048, 2048, 2448, 1648);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(2071, 2048, 2048, 2048, 2048);  //Set goal position of all the servos
  delay(500);
  Dynamixel.setNGoalPositions(2071, 2048, 2048, 2448, 1648);  //Set goal position of all the servos
  Dynamixel.setNGoalPositions(2071, 2048, 2048, 2048, 2048);  //Set goal position of all the servos
  Dynamixel.setNGoalPositions(2071, 2048, 2048, 2448, 1648);  //Set goal position of all the servos
  Dynamixel.setNGoalPositions(2071, 2048, 2048, 2048, 2048);  //Set goal position of all the servos
} 

void loop(){
  // Turn on hold on the servos:
  Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5
  
}
