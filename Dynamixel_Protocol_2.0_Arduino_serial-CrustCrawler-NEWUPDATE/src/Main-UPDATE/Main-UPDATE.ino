////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// How to run it: upload to arduino with the dynamixel_serial library installed in your arduino libraries folder.             //
//                After upload, disconnect usb from arduino and add power to CrustCrawler and the arduino board.              //
//                The program should start by it self. :)                                                                     //
//                                                                                                                            //
//                PIN Setup:  Green wire to PIN 10,                                                                           //
//                            Yellow wire to PIN 11,                                                                          //
//                            Black wire to ground,                                                                           //
//                            Red wire to 5v,                                                                                 //
//                            Blue wire to PIN2                                                                               //
//                                                                                                                            //
//                This code was developed in collaboration with several groups, to enable all the groups a good               //
//                base code to start programming the CrustCrawler from. ;)                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "Dynamixel_Serial.h"
#include <SoftwareSerial.h>


#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)
#define LED13 0x0D
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

SoftwareSerial mySerial(10, 11);    // RX, TX

int *data;

void setup(){

  Serial.flush();                                       // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                   // We now need to set Ardiuno to the new Baudrate speed 115200
  Serial.begin(57600);                                  // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                            // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);          // Optional. Set direction control pin which control if the program writes or reads to and from the robot

  // Turn on hold on the servos:
  Dynamixel.setHoldingTorque(0x01, true);               //Turn on hold torque on servo 1
  Dynamixel.setHoldingTorque(0x02, true);               //Turn on hold torque on servo 2
  Dynamixel.setHoldingTorque(0x03, true);               //Turn on hold torque on servo 3
  Dynamixel.setHoldingTorque(0x04, true);               //Turn on hold torque on servo 4
  Dynamixel.setHoldingTorque(0x05, true);               //Turn on hold torque on servo 5

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
  Dynamixel.setProfileVelocity(0x04, 200);  //Set the Profile Velocity for each servo. (max. is 1023)
  Dynamixel.setProfileVelocity(0x05, 200);  //Set the Profile Velocity for each servo. (max. is 1023)
  */


  /*
  //Get position for servos in steps
  Dynamixel.getPosition(0x01); 
  Dynamixel.getPosition(0x02);
  Dynamixel.getPosition(0x03);
  Dynamixel.getPosition(0x04);
  Dynamixel.getPosition(0x05);
    
  //Get position for servos in degrees
  Dynamixel.getPositionD(0x01);
  Dynamixel.getPositionD(0x02);
  Dynamixel.getPositionD(0x03);
  Dynamixel.getPositionD(0x04);
  Dynamixel.getPositionD(0x05);
  
  //Get load on servos in maximum procent (The power required by the servo to hold its current position)
  Dynamixel.getLoad(0x02);  
  
  //Saving and printing positon from all servos
  data = Dynamixel.getPositionN(); 

  for(int i = 0; i < 5; i++){
    Serial.println(data[i]);
  }

  */
}


void loop(){

  int id1 = 2071; //Servo 1 goal position
  int id2 = 2048; //Servo 2 goal position
  int id3 = 2048; //Servo 3 goal position
  int id4 = 2548; //Servo 4 goal position
  int id5 = 1548; //Servo 5 goal position

 Dynamixel.setNGoalPositions(-1, id2, id3, -1, -1);  //Set goal position of all the servos
    
  int i = 1;
  int id2P;

  // This while loop keeps putting servo 2's current potition into an integer
  while(i > 0){
    id2P =  Dynamixel.getPosition(0x02);  //Servo 2 current position in steps

    // This if statement checks if servo 2 have reached goal position, before it starts the next move
    if(id2==id2P){
       Dynamixel.setNGoalPositions(id1, -1, -1, id4, id5);  //Set goal position of all the servos
       i = 0; //just to end the while loop
    }
  }
}

