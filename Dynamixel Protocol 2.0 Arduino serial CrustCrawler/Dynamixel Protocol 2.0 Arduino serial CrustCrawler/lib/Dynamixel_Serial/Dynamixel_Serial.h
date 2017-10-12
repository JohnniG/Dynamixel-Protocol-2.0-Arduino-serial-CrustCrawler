#ifndef Dynamixel_Serial_h
#define Dynamixel_Serial_h

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define STATUS_PACKET_TIMEOUT           50
#define STATUS_FRAME_BUFFER             5

class DynamixelClass {
public:
    // Constructor
    DynamixelClass(): Direction_Pin(-1), Status_Return_Value(READ) { }

    void begin(long);
    void begin(HardwareSerial&, long);
    void begin(Stream&);
    void end(void);

    void setHoldingTorque(unsigned char ID, bool Set);
    void setGoalPosition(unsigned char ID, unsigned int pos);
    void setGoalVelocity(unsigned char ID, unsigned int vel);
	void setProfileAcceleration(unsigned char ID, unsigned int pac);
	void setProfileVelocity(unsigned char ID, unsigned int pvl);

    void setNGoalPositions(int m1, int m2, int m3, int m4, int m5);

    void setDirectionPin(unsigned char);
    unsigned int writeN(unsigned char ID, unsigned short addr, unsigned char *arr, int n);
    unsigned int syncN(unsigned short addr, unsigned char*arr, int n, int dataN);

    Stream *_serial;

private:
    unsigned short update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size);
    void transmitInstructionPacket(int transLen);
    unsigned int readStatusPacket(void);
    void clearRXbuffer(void);

    unsigned char   Instruction_Packet_Array[64];   // Array to hold instruction packet data
    unsigned char   Status_Packet_Array[8];         // Array to hold returned status packet data
    unsigned long   Time_Counter;                   // Timer for time out watchers
    char            Direction_Pin;                  // Pin to control TX/RX buffer chip
    unsigned char   Status_Return_Value;            // Status packet return states ( NON , READ , ALL )
};


extern DynamixelClass Dynamixel;

#endif
