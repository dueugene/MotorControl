#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
/*this is code to communicate with a simple 2-phase motor controller
the protocol is in hexadecimal and reference can be found at bottom of this file.

excpetions are not handled internally, all exceptions are of type boost::system::system_error and
can occur in possible 3 cases:
	1. SimpleSerial serial initialization fails
	2. failed write to serial
	3. failed read from serial
these can occur in all public functions, and therefore all uses of these functions should be in a try/catch statement.
Only pointers to this object should be made as the object initialization is also made in a try/catch statement.

Eugene Du
Updated June 1, 2015*/

//Serial communication protocol used
#include "SimpleSerial.h"

class MotorControl {

public:
	//constructor that accepts ports name and baud rate
	//throws boost::system::system_error if connection cannot be made
	MotorControl(std::string com, unsigned int baud);
	//tells all motors to stop motion immediately
	//throws boost::system::system_error if error in read or write
	void stop();
	//moves a motor to a specific postion
	//throws boost::system::system_error if error in read or write
	void position(int motor, int location);
	//returns integer value of motor position
	//throws boost::system::system_error if error in read or write
	int get_location(int motor);
	//returns max range of motor as an int, program should use this to 
	//not exceed the values returned
	//throws boost::system::system_error if error in read or write
	int get_max_location(int motor);
	
private:
	//writes send_code to controller and then reads reply
	//does not check if send_code is valid
	//throws boost::system::system_error if error in read or write
	void write();
	//changes byte 7 such that a 7 byte code sums to 0
	void sumify(unsigned char *seven);
	//checks to see that the sum of all bytes of read_code is valid
	int checksumif();
	//pointer to serial communication object
	SimpleSerial serial;
	//send code array
	unsigned char *send_code;
	//read code array (motor controller always sends seven bytes back
	unsigned char read_code[7];
	//lookup table of common codes (all commands are 7 bytes)
	unsigned char table[12][7];
	/*
	*table[0] stop motor 1
	*table[1] stop motor 2
	*table[2] read target 1
	*table[3] read target 2
	*table[4] status 1
	*table[5] status 2
	*table[6] position 1
	*table[7] position 2
	*table[8] limit 1
	*table[9] limit 2	
	*table[10] write target 1
	*table[11] write target 2*/

};

#endif //MOTORCONTROL_H


/* Communication protocol provided by the manufacturer

Serial data streams.


Data to the 2 Axis Motor controller serial device:


Serial port configuration:
	38400 baud
	8 bits
	no parity
	1 stop bit
	no flow control


7 bytes are written to the device. The first is a register id. The next four are the contents of a 32 bit register value


Byte 1:	0xff. This is a start of data stream flag.
Byte 2:	register id. Bit 7 is set to 1 for a write. Bit 7 is set to 0 for a read.

Byte 3:	low byte of 4 byte register.		Set to 0 for a read.
Byte 4:	second byte of 4 byte register.	Set to 0 for a read.
Byte 5:	third byte of 4 byte register.	Set to 0 for a read.
Byte 6:	high byte of 4 byte register.		Set to 0 for a read.

Byte 7:	A checksum byte such that the 7 bytes sum to 0 (mod 256).


The reply:

7 are read from the device. 

Byte 1:	0xff. This is a start of data stream flag.
Byte 2:	register id. Bit 7 will be set to 0.

Byte 3:	low byte of 4 byte register
Byte 4:	second byte of 4 byte register
Byte 5:	third byte of 4 byte register
Byte 6:	high byte of 4 byte register

Byte 7:	A checksum byte such that the 7 bytes sum to 0 (mod 256).


Register Definitions:

 

These system registers are intended to be read only.
Writes to these registers will not survive a power reset.  
REG_SYS_PRODUCTID			0x01		read only -> returns 0x4001
REG_SYS_VERSIONHW			0x02		read only 
REG_SYS_VERSIONDATE		0x03		read only 
REG_SYS_VERSIONSW			0x04		read only 
REG_SYS_PRODUCTID_SUBCLASS	0x05		read only
				1 or 0xffffffd9)	2 phase stepper
				2)			5 phase vexta
				3)			DC encoder






These system registers are intended to be system setup registers.
Changing these may result in a system that does not perform properly.

REG_SETUP_ACCEL_1			0x15	sets motor 1 acceleration
REG_SETUP_ACCEL_2			0x25	sets motor 2 acceleration

REG_SETUP_INITVELOCITY_1	0x16	sets initial motor 1 velocity
REG_SETUP_INITVELOCITY_2	0x26	sets initial motor 2 velocity

REG_SETUP_MAXVELOCITY_1		0x17	sets maximum motor 1 velocity
REG_SETUP_MAXVELOCITY_2		0x27	sets maximum motor 2 velocity

REG_SETUP_REVBACKLASH_1		0x18	sets motor 1 reverse backlash value
REG_SETUP_REVBACKLASH_2		0x28	sets motor 2 reverse backlash value

REG_SETUP_FWDBACKLASH_1		0x19	sets motor 1 forward backlash value
REG_SETUP_FWDBACKLASH_2		0x29	sets motor 2 forward backlash value

REG_SETUP_CONFIG_1		0x1B	sets motor 1 sensor configuration
REG_SETUP_CONFIG_2		0x2B	sets motor 2 sensor configuration

			Bit 0:	0 – near sensor is home
					1 – far sensor is home
			Bit 1:	0 – reverse seek direct
					1 – reverse seek thru home
			Bit 2:	0 – Axis 2 enabled(only for AXIS 2)
					1 – Axis 2 Disabled(only for AXIS 2)

REG_SETUP_LIMIT_1			0x1C	returns motor 1 limit value
REG_SETUP_LIMIT_2			0x2C	returns motor 2 limit value


REG_SETUP_WRITE	0x0E	writes all the above changes to the EEROM. The value passed in the write must be 1.


 
These user registers are intended for normal operation

REG_USER_TARGET_1			0x10	motor 1 seek command
REG_USER_TARGET_2			0x20 	motor 2 seek command

REG_USER_INCREMENT_1		0x11	motor 1 delta seek from current
REG_USER_INCREMENT_2		0x21 	motor 2 delta seek from current

REG_USER_CURRENT_1		0x12	motor 1 current position
REG_USER_CURRENT_2		0x22 	motor 2 current position

REG_USER_LIMIT_1			0x13	motor 1 limit seek command
REG_USER_LIMIT_2			0x23 	motor 2 limit seek command

		The value written drives the motor to one of the limits:
			0 – home motor
			1 – limit motor
			2 – aborts any current motor motion.

REG_USER_STATUS_1			0x14
REG_USER_STATUS_2			0x24

		Lower 8 bits (Bit 0 thru 7).
			0 – idle
			1 – driving to home
			2 – coming off home
			3 – driving to limit
			4 – seeking forward
			5 – deaccel in forward direction
			6 – forward backlash (i.e. rev after overshoot)
			7 – seeking reverse
			8 – deaccel in reverse direction
			9 – reverse backlash (i.e. forward after overshoot)
			11- forward deaccel during an abort
			12- reverse deaccel during an abort

		Bit 8 – motor on home sensor
		Bit 9 – motor on limit sensor

*/