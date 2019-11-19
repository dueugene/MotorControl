#include "MotorControl.h"
#include "windows.h"//used to include a the function Sleep(int) which pauses the program
					//between reads and writes, in case there is overflow. If used on a different system
					//consider find a differenct delay/pause function
#include <iostream>
using namespace std;

MotorControl::MotorControl(std::string com, unsigned int baud):serial(com,baud)
{
	//attempts to connect to specified serial port, if fails throws 
	//boost::system::system_error.

	//initialization of lookup table for common transfer codes
	int i,j;
	//first the first column to 0xff, set the rest to 0
	for (i=0;i<12;i++){
		for (j=0;j<7;j++){
			if (j==0){
				table[i][j]=0xff;
			}
			else {
				table[i][j]=0;
			}
		}
	}
	//populate table to standard commands (eg. read motor 1 position)
	table[0][1]=0x93;
	table[1][1]=0xa3;
	table[0][2]=0x02;
	table[1][2]=0x02;
	table[2][1]=0x10;
	table[3][1]=0x20;
	table[4][1]=0x14;
	table[5][1]=0x24;
	table[6][1]=0x12;
	table[7][1]=0x22;
	table[8][1]=0x1c;
	table[9][1]=0x2c;
	table[10][1]=0x90;
	table[11][1]=0xa0;

	//sumifies all codes in table
	//the last byte is a check byte such that sum of all bytes is 0
	for(i=0;i<12;i++){
		sumify(table[i]);
	}
}


void MotorControl::write(){
	//attempts to write a valid code then reads the response,
		serial.write(send_code,sizeof(unsigned char)*7);
		Sleep(5);
		serial.read(read_code,sizeof(unsigned char)*7);
}

void MotorControl::stop(){
	//retrieves stop codes from lookup table and calls send
	send_code=table[0];
	write();
	send_code=table[1];
	write();
}

void MotorControl::position(int motor, int location){
	//prepares correct code for a change in position and calls write
	
	// load the command from the template
	send_code=table[motor+9];

	// split the desired integer position value into a high and low byte for sending
	send_code[2]=location&0xff;
	send_code[3]=(location>>8)&0xff;

	// recalculate checksum and send
	sumify(send_code);
	write();
}

void MotorControl::sumify(unsigned char *seven){
	//create two's complement of sum(seven[0]:seven[5])
	seven[6]=-seven[0]-seven[1]-seven[2]-seven[3]-seven[4]-seven[5];
}

int MotorControl::checksumif(){
	//checks to see if return code is valid,
	//it is not used but can be used if needed for a more robust communication
	return !(send_code[6]+send_code[0]+send_code[1]+send_code[2]+send_code[3]+send_code[4]+send_code[5]);
}

int MotorControl::get_location(int motor){
	//sends code to get location, then returns location of motor as an integer
	send_code=table[motor+5];
	write();
	//recreating integer from binary
	int blah=read_code[2]|(read_code[3]<<8);
	return blah;
}

int MotorControl::get_max_location(int motor){
	//sends code to get max location, then returns max location as an integer
	int temp;
	send_code=table[motor+7];
	write();
	temp=read_code[3];
	temp=(temp<<8)|read_code[2];
	return temp;
}

