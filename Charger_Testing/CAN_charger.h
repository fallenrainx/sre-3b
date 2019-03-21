//header file to send and receive CAN messages between the BMS and the charger
//By Nuoya Xie

#ifndef CAN_CHARGER_H_
#define CAN_CHARGER_H_

#include <mcp_can.h>

//this enum is used in message sending to the charger
typedef enum {
	charger_on = 0,
	disable_off = 1
} charger_control;

//This struct contains messages send to charger
typedef struct 
{
	uint32_t message_id; //should be 0x1806E5F4 for the first charger
	uint8_t data_length_byte; //should be 8
	//data frame (8 bytes)
	union
	{
		struct
		{
			//voltage (2 Bytes)
			uint8_t voltage_to_charger_MSB;
			uint8_t voltage_to_charger_LSB;
			
			//current (2 Bytes)
			uint8_t current_to_charger_MSB;
			uint8_t current_to_charger_LSB;
			
			//control byte
			uint8_t control; //0 = on, 1 = off
			
			//reserved 3 bytes at the end (byte 5 - 7)
			uint8_t reserved_bytes[3];	
		}data_byte_field;
		uint64_t data;
	}data_frame;

	
}BMS_to_charger_CAN_message;

//this struct contains messages from the charger
typedef struct
{
	
	uint32_t message_id; //should be 0x18FF50E5 for BMS
	uint8_t data_length_byte; //should be 8
	
	//data frame (8 bytes)
	union
	{
		struct
		{
			//voltage (2 Bytes)
			uint8_t voltage_from_charger_MSB;
			uint8_t voltage_from_charger_LSB;
			
			//current (2 Bytes)
			uint8_t current_from_charger_MSB;
			uint8_t current_from_charger_LSB;
			
			//status (1 byte)
			union
			{
				uint8_t status_byte; //0 = on, 1 = off
				struct 
				{
					uint8_t bit0 : 1;
					uint8_t bit1 : 1;
					uint8_t bit2 : 1;
					uint8_t bit3 : 1;
					uint8_t bit4 : 1;
					uint8_t reserved : 3;
				}status_bit_field;
			} status_field;
			//reserved 3 bytes at the end (byte 5 - 7)
			uint8_t reserved_bytes[3];
			
		}data_byte_field;
		uint64_t data;
	}data_frame;
	
}charger_to_BMS_CAN_message;

void BMS_to_charger_message_init( float max_charging_voltage, float max_charging_current, charger_control ctrl, BMS_to_charger_CAN_message* msg);
bool BMS_to_charger_message_send(MCP_CAN* can, BMS_to_charger_CAN_message* msg);
void BMS_to_charger_message_receive(MCP_CAN* can, charger_to_BMS_CAN_message* msg);
void charger_to_BMS_data_parser(charger_to_BMS_CAN_message* msg);


#endif