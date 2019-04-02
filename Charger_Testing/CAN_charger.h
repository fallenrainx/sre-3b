//header file to send and receive CAN messages between the BMS and the charger
//By Nuoya Xie

#ifndef CAN_CHARGER_H_
#define CAN_CHARGER_H_

#include <mcp_can.h>
#include "CAN_data_structs.h"


//This struct contains messages send to charger
typedef struct
{
	uint32_t message_id; //should be 0x1806E5F4 for the first charger
	uint8_t data_length_byte; //should be 8
	data_frame_BMS data;//data frame (8 bytes)
}BMS_to_charger_CAN_message;


//this struct contains messages from the charger
typedef struct
{
	uint32_t message_id; //should be 0x18FF50E5 for BMS
	uint8_t data_length_byte; //should be 8
	data_frame_charger data; //data frame (8 bytes)
}charger_to_BMS_CAN_message;

void BMS_to_charger_message_init( float max_charging_voltage, float max_charging_current, charger_control ctrl, BMS_to_charger_CAN_message* msg);
bool BMS_to_charger_message_send(MCP_CAN* can, BMS_to_charger_CAN_message* msg);
void charger_to_BMS_message_receive(MCP_CAN* can, charger_to_BMS_CAN_message* msg);
void charger_to_BMS_data_parser(charger_to_BMS_CAN_message* msg);


#endif
