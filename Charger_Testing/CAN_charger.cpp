//Cpp file to CAN_charger.h
//By Nuoya Xie

#include "CAN_charger.h"

void BMS_to_charger_message_init( float max_charging_voltage, float max_charging_current, charger_control ctrl, BMS_to_charger_CAN_message* msg)
{
	msg->message_id = 0x1806E5F4; //fixed
	msg->data_length_byte = 8; //data = 8 bytes
	msg->data = {0}; //clear all data in the data frame

	int voltage = max_charging_voltage * 10;
	msg->data.voltage_to_charger_LSB = voltage & 0xff;
	msg->data.voltage_to_charger_MSB = (voltage >> 8) & 0xff;

	int current = max_charging_current * 10;
	msg->data.current_to_charger_LSB = current & 0xff;
	msg->data.current_to_charger_MSB = (current >> 8) & 0xff;

	msg->data.control = ctrl;
}

bool BMS_to_charger_message_send(MCP_CAN* can, BMS_to_charger_CAN_message* msg)
{
	return can->sendMsgBuf(msg->message_id, msg->data_length_byte, msg->data.dword);
}

void charger_to_BMS_message_receive(MCP_CAN* can, charger_to_BMS_CAN_message* msg)
{
	msg->message_id = 0; //clear message ID
	msg->data_length_byte = 0; //clear data length byte
	msg->data = {0}; //clear all data in the data frame
	can->readMsgBuf(&(msg->message_id), &(msg->data_length_byte), msg->data.dword);
}

void charger_to_BMS_data_parser(charger_to_BMS_CAN_message* msg)
{
	//at the moment I don't know what to save the message information to
	//so this is blank for now
}
