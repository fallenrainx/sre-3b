
//CAN manager header file
#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#include <SPI.h>
#include "MCP_CAN.h"
#include "CAN_data_structs.h"
#include "BMS_dataTypes.h"

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

typedef struct
{
	uint16_t message_id;
	uint8_t data_length_byte;
	uint8_t data[8]; //data frame
}BMS_standard_message;

class CAN_manager_singleton
{
public:
  static CAN_manager_singleton& getInstance();
  bool CAN_manager_init_bus(MCP_CAN& CAN_BUS, message_type type);
  void BMS_to_charger_set_max_charging_parameters(float max_charging_voltage, float max_charging_current);
  void BMS_to_charger_disable_charger();
  void BMS_to_charger_enable_charger();
  bool BMS_to_charger_message_send(MCP_CAN& CAN_BUS); //returns 0 if message is successfully sent
  void charger_to_BMS_message_receive(MCP_CAN& CAN_BUS);
  BMS_to_charger_CAN_message& get_BMS_to_charger_CAN_message(); //access to private variable
  charger_to_BMS_CAN_message& get_charger_to_BMS_CAN_message(); //access to private variable

	//standard traction pack message send
	bool BMS_to_car_message_send(MCP_CAN& CAN_BUS, standard_traction_pack_message& stpm); //return 0 if successful

private:
  static CAN_manager_singleton * instance;
  BMS_to_charger_CAN_message BmsToCharger_message;
  charger_to_BMS_CAN_message ChargerToBms_message;
//	BMS_standard_message bms_standard_msg;
  CAN_manager_singleton();
};

#endif
