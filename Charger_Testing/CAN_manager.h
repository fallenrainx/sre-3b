
//CAN manager header file
#ifndef CAN_MANAGER_H_
#define CAN_MANAGER_H_

#include <SPI.h>
#include "MCP_CAN.h"
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

class CAN_manager_singleton
{
public:
  static CAN_manager_singleton& getInstance();
  bool CAN_manager_init_bus(MCP_CAN& CAN_BUS, int spd);
  void BMS_to_charger_set_max_charging_parameters(float max_charging_voltage, float max_charging_current);
  void BMS_to_charger_disable_charger();
  void BMS_to_charger_enable_charger();
  void set_CAN_bus_speed(int spd);
  bool BMS_to_charger_message_send(MCP_CAN& CAN_BUS);
  void charger_to_BMS_message_receive(MCP_CAN& CAN_BUS);
  BMS_to_charger_CAN_message& get_BMS_to_charger_CAN_message(); //access to private variable
  charger_to_BMS_CAN_message& get_charger_to_BMS_CAN_message(); //access to private variable
	void set_data_ready_flag();
	void clear_data_ready_flag();
	bool is_data_ready(); //returns the data_ready flag

private:
  static CAN_manager_singleton * instance;
  BMS_to_charger_CAN_message BmsToCharger_message;
  charger_to_BMS_CAN_message ChargerToBms_message;
	bool data_ready; //variable needed for receving message through use of interrupt. ISR sets this flag, flag is cleared right after receiving msg
  CAN_manager_singleton();

  int CAN_bus_speed;   //can only be 2 values: 500k or 1M

};

#endif
