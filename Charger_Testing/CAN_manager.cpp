//cpp file for CAN_manager_singleton.h

#include "CAN_manager.h"

CAN_manager_singleton* CAN_manager_singleton::instance = NULL;

static CAN_manager_singleton&  CAN_manager_singleton::getInstance()
{
  if (instance == NULL)
  {
    instance = new CAN_manager_singleton();
  }
  return *instance;
}

CAN_manager_singleton::CAN_manager_singleton()
{
  BmsToCharger_message.message_id = 0x1806E5F4; //fixed
	BmsToCharger_message.data_length_byte = 8; //data = 8 bytes
	BmsToCharger_message.data = {0}; //clear all data in the data frame

  ChargerToBms_message.message_id = 0; //clear message ID
	ChargerToBms_message.data_length_byte = 0; //clear data length byte
	ChargerToBms_message.data = {0}; //clear all data in the data frame

  data_ready = false;
}

bool CAN_manager_singleton::CAN_manager_init_bus(MCP_CAN& CAN_BUS, int spd)
{
  CAN_bus_speed = spd;
  bool is_initialization_successful = false;
  if(CAN_bus_speed = 500000)
    is_initialization_successful =  CAN_BUS.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  else
    is_initialization_successful =  CAN_BUS.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);
CAN_BUS.setMode(MCP_NORMAL);
return is_initialization_successful;
}

void CAN_manager_singleton::set_CAN_bus_speed(int spd)
{
  CAN_bus_speed = spd;
}

void CAN_manager_singleton::BMS_to_charger_set_max_charging_parameters(float max_charging_voltage, float max_charging_current)
{
  int voltage = max_charging_voltage * 10;
  BmsToCharger_message.data.voltage_to_charger_LSB = voltage & 0xff;
  BmsToCharger_message.data.voltage_to_charger_MSB = (voltage >> 8) & 0xff;

  int current = max_charging_current * 10;
  BmsToCharger_message.data.current_to_charger_LSB = current & 0xff;
  BmsToCharger_message.data.current_to_charger_MSB = (current >> 8) & 0xff;
}
void CAN_manager_singleton::BMS_to_charger_disable_charger()
{
BmsToCharger_message.data.control = 1;
}
void CAN_manager_singleton::BMS_to_charger_enable_charger()
{
BmsToCharger_message.data.control = 0;
}

bool CAN_manager_singleton::BMS_to_charger_message_send(MCP_CAN& CAN_BUS)
{
return CAN_BUS.sendMsgBuf(BmsToCharger_message.message_id, BmsToCharger_message.data_length_byte, BmsToCharger_message.data.dword);
}

void CAN_manager_singleton::charger_to_BMS_message_receive(MCP_CAN& CAN_BUS)
{
  if (data_ready)
  {
    CAN_BUS.readMsgBuf(&(ChargerToBms_message.message_id), &(ChargerToBms_message.data_length_byte), ChargerToBms_message.data.dword);
    data_ready = false;
  }
}

BMS_to_charger_CAN_message& CAN_manager_singleton::get_BMS_to_charger_CAN_message()
{
  return BmsToCharger_message;
}

charger_to_BMS_CAN_message& CAN_manager_singleton::get_charger_to_BMS_CAN_message()
{
  return ChargerToBms_message;
}

void CAN_manager_singleton::set_data_ready_flag()
{
  data_ready = true;
}
void CAN_manager_singleton::clear_data_ready_flag()
{
  data_ready = false;
}

bool CAN_manager_singleton::is_data_ready()
{
  return data_ready;
}
