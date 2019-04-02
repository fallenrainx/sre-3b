//cpp file for BMS_singleton.h

#include "BMS.h"

BMS_singleton* BMS_singleton::instance = NULL;

static BMS_singleton& BMS_singleton::getInstance()
{
  if (instance == NULL)
  {
    instance = new BMS_singleton();
  }
  return *instance;
}

BMS_singleton::BMS_singleton()
{
  charger_max_voltage_V = 0; //voltage data send to charger
  charger_max_current_A = 0; //current data send to charger
  charger_output_voltage_V = 0; //voltage data received from charger
  charger_output_current_A = 0; //voltage data received from charger
  charger_flags = 0;
}

void BMS_singleton::read_from_charger(CAN_manager_singleton& can_manager)
{
  charger_to_BMS_CAN_message msg = can_manager.get_charger_to_BMS_CAN_message();

  //obtaining voltage value
  uint8_t voltageMSB = msg.data.voltage_from_charger_MSB;
  uint8_t voltageLSB = msg.data.voltage_from_charger_LSB;
  charger_output_voltage_V = ((voltageMSB << 8) | voltageLSB) * 0.1;

//obtaining current values
uint8_t currentMSB = msg.data.current_from_charger_MSB;
uint8_t currentLSB = msg.data.current_from_charger_LSB;
charger_output_current_A = ((currentMSB << 8) | currentLSB) * 0.1;

//obtain status flag
charger_flags = msg.data.charger_status.status_byte;
}

//BMS_singleton write to charger max voltage and current values
void BMS_singleton::write_to_charger(CAN_manager_singleton& can_manager, float max_charging_voltage, float max_charging_current)
{
  can_manager.BMS_to_charger_set_max_charging_parameters(max_charging_voltage, max_charging_current);
  charger_max_voltage_V = max_charging_voltage;
  charger_max_current_A = max_charging_current;
}

void BMS_singleton::disable_charger(CAN_manager_singleton& can_manager)
{
  can_manager.BMS_to_charger_disable_charger();
}
void BMS_singleton::enable_charger(CAN_manager_singleton& can_manager)
{
  can_manager.BMS_to_charger_disable_charger();
}

float BMS_singleton::get_charger_output_voltage()
{
  return charger_output_voltage_V;
}
float BMS_singleton::get_charger_output_current()
{
  return charger_output_current_A;
}

//returns all the flags in one 8 bit variable
uint8_t BMS_singleton::get_charger_flags()
{
  return charger_flags;
}

//check if the hardware failure flag is set
bool BMS_singleton::is_hardware_failed_charger_flag()
{
 return (charger_flags & (0x1));
}

bool BMS_singleton::is_over_temperature_charger_flag()
{
return (charger_flags & (0x2));
}

bool BMS_singleton::is_AC_voltage_in_range_charger_flag()
{
return (charger_flags & (0x4));
}

bool BMS_singleton::is_DC_voltage_in_range_charger_flag()
{
return (charger_flags & (0x8));
}

bool BMS_singleton::is_communication_time_out_charger_flag()
{
return (charger_flags & (0x10));
}
