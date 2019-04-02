
//BMS_singleton header file
#ifndef BMS_H_
#define BMS_H_

#include "CAN_manager.h"

class BMS_singleton
{
public:
  static BMS_singleton& getInstance();

  //BMS_singleton read from CAN_manager_singleton and store data in member variables
  //this is merely accessing charger information stored in CAN_manager_singleton object
  //it does not initiate new receiving of messages from charger,  which is controlled by
  //the CAN_manager_singleton object
  void read_from_charger(CAN_manager_singleton& CAN_manager);

  //BMS_singleton write to CAN_manager_singleton for the purpose of sending new data to charger
  //this is merely updating member variables inside CAN_manager_singleton object
  //it does not initiate new sending of messages to charger, which is controlled by
  //the CAN_manager_singleton object
  void write_to_charger(CAN_manager_singleton& CAN_manager, float max_charging_voltage, float max_charging_current);
  void disable_charger(CAN_manager_singleton& CAN_manager);
  void enable_charger(CAN_manager_singleton& CAN_manager);
  float get_charger_output_voltage();
  float get_charger_output_current();

  //member functions allowing the charger flags to be obtained
  uint8_t get_charger_flags();
  bool is_hardware_failed_charger_flag();
  bool is_over_temperature_charger_flag();
  bool is_AC_voltage_in_range_charger_flag();
  bool is_DC_voltage_in_range_charger_flag();
  bool is_communication_time_out_charger_flag();

private:
  static BMS_singleton * instance;
  float charger_max_voltage_V; //voltage data send to charger
  float charger_max_current_A; //current data send to charger
  float charger_output_voltage_V; //voltage data received from charger
  float charger_output_current_A; //voltage data received from charger
  uint8_t charger_flags;

  BMS_singleton(); //private constructor for singleton class
};

#endif
