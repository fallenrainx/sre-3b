
//BMS_singleton header file
#ifndef BMS_H_
#define BMS_H_

#include "CAN_manager.h"
#include "BMS_dataTypes.h"

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
  float get_BMS_command_voltage();
  float get_BMS_command_current();

  //member functions allowing the charger flags to be obtained
  uint8_t get_charger_flags();
  bool is_hardware_failed_charger_flag();
  bool is_over_temperature_charger_flag();
  bool is_input_voltage_wrong_charger_flag();
  bool is_battery_voltage_detected_charger_flag();
  bool is_communication_time_out_charger_flag();

  //battery state setting and getter
  void set_battery_state(battery_state state);
  battery_state get_battery_state();

  //member functions for 6811 battery monitor
  void read_all_cell_groups_voltage();
  void read_all_cell_groups_temp();
  void read_cell_groups_temp(int cell_group_number); //reads temperature of only one cell group. group number starts with 0 to 8
  float get_individual_cell_group_voltage(int module_number, int cell_group_number); //number start @ 0
  float get_individual_cell_group_temp(int module_number, int cell_group_number); //number start @ 0
  void print_all_cell_groups_voltage_and_temp();

  /*this function should parse through all cell group voltages and flag
  ones with voltage at 4.1V. As soon as any cell group hits 4.1V,
  charging will be slowed to 50mA*8 (end current). Charging will stop when any
  cell group hits 4.2V. This function should also parse through all cell group temperatures and flag
  ones with temperature equal or exceeding the temperature threshold stated.
  charging/discharging will stop once the flag is set */
  void monitor_all_cell_groups_voltage_and_temp(CAN_manager_singleton& CAN_manager);

  cell_asic bms_ic[TOTAL_IC];

private:
  // variables related to singleton object
  static BMS_singleton * instance;
  BMS_singleton(); //private constructor for singleton class

  //Charger-related member variables
  float charger_max_voltage_V; //voltage data send to charger
  float charger_max_current_A; //current data send to charger
  float charger_output_voltage_V; //voltage data received from charger
  float charger_output_current_A; //voltage data received from charger
  uint8_t charger_flags;
  battery_state current_state_of_battery; //either on the cart (charging) or in the car (discharging)

  //6811 battery monitor-related member variables
  cell_group battery_monitor[8][9]; //8 modules, with 9 cell groups in each module

  ///function that converts COMM reading to actual temp in degree C
  //This function is taken from BMS_temp.h, by Alex and Tim
  float temp_conversion(uint16_t comm_reading);
};

#endif
