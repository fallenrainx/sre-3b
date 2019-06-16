
//BMS_singleton header file
#ifndef BMS_H_
#define BMS_H_

#include "CAN_manager.h"
#include "BMS_dataTypes.h"

class BMS_singleton
{
public:
  static BMS_singleton& getInstance();

  //standard traction pack message functions
  //this function calls the CAN_manager singleton class to send out the msg on the bus
void send_stpm(CAN_manager_singleton& CAN_manager, MCP_CAN& CAN_BUS);

//function to read from CAN bus.
//at the moment only 2 messages are received - from PCANView to initiate BSPD testing
//and values to toggle the DAC for BSPD testing
void read_CAN_bus(CAN_manager_singleton& CAN_manager, MCP_CAN& CAN_BUS);
void set_BSPD_testing_current(uint8_t buffer[8]);

  //BMS_singleton read from CAN_manager_singleton and store data in member variables
  //this is merely accessing charger information stored in CAN_manager_singleton object
  //it does not initiate new receiving of messages from charger,  which is controlled by
  //the CAN_manager_singleton object
  void read_from_charger(CAN_manager_singleton& CAN_manager);

  //i2c functions
  void write_to_i2c(uint8_t slave_address, uint8_t bytes_to_write, uint8_t value_to_write[8]);
  void read_from_i2c(uint8_t slave_address, uint8_t byte_to_read, uint32_t& value_from_reading);
  //this function read at most 4 bytes of data in one transation. if more are needed, change the parameter type
  void write_repeat_read_i2c(uint8_t slave_address, uint8_t value_to_write, uint8_t byte_to_read, uint32_t& value_from_reading);

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
  //float get_individual_cell_group_voltage(int module_number, int cell_group_number); //number start @ 0
  //float get_individual_cell_group_temp(int module_number, int cell_group_number); //number start @ 0
  //void print_all_cell_groups_voltage_and_temp();

  /*this function should parse through all cell group voltages and flag
  ones with voltage at 4.1V. As soon as any cell group hits 4.1V,
  charging will be slowed to 50mA*8 (end current). Charging will stop when any
  cell group hits 4.2V. This function should also parse through all cell group temperatures and flag
  ones with temperature equal or exceeding the temperature threshold stated.
  charging/discharging will stop once the flag is set */
  void monitor_all_cell_groups_voltage_and_temp(CAN_manager_singleton& CAN_manager);

  /* ltc2946 current monitor functions*/
  void configure_ltc2946();
  void read_current_sensor();

  cell_asic bms_ic[TOTAL_IC];

  //standard traction pack message to communicate with the VCU
  standard_traction_pack_message stpm; //how to initialize all to 0?

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
//  cell_group battery_monitor[8][9]; //8 modules, with 9 cell groups in each module

  ///function that converts COMM reading to actual temp in degree C
  //This function is taken from BMS_temp.h, by Alex and Tim
  float temp_conversion(uint16_t comm_reading);
  void temp_fill_COMM_reg(uint8_t current_ic, uint8_t slave_address);
};

#endif
