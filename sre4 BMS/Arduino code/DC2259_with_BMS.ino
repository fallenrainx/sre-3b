/*!
Linear Technology DC2259 Demonstration Board
LTC6811-1: Battery stack monitor


@verbatim

NOTES
 Setup:
   Set the terminal baud rate to 115200 and select the newline terminator.
   Ensure all jumpers on the demo board are installed in their default positions from the factory.
   Refer to Demo Manual DC2259.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0)
 binary  : B10000000000
 float   : 1024.0
@endverbatim

http://www.linear.com/product/LTC6811-1

http://www.linear.com/product/LTC6811-1#demoboards

Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

/*! @file
    @ingroup LTC6811-1
*/

#include "BMS.h"
#include "CAN_manager.h"
#define CAN_BUS_INT  2

MCP_CAN CAN_BUS(9);
CAN_manager_singleton &CAN_manager = CAN_manager_singleton::getInstance();
BMS_singleton &BMS = BMS_singleton::getInstance();
 
void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 500KHz SPI Clock

  if (!CAN_manager.CAN_manager_init_bus(CAN_BUS))//init bus with 500kHZ bus speed
    Serial.println(F("MCP2515 Initialized Successfully!"));
  else
    Serial.println(F("Error Initializing MCP2515...")); 

  BMS.write_to_charger(CAN_manager, 0, 0); //set charger max voltage and current to zero at first
  BMS.disable_charger(CAN_manager); //disable charger
  BMS.set_battery_state(charging); 
  digitalWrite(9, HIGH);
}

void loop()
{
    BMS.read_all_cell_groups_voltage();
    //BMS.read_all_cell_groups_temp();
    BMS.read_cell_groups_temp(0);
    BMS.print_all_cell_groups_voltage_and_temp();
   // BMS.monitor_all_cell_groups_voltage_and_temp(CAN_manager);
    
  // send data on CAN bus
  /*bool is_send_failed = CAN_manager.BMS_to_charger_message_send(CAN_BUS);
  if (is_send_failed == false) {
    Serial.println(F("Message Sent Successfully to the charger!"));
  } else {
    Serial.println(F("Error Sending Message..."));
  }

  //receive data from the charger if data is ready
  CAN_manager.charger_to_BMS_message_receive(CAN_BUS);

  //obtain values from can manager:
  BMS.read_from_charger(CAN_manager);
  Serial.print(F("BMS to Charger voltage: "));
  Serial.println(BMS.get_BMS_command_voltage());
  Serial.print(F("BMS to Charger current: "));
  Serial.println(BMS.get_BMS_command_current());

  Serial.print(F("Incoming voltage from charger: "));
  Serial.println(BMS.get_charger_output_voltage());
  Serial.print(F("Incoming current from charger: "));
  Serial.println(BMS.get_charger_output_current());

 if(BMS.is_hardware_failed_charger_flag())
  Serial.println(F("Hardware Failure Detected"));

 if(BMS.is_over_temperature_charger_flag())
  Serial.println(F("Over Temperature Detected"));

  if(BMS.is_input_voltage_wrong_charger_flag())
  Serial.println(F("Abnormal Input Voltage"));

 if(BMS.is_battery_voltage_detected_charger_flag())
  Serial.println(F("Battery Voltage Not Normal, Charger Stopped"));
  
  if(BMS.is_communication_time_out_charger_flag())
  Serial.println(F("Communication Timed Out"));
  //Serial.println(BMS.get_charger_flags()); 
  
 */
  delay(1000);
}


