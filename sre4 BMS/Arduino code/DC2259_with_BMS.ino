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

MCP_CAN CAN_BUS(CAN_TRANSCEIVER_PIN);
CAN_manager_singleton &CAN_manager = CAN_manager_singleton::getInstance();
BMS_singleton &BMS = BMS_singleton::getInstance();
int loop_number = 0;

void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 500KHz SPI Clock

  if (!CAN_manager.CAN_manager_init_bus(CAN_BUS))//init bus with 500kHZ bus speed
    Serial.println(F("MCP2515 Initialized Successfully!"));
  else
    Serial.println(F("Error Initializing MCP2515..."));
  delay(5000); //wait 5 seconds
  BMS.write_to_charger(CAN_manager, 0, 0); //set charger max voltage and current to zero at first
  BMS.disable_charger(CAN_manager); //disable charger
  //BMS.set_battery_state(charging);

  //setting configuration register for 6811. Somehow when this is inside the bms class it freezes the whole program
  wakeup_sleep(TOTAL_IC);
  LTC6811_wrcfg(TOTAL_IC, BMS.bms_ic);
  digitalWrite(BMS_FAULT_PIN, HIGH); //disable BMS fault pin
}

void loop()
{
  Serial.print(F("BMS state: "));
  Serial.println(BMS.get_battery_state()); 
  
  bool is_send_failed = 0;
  //read the CAN bus to activate various states in the switch statement below
  BMS.read_CAN_bus(CAN_manager, CAN_BUS);

  //read temp, voltage, and current of the battery cells
  BMS.read_all_cell_groups_voltage();
  BMS.read_current_sensor();
  if (loop_number % 5 == 0) //read temp every 5th loop
  {
    BMS.read_all_cell_groups_temp();
  }
   if(BMS.stpm.BMS_fault_flag) //if BMS has a fault
  {
    digitalWrite(BMS_FAULT_PIN, LOW); //TODO: active low or high?
    BMS.set_battery_state(idle); //if fault occurs, do not do anything anymore
  }

  //The big FSM to control actions
  switch (BMS.get_battery_state())
  {
    case testing: //state to test BSPD
      //continuing looking on the bus to see if any command regarding BSPD is sent
      BMS.read_CAN_bus(CAN_manager, CAN_BUS);
      break;

    case discharging:
      //calculate allowable current based on current state of battery
      if(BMS.stpm.cell_voltage_below_2_6_flag) //if voltage falls below 2.6V
      {
        //TODO: allow smaller amount of current here?
      }
      else if(BMS.stpm.battery_temp_too_high_for_discharging_flag) //if battery temp reaches beyong the spec temp
      {
        //TODO: do something here to lower current draw?
      }
      else
      {
        //TODO: allow max current draw?
      }
      //state switching is taken care of in the BMS.read_current_sensor() function, when current is read.
      break;

    case charging:
      //checking pack voltage and temp
      if (BMS.stpm.cell_voltage_reach_4_16_flag || BMS.stpm.battery_temp_too_high_for_charging_flag || BMS.stpm.total_voltage > TOTAL_BATTERY_VOLTAGE) //if any cell_group reached 4.2 V or overheats while charging
      {
        BMS.write_to_charger(CAN_manager, 0, 0);//disable charger
        BMS.disable_charger(CAN_manager);//stop charging
        BMS.set_battery_state(discharging);
        Serial.println(F("Full cell or over temperature or total voltage achieved, charging is stopped..."));
      }
      else if (BMS.stpm.cell_voltage_reach_4_flag) //if any cell_group reached 4 V  288/72 = 4V
      {
        BMS.write_to_charger(CAN_manager, TOTAL_BATTERY_VOLTAGE, CHARGER_OUTPUT_CURRENT_SPEC);//get batteries to higher voltage
        BMS.enable_charger(CAN_manager);
        Serial.println(F("Cell at threshold voltage 4.0V detected. increase charger voltage to 299.7V"));
      }
      else if (BMS.stpm.cell_voltage_too_low_flag || BMS.stpm.BMS_fault_flag || BMS.stpm.cell_voltage_too_high_flag )
      {
        BMS.write_to_charger(CAN_manager, 0, 0);//disable charger
        BMS.disable_charger(CAN_manager);//stop charging
        BMS.set_battery_state(discharging);
        Serial.println(F("battery voltage below 2.6 or BMS fault detected"));
      }
      else //if there is no issue
      {
        Serial.println(F("No battery over voltage threshold or over temperature detected, everything fine"));
        BMS.write_to_charger(CAN_manager, CHARGER_OUTPUT_VOLTAGE_SPEC, CHARGER_OUTPUT_CURRENT_SPEC); //set charger max voltage and current: 299.5V, 7.5A
        BMS.enable_charger(CAN_manager); //enable charger
      }
      // send data on CAN bus
      is_send_failed = CAN_manager.BMS_to_charger_message_send(CAN_BUS);
      if (is_send_failed == false) {
        Serial.println(F("Message Sent Successfully to the charger!"));
      } else {
        Serial.println(F("Error Sending Message..."));
      }
      //receive data from the charger if data is ready
      CAN_manager.charger_to_BMS_message_receive(CAN_BUS);

      //obtain values from can manager:
      BMS.read_from_charger(CAN_manager);
      Serial.print(F("BMS to Charger voltage, current: "));
      Serial.println(BMS.get_BMS_command_voltage());
      Serial.print(F("BMS to Charger current: "));
      Serial.println(BMS.get_BMS_command_current());

      Serial.print(F("Incoming voltage from charger: "));
      Serial.println(BMS.get_charger_output_voltage());
      Serial.print(F("Incoming current from charger: "));
      Serial.println(BMS.get_charger_output_current());

      if (BMS.is_hardware_failed_charger_flag())
        Serial.println(F("Hardware Failure Detected"));

      if (BMS.is_over_temperature_charger_flag())
        Serial.println(F("Over Temperature Detected"));

      if (BMS.is_input_voltage_wrong_charger_flag())
        Serial.println(F("Abnormal Input Voltage"));

      if (BMS.is_battery_voltage_detected_charger_flag())
        Serial.println(F("Battery Voltage Not Normal, Charger Stopped"));

      if (BMS.is_communication_time_out_charger_flag())
        Serial.println(F("Communication Timed Out"));
      break;

    case regening:
    //calculate allowable regen current based on current state of battery
      if(BMS.stpm.cell_voltage_reach_4_flag) //if voltage of any particular cell is above 4V
      {
        //TODO: allow smaller amount of current here?
      }
      else if (BMS.stpm.cell_voltage_reach_4_16_flag) //if voltage of any particular cell is above 4.16V
      {
        //TODO: allow smaller amount of current here?
      }
      else if(BMS.stpm.battery_temp_too_high_for_charging_flag) //if battery temp reaches beyong the spec temp
      {
        //TODO: do something here to lower current draw?
      }
      else
      {
        //TODO: allow max current regen?
      }
    //state switching is taken care of in the BMS.read_current_sensor() function, when current is read.
      break;
    default: //idle state
    //nothing to do
    break;
  }

  //send Standard traction pack message onto the CAN bus
  BMS.send_stpm(CAN_manager, CAN_BUS);

  unsigned long current_time = millis();
  if(current_time - BMS.can_msg_time_track > 10000) //if last msg from charger was more than 10 seconds ago
  {
    BMS.set_battery_state(idle); //go back to idle state
  }

  ++loop_number;
  delay(1000);
}


