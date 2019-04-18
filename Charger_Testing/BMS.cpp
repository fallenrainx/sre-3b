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

  //functions needed for the 6811
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);

  //write to configuration register for GPIO4 and GPIO5
  //stated in the data sheet that these two ports will have to set to 1 for i2c
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    bms_ic[current_ic].config.tx_data[0] |= 0xC0; //toggle gpio 4 and 5 to 1
  }
  LTC6811_wrcfg(TOTAL_IC, bms_ic);
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
  charger_flags = msg.data.status_byte;
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
  can_manager.BMS_to_charger_enable_charger();
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

void BMS_singleton::set_battery_state(battery_state state)
{
  current_state_of_battery = state;
}

battery_state BMS_singleton::get_battery_state()
{
  return current_state_of_battery;
}

//6811 member functions
//void BMS_singleton::read_cell_groups_voltage(cell_asic (&bms_ic)[TOTAL_IC])
void BMS_singleton::read_all_cell_groups_voltage()
{
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); // Start Cell ADC Measurement
  LTC6811_pollAdc(); //this function will block operation until ADC completes
  int8_t error = LTC6811_rdcv(0, TOTAL_IC, bms_ic); // read back all cell voltage registers
  if (error == -1) //checking for read error
  Serial.println(F("A PEC error was detected in the received data"));
  else //if there is no error
  {
    //move the data from bms_ic into our BMS data struct
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
    {
      for (int current_cell= 0; current_cell < TOTAL_CELL_GROUP; current_cell++)
      {
        battery_monitor[current_ic][current_cell].cell_group_voltage = bms_ic[current_ic].cells.c_codes[current_cell]*0.0001;
      }//for
    }//for
  }//else
}

//void BMS_singleton::read_cell_groups_temp(cell_asic (&bms_ic)[TOTAL_IC])
void BMS_singleton::read_all_cell_groups_temp()
{
  //TODO: read cell group temp
  for (int current_cell = 0 ; current_cell < TOTAL_CELL_GROUP; current_cell++)
  {
    read_cell_groups_temp(current_cell); //cell group starts @ 0
  }
}

//read temperature of just one node in each IC
void BMS_singleton::read_cell_groups_temp(int cell_group_number)
{
  for (int current_ic= 0; current_ic < TOTAL_IC; current_ic++)
    {
      uint8_t slave_address = ((TEMP_SENSORS_BASE_ADDRESS + cell_group_number) << 1) | 0x1; //decimal.  0x14(hex),  0010100(binary), according to Alex
      COMM_WR_REG reg;
      reg.fields.ICOM0 = 0x6; 							// send start
      reg.fields.D0 = slave_address; 	// to adc channel addr
      reg.fields.FCOM0 = 0x8; 							// send nack
      reg.fields.ICOM1 = 0x0;  							// send blank
      reg.fields.D1 =    0x00; 							// send empty byte
      reg.fields.FCOM1 = 0x8;  							// send NACK+STOP
      reg.fields.ICOM2 = 0x0;  							// no send
      reg.fields.D2 =    0x00; 							// empty byte
      reg.fields.FCOM2 = 0x9;  							// send ack
      memcpy(bms_ic[current_ic].com.tx_data, reg.bytes, 6);

      Serial.println("sending wrcomm");
      LTC6811_wrcomm(TOTAL_IC, bms_ic);//writes to the comm registers
      LTC6811_stcomm(); //shift comm bytes to the bus
      if(LTC6811_rdcomm(TOTAL_IC, bms_ic)) //read comm registers for temp values
      {
        Serial.println("PEC for rdcomm does not match \n"); //returns -1 if PEC error, 0 if no error
      }

      COMM_RD_REG rd_reg;   //obtain values from bms_ic array and store into battery_monitor arraay

      memcpy(rd_reg.bytes, bms_ic[current_ic].com.rx_data, 8);//load data from bms_ic
      uint16_t comm_reading = (rd_reg.fields.D1 << 8) | (rd_reg.fields.D2); //cast data bytes to uint16

      float cell_temp = temp_conversion(comm_reading);
      battery_monitor[current_ic][cell_group_number].cell_group_temp = cell_temp;
      Serial.print("module ");
      Serial.print(current_ic);
      Serial.print(" cell group ");
      Serial.print(cell_group_number);
      Serial.print(" temperature ");
      Serial.println(cell_temp);
    }//for current ic
}

float BMS_singleton::temp_conversion(uint16_t comm_reading)
{
  float resistance = SERIES_RESISTOR / (65535 / comm_reading - 1);
  float steinhart;
  steinhart = resistance / THERMISTOR_NOMINAL;     	// (R/Ro)
  steinhart = log(steinhart);                  		// ln(R/Ro)
  steinhart /= BETA_COEFFICIENT;                   		// 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMP_NOMINAL + 273.15); 	// + (1/To)
  steinhart = 1.0 / steinhart;                 		// Invert
  steinhart -= 273.15;                         		// convert to C
  return steinhart;
}

float BMS_singleton::get_individual_cell_group_voltage(int module_number, int cell_group_number)
{
  return battery_monitor[module_number][cell_group_number].cell_group_voltage;
}

float BMS_singleton::get_individual_cell_group_temp(int module_number, int cell_group_number)
{
  return battery_monitor[module_number][cell_group_number].cell_group_temp;
}

void BMS_singleton::print_all_cell_groups_voltage_and_temp()
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(", "));
    for (int current_cell_group=0; current_cell_group<TOTAL_CELL_GROUP; current_cell_group++)
    {
      Serial.print(F(" C"));
      Serial.print(current_cell_group+1,DEC);
      Serial.print(F(":"));
      Serial.print(battery_monitor[current_ic][current_cell_group].cell_group_voltage);
      Serial.print(F(","));
    }
    Serial.println();
  }
  Serial.println();
}

void BMS_singleton::monitor_all_cell_groups_voltage_and_temp(CAN_manager_singleton& CAN_manager)
{
  bool is_any_cell_group_battery_full = false;
  bool is_any_cell_group_over_threshold_voltage_4_1 = false;
  bool is_any_cell_group_over_threshold_temp = false;

  //loop through the cell groups
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int current_cell= 0; current_cell < TOTAL_CELL_GROUP; current_cell++)
    {
      //checking voltage
      float current_voltage = battery_monitor[current_ic][current_cell].cell_group_voltage;
      if(current_voltage >= (CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V - 0.05)) //if the cell group voltage actually reached 4.2-0.05v
      {
        battery_monitor[current_ic][current_cell].cell_group_battery_full_flag = true; //set voltage full flag
        battery_monitor[current_ic][current_cell].over_voltage_threshold_flag = true; //set overvoltage flag
        is_any_cell_group_battery_full = true;
      }

      else if (current_voltage >= CELL_OVER_VOLTAGE_THRESHOLD_V) //if any voltage >= 4.1v
      {
        battery_monitor[current_ic][current_cell].cell_group_battery_full_flag = false; //clear voltage full flag
        battery_monitor[current_ic][current_cell].over_voltage_threshold_flag = true; //set overvoltage flag
        is_any_cell_group_over_threshold_voltage_4_1 = true;
      }
      else //if voltage hasn't reach the threshold yet
      {
        battery_monitor[current_ic][current_cell].over_voltage_threshold_flag = false; //clear overvoltage flag
        battery_monitor[current_ic][current_cell].cell_group_battery_full_flag = false; //clear full flag
      }

      //checking temperature
      //if the battery is in charging state and cell group temp > charging temp threshold, or if battery is in discharging state and temp over threshold
      if((battery_monitor[current_ic][current_cell].cell_group_temp >= CELL_OVER_TEMP_THRESHOLD_CHARGE_C && current_state_of_battery == charging)
      || (battery_monitor[current_ic][current_cell].cell_group_temp >= CELL_OVER_TEMP_THRESHOLD_DISCHARGE_C && current_state_of_battery == discharging))
      {
        is_any_cell_group_over_threshold_temp = true;
        battery_monitor[current_ic][current_cell].over_temp_threshold_flag = true; //set over temp flag
      }
      else //if the cell group temp hasn't reach the threshold yet
      battery_monitor[current_ic][current_cell].over_temp_threshold_flag = false; //clear over temp flag
    }//inner for loop
  }//outer for loop

  //after looping through all cells:
  if (is_any_cell_group_battery_full == true || is_any_cell_group_over_threshold_temp == true) //if any cell_group reached 4.2 V or overheats while charging
  {
    if(current_state_of_battery == charging)
    {
      disable_charger(CAN_manager);//stop charging
      Serial.println(F("Full battery or over temperature detected, charging is stopped..."));
    }
    else if (current_state_of_battery == discharging)
    {
      //need to do temp scheduling
      Serial.println(F("Full battery or over temperature detected, opening HVIL..."));
    }
  }
  else if (is_any_cell_group_over_threshold_voltage_4_1 == true && current_state_of_battery == charging)//if any cell_group reached 4.1 V while charging
  {
    write_to_charger(CAN_manager, CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V*72, CELL_STANDARD_CHARGE_CONST_CURRENT_MA*8/1000);//activate slow charge
    enable_charger(CAN_manager);
    Serial.println(F("battery at threshold voltage detected. slow charging starts..."));
  }
  else
  Serial.println(F("no battery over voltage threshold or over temperature detected"));
}
