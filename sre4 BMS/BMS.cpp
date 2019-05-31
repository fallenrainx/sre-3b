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

  stpm = {0};

  //set everything to zero at first
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int current_cell= 0; current_cell < TOTAL_CELL_GROUP; current_cell++)
    {
      battery_monitor[current_ic][current_cell].cell_group_voltage = 0;
      battery_monitor[current_ic][current_cell].cell_group_temp = 0;
    }//for
  }//for

  //functions needed for the 6811
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
  bool gpio_reg[5] = {1, 1, 1, 1, 1};
  LTC681x_set_cfgr_gpio(TOTAL_IC,bms_ic,gpio_reg);
}

void BMS_singleton::send_stpm(CAN_manager_singleton& CAN_manager, MCP_CAN& CAN_BUS)
{
  //for debug purposes: set values for stpm. will be deleted once real values are available
  stpm.SoC = 20; //20%
  stpm.max_discharge_current = 42; //42A
  stpm.max_regen_current = 320.1; //320.1A
  stpm.battery_temp = 35.5;
  stpm.flag = 0x19; //0b0001_1001
  CAN_manager.BMS_to_car_message_send(CAN_BUS, stpm);
}

//at the moment only 2 messages are received - from PCANView to initiate BSPD testing
//and values to toggle the DAC for BSPD testing
void BMS_singleton::read_CAN_bus(CAN_manager_singleton& CAN_manager, MCP_CAN& CAN_BUS)
{
  //temp storage for message received
  uint32_t msg_ID = 0;
  uint8_t msg_length = 0;
  uint8_t buffer[8] = {0};
  CAN_manager.read_CAN_bus_std_format(CAN_BUS, msg_ID, msg_length, buffer);

  //checking if bspd testing
  switch(msg_ID)
  {
    case BSPD_ON_OFF_MID:
    if(msg_length == 1 && buffer[0] == 0x1) //bspd testing on
    {
      set_battery_state(testing);
      //TODO: flip the gpio on BMS to low for cs for DAC pin
      //DigitalWrite(CSPIN, LOW);
    }
    else if (msg_length == 1 && buffer[0] == 0x0) //bspd testing off
    {
      set_battery_state(discharging); //TODO: is this right?
      //TODO: flip the gpio on BMS to high to disable DAC pin
      //DigitalWrite(CSPIN, HIGH);
    }
    break;

    case BSPD_SET_CURRENT_MID:
      if(current_state_of_battery == testing)
      {
        set_BSPD_testing_current(buffer);
      }
    break;
    default:
    //no received message, do nothing
    break;
  }
}

void BMS_singleton::set_BSPD_testing_current(uint8_t buffer[8])
{
  float testing_current = buffer[0] / 10.0;
  float DAC_voltage = testing_current * 0.15;
  //controlling for allowable range of DAC (0 - 4.5V)
  if(DAC_voltage < 0) DAC_voltage = 0;
  if(DAC_voltage > 4.5) DAC_voltage = 4.5;

  uint16_t DAC_mapped_voltage = (int) (DAC_voltage / 5.0 * 4096); //0 - 4095 is 12 bits
  uint8_t PDs = 0x0; //TODO: is this right?
  uint8_t temp_buffer[2] = {0};
  temp_buffer[0] = (PDs << 4) | (DAC_mapped_voltage >> 8);
  temp_buffer[1] = DAC_mapped_voltage & 0xff; //8 least sig. bits
  write_to_i2c(BSPD_DAC_5321_ADDRESS, 2, temp_buffer);

}

void BMS_singleton::write_to_i2c(uint8_t slave_address, uint8_t bytes_to_write, uint8_t value_to_write[8])
{
  //need Wire.begin();
  Wire.beginTransmission(slave_address); // transmit to device #8
  for(int i = 0; i < bytes_to_write; i++)
  {
    Wire.write(value_to_write[i]);              // sends one byte
  }
  Wire.endTransmission();    // stop transmitting
}

void BMS_singleton::read_from_i2c(uint8_t slave_address, uint8_t byte_to_read, uint32_t& value_from_reading)
{
  //need Wire.begin();
  Wire.requestFrom(slave_address, byte_to_read);    // request 6 bytes from slave device #8
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    value_from_reading = value_from_reading << 8;
    Serial.print(c, HEX);         // print the character
    value_from_reading |= c;
  }
    Wire.endTransmission();    // stop transmitting
}

void BMS_singleton::write_repeat_read_i2c(uint8_t slave_address, uint8_t value_to_write, uint8_t byte_to_read, uint32_t& value_from_reading)
{
  value_from_reading = 0;
  //need Wire.begin();
  Wire.beginTransmission(slave_address);     // Slave addres
  Wire.write(value_to_write); //0x0 signifies default conversion rate of 60Hz
  Wire.endTransmission(false); // Sending data in a burst and doing a repeated start
  Wire.requestFrom(slave_address, byte_to_read);   // Requesting 16-bit of data
  while(Wire.available())
  {
    byte c = Wire.read(); // receive a byte as character
    value_from_reading = value_from_reading << 8;
    Serial.print(c, HEX);         // print the character
    value_from_reading |= c;
  }
    Wire.endTransmission();    // stop transmitting
}

void BMS_singleton::read_from_charger(CAN_manager_singleton& CAN_manager)
{
  charger_to_BMS_CAN_message msg = CAN_manager.get_charger_to_BMS_CAN_message();

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
void BMS_singleton::write_to_charger(CAN_manager_singleton& CAN_manager, float max_charging_voltage, float max_charging_current)
{
  CAN_manager.BMS_to_charger_set_max_charging_parameters(max_charging_voltage, max_charging_current);
  charger_max_voltage_V = max_charging_voltage;
  charger_max_current_A = max_charging_current;
}

void BMS_singleton::disable_charger(CAN_manager_singleton& CAN_manager)
{
  CAN_manager.BMS_to_charger_disable_charger();
}
void BMS_singleton::enable_charger(CAN_manager_singleton& CAN_manager)
{
  CAN_manager.BMS_to_charger_enable_charger();
}

float BMS_singleton::get_charger_output_voltage()
{
  return charger_output_voltage_V;
}
float BMS_singleton::get_charger_output_current()
{
  return charger_output_current_A;
}

float BMS_singleton::get_BMS_command_voltage()
{
  return charger_max_voltage_V;
}
float BMS_singleton::get_BMS_command_current()
{
  return charger_max_current_A;
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

bool BMS_singleton::is_input_voltage_wrong_charger_flag()
{
  return (charger_flags & (0x4));
}

bool BMS_singleton::is_battery_voltage_detected_charger_flag()
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
  Serial.println(F("entering read all voltage function"));
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
  Serial.println(F("entering read temp function"));
    bool gpio_reg[5] = {1, 1, 1, 1, 1};
    wakeup_sleep(TOTAL_IC);
    LTC681x_set_cfgr_gpio(TOTAL_IC,bms_ic,gpio_reg);
  uint8_t slave_address = ((TEMP_SENSORS_BASE_ADDRESS + cell_group_number) << 1) | 0x1; //decimal.  0x14(hex),  0010100(binary), according to Alex
  for (int current_ic= 0; current_ic < TOTAL_IC; current_ic++)
    {
      temp_fill_COMM_reg(current_ic, slave_address);

    /*Serial.print(F("\nBYTES:"));
      for(int i = 0; i < 6; i++)
      {
        Serial.println(bms_ic[current_ic].com.tx_data[i], HEX);
      }
      Serial.println(""); */
      //Serial.println(F("sending WRCOMM"));
      LTC6811_wrcomm(TOTAL_IC, bms_ic);//writes to the comm registers
      //Serial.println(F("sending STCOMM"));
      LTC6811_stcomm(); //shift comm bytes to the bus
      //Serial.println(F("reading RDCOMM"));
      uint8_t pec_error = LTC6811_rdcomm(TOTAL_IC, bms_ic); //read comm registers for temp values

      /*for(int i = 0; i < 8; i++)
      {
        Serial.println(bms_ic[current_ic].com.rx_data[i], HEX);
      }*/
      //extract data1 and data2 to construct reading from ltc2451
      uint8_t D1 = ((bms_ic[current_ic].com.rx_data[2] & 0xf) << 4) | (bms_ic[current_ic].com.rx_data[3] >> 4);
      uint8_t D2 = ((bms_ic[current_ic].com.rx_data[4] & 0xf) << 4) | (bms_ic[current_ic].com.rx_data[5] >> 4);
      uint16_t comm_reading = (D1 << 8) | (D2);

      //the voltage below is just for debug. The actual sensor reading should be converted to temperature
      Serial.print("comm_reading value: ");
      Serial.println(comm_reading, HEX);
      float voltage = comm_reading * 5.0 / 65535;
      Serial.print("voltage read: ");
      Serial.println(voltage);

      //temperature calculation
      //float cell_temp = temp_conversion(comm_reading);
      //battery_monitor[current_ic][cell_group_number].cell_group_temp = cell_temp;
      /*Serial.print("module ");
      Serial.print(current_ic);
      Serial.print(" cell group ");
      Serial.print(cell_group_number);
      Serial.print(" temperature ");
      Serial.println(cell_temp);*/
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

void BMS_singleton::temp_fill_COMM_reg(uint8_t current_ic, uint8_t slave_address)
{
  //bms_ic[current_ic].com.tx_data[0] = 0x62;
  //bms_ic[current_ic].com.tx_data[1] = 0x98;
  bms_ic[current_ic].com.tx_data[0] = (0x6 << 4) | (slave_address >> 4);
  bms_ic[current_ic].com.tx_data[1] = ((slave_address & 0xf) << 4) | 0x8;
  bms_ic[current_ic].com.tx_data[2] = 0x0f;
  bms_ic[current_ic].com.tx_data[3] = 0xf0;
  bms_ic[current_ic].com.tx_data[4] = 0x0f;
  bms_ic[current_ic].com.tx_data[5] = 0xf9;
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
    Serial.print(current_ic,DEC);
    Serial.print(F(", "));
    for (int current_cell_group=0; current_cell_group<TOTAL_CELL_GROUP; current_cell_group++)
    {
      Serial.print(F(" Cell"));
      Serial.print(current_cell_group,DEC);
      Serial.print(F(" V:"));
      Serial.print(battery_monitor[current_ic][current_cell_group].cell_group_voltage);
      Serial.print(F(" T:"));
      Serial.print(battery_monitor[current_ic][current_cell_group].cell_group_temp);
      Serial.print(F(","));
    }
    Serial.println();
  }
  Serial.println();
}

void BMS_singleton::monitor_all_cell_groups_voltage_and_temp(CAN_manager_singleton& CAN_manager)
{
  bool is_any_cell_group_battery_full = false;
  bool is_any_cell_group_over_threshold_voltage_4_16 = false;
  bool is_any_cell_group_over_threshold_temp = false;
  float total_voltage = 0;

  //loop through the cell groups
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    for (int current_cell= 0; current_cell < TOTAL_CELL_GROUP; current_cell++)
    {
      //checking voltage
      float current_voltage = battery_monitor[current_ic][current_cell].cell_group_voltage;
      total_voltage += current_voltage; //add current voltage to total voltage
      if(current_voltage >= CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V) //if the cell group voltage actually reached 4.2
      {
        battery_monitor[current_ic][current_cell].cell_group_battery_full_flag = true; //set voltage full flag
        battery_monitor[current_ic][current_cell].over_voltage_threshold_flag = true; //set overvoltage flag
        is_any_cell_group_battery_full = true;
      }

      else if (current_voltage >= CELL_OVER_VOLTAGE_THRESHOLD_V) //if any voltage >= 4.1v
      {
        battery_monitor[current_ic][current_cell].cell_group_battery_full_flag = false; //clear voltage full flag
        battery_monitor[current_ic][current_cell].over_voltage_threshold_flag = true; //set overvoltage flag
        is_any_cell_group_over_threshold_voltage_4_16 = true;
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
      write_to_charger(CAN_manager, 0, 0);//disable charger
      disable_charger(CAN_manager);//stop charging
      Serial.println(F("Full battery or over temperature detected, charging is stopped..."));
    }
    else if (current_state_of_battery == discharging)
    {
      //need to do temp scheduling
      Serial.println(F("Full battery or over temperature detected"));
    }
  }
  else if (is_any_cell_group_over_threshold_voltage_4_16 == true || total_voltage >299.52) //if any cell_group reached 4.1 V or total voltage reaches 299.52
  {
    if(current_state_of_battery == charging)//if in charging state
    {
      write_to_charger(CAN_manager, 0, 0);//disable charger
      disable_charger(CAN_manager);
      Serial.println(F("battery at threshold voltage detected. disable charger..."));
    }
  }
  else //if there is no issue
  {
      Serial.println(F("no battery over voltage threshold or over temperature detected, everything fine"));
      if(current_state_of_battery == charging)
      {
        Serial.println(F("charging at 288V and 7.5A"));
        write_to_charger(CAN_manager, 288, 7.5); //set charger max voltage and current: 299.5V, 7.5A
        enable_charger(CAN_manager); //enable charger
      }
  }
}
