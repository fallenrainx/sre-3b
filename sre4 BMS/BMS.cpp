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
    Serial.println(F("right before wrcfg"));
  charger_max_voltage_V = 0; //voltage data send to charger
  charger_max_current_A = 0; //current data send to charger
  charger_output_voltage_V = 0; //voltage data received from charger
  charger_output_current_A = 0; //voltage data received from charger
  charger_flags = 0;

  stpm = {0};
  current_state_of_battery = discharging; //initialize to idle state at the beginning of the program

  //functions needed for the 6811
  //change is made in LTC681x.h to change gpio 3 to pull_down enabled (for address translator)
  LTC681x_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
  //wakeup_sleep(TOTAL_IC);
  //LTC6811_wrcfg(TOTAL_IC,bms_ic);
  //configure_ltc2946();
}

void BMS_singleton::send_stpm(CAN_manager_singleton& CAN_manager, MCP_CAN& CAN_BUS)
{
  //for debug purposes: set values for stpm. will be deleted once real values are available
  //stpm.SoC = 20; //20%
  stpm.max_discharge_current = 10; //42A
  stpm.max_regen_current = 10; //320.1A
  //stpm.battery_temp = 35.5;
  //stpm.flag = 0x19; //0b0001_1001*/
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

    case CHARGER_ADDRESS:
    //charger address detected, therefore state is charging
      set_battery_state(charging);
    default:
    //some message that BMS doesn't care about
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
  Wire.beginTransmission(slave_address); // transmit to device
  for(int i = 0; i < bytes_to_write; i++)
  {
    Wire.write(value_to_write[i]);              // sends one byte
  }
  Wire.endTransmission();    // stop transmitting
}

void BMS_singleton::read_from_i2c(uint8_t slave_address, uint8_t byte_to_read, uint32_t& value_from_reading)
{
  //need Wire.begin();
  Wire.requestFrom(slave_address, byte_to_read);    // request bytes
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
  Wire.requestFrom(slave_address, byte_to_read);   // Requesting 8-bit of data
  while(Wire.available())
  {
    byte c = Wire.read(); // receive a byte as character
    value_from_reading = value_from_reading << 8;
    Serial.print(c, HEX);         // print the character
    value_from_reading |= c;
  }
    Wire.endTransmission();    // stop transmitting
}

void BMS_singleton::configure_ltc2946()
{
  //CTRLA register
  //TODO: what is ADIN with respect to? GND or INTVCC? GND is by default. <- assume this for now
  //voltage selection? ->which one? Assume ADIN for now.
  //channel configuration? assume 100 for now...
  uint8_t temp_value_to_write[8] = {0};
  temp_value_to_write[0] = 0x0; //CTRLA register address
  temp_value_to_write[1] = B00010100; //value to write: 0b00010100
  write_to_i2c(CURRENT_SENSOR_2946_ADDRESS, 2, temp_value_to_write);

}
void BMS_singleton::read_current_sensor()
{
  //read power register (3 bytes) (address 05h to 07h)
  uint32_t power_value_MSB2 = 0;
  uint32_t power_value_MSB1 = 0;
  uint32_t power_value_LSB = 0;
  write_repeat_read_i2c(CURRENT_SENSOR_2946_ADDRESS, 0x5, 1, power_value_MSB2);
  write_repeat_read_i2c(CURRENT_SENSOR_2946_ADDRESS, 0x6, 1, power_value_MSB1);
  write_repeat_read_i2c(CURRENT_SENSOR_2946_ADDRESS, 0x7, 1, power_value_LSB);
  uint32_t power_value = ((power_value_MSB2 & 0xff) << 16) | ((power_value_MSB1 & 0xff) << 8) | (power_value_LSB & 0xff);
  stpm.battery_power = power_value;

  //read ADC vin register (2 bytes) (address 1Eh to 1Fh)
  uint32_t vin_voltage_MSB = 0;
  uint32_t vin_voltage_LSB = 0;
  write_repeat_read_i2c(CURRENT_SENSOR_2946_ADDRESS, 0x1E, 1, vin_voltage_MSB);
  write_repeat_read_i2c(CURRENT_SENSOR_2946_ADDRESS, 0x1F, 1, vin_voltage_LSB);
  uint32_t vin_voltage = ((vin_voltage_MSB & 0xff) << 8) | (vin_voltage_LSB & 0xff);

  //calculate current from power and voltage (I = P / V)
  float current = power_value * 1.0 / vin_voltage;
  stpm.total_current = current;
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
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); // Start Cell ADC Measurement
  LTC6811_pollAdc(); //this function will block operation until ADC completes
  int8_t error = LTC6811_rdcv(0, TOTAL_IC, bms_ic); // read back all cell voltage registers
  if (error == -1) //checking for read error
    Serial.println(F("A PEC error was detected in the received data"));
  else //if there is no error
  {
    bool is_any_cell_not_read = false;
    //reset all flag and total voltage
    stpm.cell_voltage_reach_4_flag = 0;
    stpm.cell_voltage_reach_4_16_flag = 0;
    stpm.total_voltage = 0;
    stpm.BMS_fault_flag = 0;
    stpm.cell_voltage_too_high_flag = 0;
    stpm.cell_voltage_too_low_flag = 0;
    stpm.cell_voltage_below_2_6_flag = 0;
    stpm.pack_voltage_high_flag = 0;
    stpm.pack_voltage_low_flag = 0;

    //move the data from bms_ic into our BMS data struct
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
    {
      Serial.print(F(" IC "));
      Serial.print(current_ic,DEC);
      Serial.print(F(", "));
      for (int current_cell= 0; current_cell < TOTAL_CELL_GROUP; current_cell++)
      {
        float voltage_value =  bms_ic[current_ic].cells.c_codes[current_cell]*0.0001;
        if(voltage_value > 6.50)  //when the cell voltage isn't available, it shows 6.55
          is_any_cell_not_read = true;
        else
          {
            stpm.total_voltage += voltage_value;
            Serial.print(F(" Cell"));
            Serial.print(current_cell,DEC);
            Serial.print(F(" V:"));
            Serial.print(voltage_value);
            if(voltage_value >= CELL_OVER_VOLTAGE_THRESHOLD_V) //if the cell group voltage actually reached 4.16
            {
              stpm.cell_voltage_reach_4_16_flag = 1;
              stpm.cell_voltage_reach_4_flag = 1;
            }
            else if (voltage_value >= CELL_ALMOST_FULL_VOLTAGE_V) //if any voltage >= 4v
            {
              stpm.cell_voltage_reach_4_flag = 1;
            }
            else if (voltage_value < CELL_UNDER_VOLTAGE_THRESHOLD_V && current_cell != 5) //5th cell is always 0
            {
              stpm.cell_voltage_below_2_6_flag = 1;
            }
            else if (current_cell != 5 && (voltage_value > CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V)) //if cell voltage ever gets above 4.2V and it isn't the 5th cell
            {
              stpm.cell_voltage_too_high_flag = 1;
              stpm.BMS_fault_flag = 1; //raise BMS fault
            }
            else if (current_cell != 5 && (voltage_value < CELL_ABSOLUTE_MINIMUM_VOLTAGE_V))//if cell voltage ever falls below 2.5V and it isn't the 5th cell
            {
              stpm.cell_voltage_too_low_flag = 1;
              stpm.BMS_fault_flag = 1; //raise BMS fault
            }
            else //if voltage hasn't reach the threshold yet
            {
            //everything fine
            }
          }//else voltage is legit
      }//for
      Serial.println();
    }//for
      Serial.println();
      if(stpm.total_voltage > TOTAL_BATTERY_VOLTAGE - 5 && is_any_cell_not_read == 0) //TODO: is this ok?
      {
        stpm.pack_voltage_high_flag = 1;
      }
      else if (stpm.total_voltage < 192 && is_any_cell_not_read == 0) //this is 10% battery //TODO: is this ok?
      {
        stpm.pack_voltage_low_flag = 1;
      }
      else //everything ok
        stpm.SoC = int ((stpm.total_voltage / TOTAL_BATTERY_VOLTAGE) * 100); //percentage in terms of int
  }//else
}

//void BMS_singleton::read_cell_groups_temp(cell_asic (&bms_ic)[TOTAL_IC])
void BMS_singleton::read_all_cell_groups_temp()
{
  //reset temp flag
  stpm.battery_temp = 0;
  stpm.battery_temp_too_high_for_discharging_flag = 0;
  stpm.battery_temp_too_high_for_charging_flag = 0;
  for (int current_cell = 0 ; current_cell < TOTAL_CELL_GROUP - 1; current_cell++) //only 9 ics in one module, so total_cell_group - 1
  {
    Serial.print(F(" Cell "));
    Serial.print(current_cell, DEC);
    Serial.print(F(", "));
    read_cell_groups_temp(current_cell); //cell group starts @ 0
  }
  Serial.println();
}

//read temperature of just one node in each IC
void BMS_singleton::read_cell_groups_temp(int cell_group_number)
{
  uint8_t slave_address = ((TEMP_SENSORS_BASE_ADDRESS + cell_group_number) << 1) | 0x1; //decimal.  0x14(hex),  0010100(binary), according to Alex
  for (int current_ic= 0; current_ic < TOTAL_IC; current_ic++)
    {
      temp_fill_COMM_reg(current_ic, slave_address);

    /*Serial.print(F("\nTX BYTES:\n"));
      for(int i = 0; i < 6; i++)
      {
        Serial.println(bms_ic[current_ic].com.tx_data[i], HEX);
      }
      Serial.println(""); */
      wakeup_sleep(TOTAL_IC);
      LTC6811_wrcomm(TOTAL_IC, bms_ic);//writes to the comm registers
      LTC6811_stcomm(); //shift comm bytes to the bus
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
      float voltage = comm_reading * 5.0 / 65535;
      Serial.print(F("ADC reading: "));
    //  Serial.println(voltage);
    Serial.print(voltage);
      delay(100);

      //temperature calculation
      float temperature = temp_conversion(comm_reading);
      Serial.print(F(" T: "));
      Serial.println(temperature);
      if(temperature > stpm.battery_temp) //store temp in stpm if it is high than current stpm temp
      {
       stpm.battery_temp = temperature;
      }
      if(stpm.battery_temp >= CELL_OVER_TEMP_THRESHOLD_CHARGE_C && current_state_of_battery == charging)
      {
        stpm.battery_temp_too_high_for_charging_flag = 1;
      }
      if(stpm.battery_temp >= CELL_OVER_TEMP_THRESHOLD_DISCHARGE_C && current_state_of_battery == discharging)
      {
        stpm.battery_temp_too_high_for_discharging_flag = 1;
      }
    }//for current ic
}

float BMS_singleton::temp_conversion(uint16_t comm_reading)
{
  uint16_t THERMISTOR_NOMINAL = 10000;
  uint8_t TEMP_NOMINAL = 25;
  uint16_t BETA_COEFFICIENT = 3950;
  uint16_t SERIES_RESISTOR = 10000;

  float resistance = SERIES_RESISTOR * 1.0 / (65535 * 1.0 / comm_reading - 1);
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

void BMS_singleton::monitor_all_cell_groups_voltage_and_temp(CAN_manager_singleton& CAN_manager)
{
  //2.6 low side cutoff for voltage. 4.167 - high side

  //after looping through all cells:
  if (stpm.cell_voltage_reach_4_16_flag || stpm.battery_temp_too_high_for_charging_flag || stpm.battery_temp_too_high_for_discharging_flag || stpm.total_voltage > TOTAL_BATTERY_VOLTAGE) //if any cell_group reached 4.2 V or overheats while charging
  {
    if(current_state_of_battery == charging)
    {
      write_to_charger(CAN_manager, 0, 0);//disable charger
      disable_charger(CAN_manager);//stop charging
      Serial.println(F("Full cell or over temperature or total voltage achieved, charging is stopped..."));
    }
    else if (current_state_of_battery == discharging)
    {
      //need to do temp scheduling
      //Serial.println(F("Full battery or over temperature detected"));
    }
  }
  else if (stpm.cell_voltage_reach_4_flag) //if any cell_group reached 4 V  288/72 = 4V
  {
    if(current_state_of_battery == charging)//if in charging state
    {
      write_to_charger(CAN_manager, TOTAL_BATTERY_VOLTAGE, CHARGER_OUTPUT_CURRENT_SPEC);//get batteries to higher voltage
      enable_charger(CAN_manager);
      Serial.println(F("Cell at threshold voltage 4.0V detected. increase charger voltage to 299.7V"));
    }
  }
  else if (stpm.cell_voltage_too_low_flag || stpm.BMS_fault_flag || stpm.cell_voltage_too_high_flag )
  {
    if(current_state_of_battery == charging)
    {
      write_to_charger(CAN_manager, 0, 0);//disable charger
      disable_charger(CAN_manager);//stop charging
    }
      Serial.println(F("battery voltage below 2.6 or BMS fault detected"));
  }
  else //if there is no issue
  {
      Serial.println(F("No battery over voltage threshold or over temperature detected, everything fine"));
      if(current_state_of_battery == charging)
      {
        write_to_charger(CAN_manager, CHARGER_OUTPUT_VOLTAGE_SPEC, CHARGER_OUTPUT_CURRENT_SPEC); //set charger max voltage and current: 299.5V, 7.5A
        enable_charger(CAN_manager); //enable charger
      }
  }
}
