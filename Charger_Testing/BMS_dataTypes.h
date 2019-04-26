//header file to send and receive CAN messages between the BMS and the charger
//By Nuoya Xie

#ifndef BMS_DATATYPES_H_
#define BMS_DATATYPES_H_

#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC6811.h"

//Individual cell specifications (from Hg2 data sheet)
#define CELL_OVER_VOLTAGE_THRESHOLD_V 4.1
#define CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V 4.2
#define CELL_OVER_TEMP_THRESHOLD_CHARGE_C 50
#define CELL_OVER_TEMP_THRESHOLD_DISCHARGE_C 75
#define CELL_STANDARD_CHARGE_CONST_CURRENT_MA 1500
#define CELL_STANDARD_CHARGE_CONST_VOLTAGE_V 4.2

//self-defined variables for 6811 Inc
#define TOTAL_CELL_GROUP 9 //9 cell group/module
#define ENABLED 1
 #define DISABLED 0
 #define DATALOG_ENABLED 1
 #define DATALOG_DISABLED 0
 #define TEMP_SENSORS_BASE_ADDRESS 20 //decimal(20).  0x14(hex),  0010100(binary), according to Alex

//setup variables and constants needed for the linduino 6811 APIs, created by LT
const uint8_t TOTAL_IC = 1;//!<number of ICs in the daisy chain, 8 in our case
//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See LTC6811_daisy.h for Options

const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold ADC Code. LSB = 0.0001

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED

//the variables below are for temperature conversion, by Alex and Tim
 // Nominal thermistor
#define THERMISTOR_NOMINAL 10000
// Temp for nominal resistor (25 C)
#define TEMP_NOMINAL 25
// Beta coefficient of thermistor (3000 - 4000)
#define BETA_COEFFICIENT 3950
// 10kOhm resistor
#define SERIES_RESISTOR 10000

//This struct contains the voltage information and over-voltage flag for each cell group
typedef struct
{
	float cell_group_voltage;
	float cell_group_temp;
	bool over_voltage_threshold_flag;
	bool over_temp_threshold_flag;
	bool cell_group_battery_full_flag;
}cell_group;

//state of the battery - on the charger or inside the car?
typedef enum
{
	discharging = 0, //this means battery is in the car, not charging
	charging = 1, //this means battery is off the car for charging
  regening = 2
}battery_state;

//The below structs are done by Alex and tim for their BMS_temp.h. I am merely borrowing it
union COMM_WR_REG {
    uint8_t bytes[6];
    // Bit fields of Write register
    struct COMM {
        uint8_t ICOM0   : 4;
        uint8_t D0      : 8;
        uint8_t FCOM0   : 4;
        uint8_t ICOM1   : 4;
        uint8_t D1      : 8;
        uint8_t FCOM1   : 4;
        uint8_t ICOM2   : 4;
        uint8_t D2      : 8;
        uint8_t FCOM2   : 4;
        // uint16_t PEC    : 16;
    } __attribute__((packed)) fields;
};

union COMM_RD_REG {
    uint8_t bytes[8];
    // Bit fields of Read register
    struct COMM {
        uint8_t ICOM0   : 4;
        uint8_t D0      : 8;
        uint8_t FCOM0   : 4;
        uint8_t ICOM1   : 4;
        uint8_t D1      : 8;
        uint8_t FCOM1   : 4;
        uint8_t ICOM2   : 4;
        uint8_t D2      : 8;
        uint8_t FCOM2   : 4;
        uint16_t PEC    : 16;
    } __attribute__((packed)) fields;
};


#endif
