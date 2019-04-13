//header file to send and receive CAN messages between the BMS and the charger
//By Nuoya Xie

#ifndef BMS_DATATYPES_H_
#define BMS_DATATYPES_H_

//Individual cell specifications (from Hg2 data sheet)
#define CELL_OVER_VOLTAGE_THRESHOLD_V 4.1
#define CELL_ABSOLUTE_MAXIMUM_VOLTAGE_V 4.2
#define CELL_OVER_TEMP_THRESHOLD_CHARGE_C 50
#define CELL_OVER_TEMP_THRESHOLD_DISCHARGE_C 75
#define CELL_STANDARD_CHARGE_CONST_CURRENT_MA 1500
#define CELL_STANDARD_CHARGE_CONST_VOLTAGE_V 4.2
#define CELL_STANDARD_CHARGE_END_CURRENT_MA 50

//self-defined variables for 6811 Inc
#define TOTAL_CELL_GROUP 9 //9 cell group/module

//setup variables and constants needed for the linduino 6811 APIs, created by LT
const uint8_t TOTAL_IC = 8;//!<number of ICs in the daisy chain, 8 in our case
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ;
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options//MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options

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
	charging = 1 //this means battery is off the car for charging
}battery_state;

#endif
