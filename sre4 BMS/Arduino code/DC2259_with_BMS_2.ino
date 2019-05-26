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

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2017 Linear Technology Corp. (LTC)
 */


/*! @file
    @ingroup LTC6811-1
*/

#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC6811.h"
#include "BMS.h"
#include "CAN_manager.h"

#define CAN_BUS_INT  2

/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
/*const uint8_t TOTAL_IC = 1;//!<number of ICs in the daisy chain

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
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED*/
/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the LTC6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/

//cell_asic bms_ic[TOTAL_IC];

MCP_CAN CAN_BUS(9); //move from pin 10 to pin 9 because of conflict of cs pin with 6811
CAN_manager_singleton &CAN_manager = CAN_manager_singleton::getInstance();
BMS_singleton &BMS = BMS_singleton::getInstance();


/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
  Serial.begin(115200);
  quikeval_SPI_connect();
  //spi_enable(SPI_CLOCK_DIV16); // This will set the Linduino to have a 1MHz Clock
  spi_enable(SPI_CLOCK_DIV32); // This will set the Linduino to have a 500KHz Clock

  if (!CAN_manager.CAN_manager_init_bus(CAN_BUS, 500000))//init bus with 500kHZ bus speed
    Serial.println(F("MCP2515 Initialized Successfully!"));
  else
    Serial.println(F("Error Initializing MCP2515..."));
}

/*!*********************************************************************
  \brief main loop
***********************************************************************/
void loop()
{
     BMS.read_all_cell_groups_voltage();
     BMS.read_all_cell_groups_temp();
     BMS.print_all_cell_groups_voltage_and_temp();

delay(1000);
}





