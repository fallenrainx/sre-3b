//testing charger send

#include <mcp_can.h>
#include <SPI.h>
#include "CAN_charger.h"

MCP_CAN CAN0(10);     // Set CS to pin 10
#define CAN0_INT 2                              // Set INT to pin 2
BMS_to_charger_CAN_message CANmsg_send;
charger_to_BMS_CAN_message CANmsg_receive;

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 250kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  //byte data[8] = {0x00, 0x32, 0x00, 0xA, 0x00, 0x00, 0x00, 0x00};
  //BMS_to_charger_message_init(50, 10, charger_on, &CANmsg_send);
  BMS_to_charger_message_init( 5.0, 1.0, charger_on, &CANmsg_send);
}

void loop()
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = BMS_to_charger_message_send(&CAN0, &CANmsg_send);
  if (sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }

  //receive data from the charger
  if (!digitalRead(CAN0_INT))                // If CAN0_INT pin is low, read receive buffer
  {
    BMS_to_charger_message_receive(&CAN0, &CANmsg_receive);

  //the following should be part of a parser, but for now it is here
    Serial.print("Message ID: ");
    Serial.println(CANmsg_receive.message_id);
    
    uint8_t voltageMSB = CANmsg_receive.data_frame.data_byte_field.voltage_from_charger_MSB;
    uint8_t voltageLSB = CANmsg_receive.data_frame.data_byte_field.voltage_from_charger_LSB;
    float voltage_value = ((voltageMSB << 8) | voltageLSB) * 0.1;
    Serial.print("Incoming voltage: ");
    Serial.println(voltage_value);

    uint8_t currentMSB = CANmsg_receive.data_frame.data_byte_field.current_from_charger_MSB;
    uint8_t currentLSB = CANmsg_receive.data_frame.data_byte_field.current_from_charger_LSB;
    float current_value = ((currentMSB << 8) | currentLSB) * 0.1;
    Serial.print("Incoming current: ");
    Serial.println(current_value);

    Serial.println("Incoming status: ");
    uint8_t status_bits = CANmsg_receive.data_frame.data_byte_field.status_field.status_byte;
    switch (status_bits & (0x1)) //bit 0
    {
      case 1:
      Serial.println("Hardware Failure");
      break;
      default:
      Serial.println("Hareware Normal");
    }

    switch (status_bits & (0x2)) //bit 1
    {
      case 1:
      Serial.println("Over Temperature Protection");
      break;
      default:
      Serial.println("Temperature Normal");
    }

    switch (status_bits & (0x4)) //bit 2
    {
      case 1:
      Serial.println("Input voltage is wrong, the charger will stop working");
      break;
      default:
      Serial.println("Input Voltage Normal");
    }

    switch (status_bits & (0x8)) //bit 3
    {
      case 1:
      Serial.println("Charger stays turned off (to prevent reverse polarity)");
      break;
      default:
      Serial.println("Charger detects battery voltage and starts charging");
    }
    
     switch (status_bits & (0x10)) //bit 4
    {
      case 1:
      Serial.println("Communication receive time-out");
      break;
      default:
      Serial.println("Communication Normal");
    }
    
  }
  delay(1000);   // send data per 1000ms

}
