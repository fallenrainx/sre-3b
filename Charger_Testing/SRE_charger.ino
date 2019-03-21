//testing charger send

#include <mcp_can.h>
#include <SPI.h>
#include "CAN_charger.h"

MCP_CAN CAN0(10);     // Set CS to pin 10
#define CAN0_INT 2                              // Set INT to pin 2
BMS_to_charger_CAN_message CANmsg_send = {0};
charger_to_BMS_CAN_message CANmsg_receive = {0};

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 running at 16MHz with a baudrate of 250kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  //initiate the message to the charger with: 5.0V, 1.0A, enable charger
  BMS_to_charger_message_init( 12.0, 2.0, charger_on, &CANmsg_send);
  /*Serial.print("can data byte 0: ");
    Serial.println(CANmsg_send.data.voltage_to_charger_MSB);
    Serial.print("can data byte 1: ");
    Serial.println(CANmsg_send..data.voltage_to_charger_LSB);
    Serial.print("can data byte 2: ");
    Serial.println(CANmsg_send.data.current_to_charger_MSB);
    Serial.print("can data byte 3: ");
    Serial.println(CANmsg_send.data.current_to_charger_LSB);
    Serial.print("can data byte 4: ");
    Serial.println(CANmsg_send.data.control);
    Serial.print("can data byte 5: ");
    Serial.println(CANmsg_send.data.reserved_bytes[0]);
    Serial.print("can data byte 6: ");
    Serial.println(CANmsg_send.data.reserved_bytes[1]);
    Serial.print("can data byte 7: ");
    Serial.println(CANmsg_send.data.reserved_bytes[2]); */
}

void loop()
{
  // send data on CAN bus
  byte sndStat = BMS_to_charger_message_send(&CAN0, &CANmsg_send);
  if (sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully to the charger!");
  } else {
    Serial.println("Error Sending Message...");
  }

  //receive data from the charger
  if (!digitalRead(CAN0_INT))// If CAN0_INT pin is low, read receive buffer
  {
    BMS_to_charger_message_receive(&CAN0, &CANmsg_receive);

    //parsing the message received and display it
    //the following should be part of a parser, but for now it is here
    Serial.print("Message ID: ");
    Serial.println(CANmsg_receive.message_id, HEX);
    Serial.print("data length: ");
    Serial.println(CANmsg_receive.data_length_byte);


    uint8_t voltageMSB = CANmsg_receive.data.voltage_from_charger_MSB;
    uint8_t voltageLSB = CANmsg_receive.data.voltage_from_charger_LSB;
    float voltage_value = ((voltageMSB << 8) | voltageLSB) * 0.1;
    Serial.print("Incoming voltage: ");
    Serial.println(voltage_value);

    uint8_t currentMSB = CANmsg_receive.data.current_from_charger_MSB;
    uint8_t currentLSB = CANmsg_receive.data.current_from_charger_LSB;
    float current_value = ((currentMSB << 8) | currentLSB) * 0.1;
    Serial.print("Incoming current: ");
    Serial.println(current_value);

    Serial.println("Incoming status: ");
    uint8_t status_bits = CANmsg_receive.data.charger_status.status_byte;
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
        Serial.println("Over Temperature");
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
