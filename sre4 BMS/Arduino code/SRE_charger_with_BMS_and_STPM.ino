//testing charger send

#include "BMS.h"
#include "CAN_manager.h"

#define CAN_BUS_INT  2

MCP_CAN CAN_BUS(10);

CAN_manager_singleton &CAN_manager = CAN_manager_singleton::getInstance();
BMS_singleton &BMS = BMS_singleton::getInstance();

void setup()
{
  Serial.begin(115200);
  if (!CAN_manager.CAN_manager_init_bus(CAN_BUS, standard))//init bus with 1MHZ bus speed
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  //BMS.write_to_charger(CAN_manager, 12.0, 0.5); //set charger max voltage and current: 5.0V, 1.0A
  //BMS.enable_charger(CAN_manager); //enable charger
}
void loop()
{
  /* send data on CAN bus
  bool is_send_failed = CAN_manager.BMS_to_charger_message_send(CAN_BUS);
  if (is_send_failed == false) {
    Serial.println("Message Sent Successfully to the charger!");
  } else {
    Serial.println("Error Sending Message...");
  }

  //receive data from the charger if data is ready

  Serial.println("receiving data...");
  CAN_manager.charger_to_BMS_message_receive(CAN_BUS);

  //obtain values from can manager:
  BMS.read_from_charger(CAN_manager);
  Serial.print("Incoming voltage: ");
  Serial.println(BMS.get_charger_output_voltage());
  Serial.print("Incoming current: ");
  Serial.println(BMS.get_charger_output_current());

  Serial.print("is Hardware Failure?");
  Serial.println(BMS.is_hardware_failed_charger_flag());

  Serial.print("is charger over temperature?");
  Serial.println(BMS.is_over_temperature_charger_flag());

  Serial.print("is DC voltage in range?");
  Serial.println(BMS.is_battery_voltage_detected_charger_flag());

  Serial.print("is communication to charger timed out?");
  Serial.println(BMS.is_communication_time_out_charger_flag());

  Serial.print("is AC voltage in range?");
  Serial.println(BMS.is_input_voltage_wrong_charger_flag()); */

  
  BMS.send_stpm(CAN_manager, CAN_BUS);
  delay(1000);   // send data per 1000ms

}
