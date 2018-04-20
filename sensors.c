/*****************************************************************************
* Sensors
******************************************************************************
* bla bla bla.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-12-01 - Rusty Pedrosa - Changed loading of sensor data to switch
*                              statement inside of a loop
*****************************************************************************/

#include "IO_Driver.h"  //Includes datatypes, constants, etc - should be included in every c file
#include "IO_ADC.h"
#include "IO_PWD.h"
#include "IO_PWM.h"
#include "IO_DIO.h"

#include "sensors.h"
#include "mathFunctions.h"
<<<<<<< HEAD
#include "initializations.h"
=======
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db

extern Sensor Sensor_TPS0;
extern Sensor Sensor_TPS1;
extern Sensor Sensor_BPS0;
//extern Sensor Sensor_BPS1;
extern Sensor Sensor_WSS_FL;
extern Sensor Sensor_WSS_FR;
extern Sensor Sensor_WSS_RL;
extern Sensor Sensor_WSS_RR;
extern Sensor Sensor_WPS_FL;
extern Sensor Sensor_WPS_FR;
extern Sensor Sensor_WPS_RL;
extern Sensor Sensor_WPS_RR;
extern Sensor Sensor_SAS;
extern Sensor Sensor_LVBattery;

extern Sensor Sensor_BenchTPS0;
extern Sensor Sensor_BenchTPS1;

extern Sensor Sensor_RTDButton;
extern Sensor Sensor_EcoButton;
extern Sensor Sensor_TCSKnob;
extern Sensor Sensor_TCSSwitchUp;
extern Sensor Sensor_TCSSwitchDown;
extern Sensor Sensor_HVILTerminationSense;

/*-------------------------------------------------------------------
* getPercent
* Returns the % (position) of value, between min and max
* If zeroToOneOnly is true, then % will be capped at 0%-100% (no negative % or > 100%)
-------------------------------------------------------------------*/
//----------------------------------------------------------------------------
// Read sensors values from ADC channels
// The sensor values should be stored in sensor objects.
//----------------------------------------------------------------------------
void sensors_updateSensors(void)
{
    //TODO: Handle errors (using the return values for these Get functions)

    //TODO: RTDS

    //Torque Encoders ---------------------------------------------------
    //Sensor_BenchTPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_00, &Sensor_BenchTPS0.sensorValue, &Sensor_BenchTPS0.fresh);
    //Sensor_BenchTPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_01, &Sensor_BenchTPS1.sensorValue, &Sensor_BenchTPS1.fresh);
    ubyte2 sensorValueTPS0 = (ubyte2)Sensor_TPS0.sensorValue;
    ubyte2 sensorValueTPS1 = (ubyte2)Sensor_TPS1.sensorValue;
    
    Sensor_TPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_00, &sensorValueTPS0, &Sensor_TPS0.fresh);
    Sensor_TPS1.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_01, &sensorValueTPS1, &Sensor_TPS1.fresh);
    //Sensor_TPS0.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_00, &Sensor_TPS0.sensorValue);
<<<<<<< HEAD
<<<<<<< HEAD
	//Sensor_TPS1.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_01, &Sensor_TPS1.sensorValue);
=======
    //Sensor_TPS1.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_01, &Sensor_TPS1.sensorValue);
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485

    
    //Brake Position Sensor ---------------------------------------------------
    ubyte2 sensorValueBPS0 = (ubyte2)Sensor_BPS0.sensorValue;
<<<<<<< HEAD
	Sensor_BPS0.ioErr_signalGet = IO_ADC_Get(IO_DO_05, &sensorValueBPS0, &Sensor_BPS0.fresh);
=======
    Sensor_BPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_02, &sensorValueBPS0, &Sensor_BPS0.fresh);
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485

    //TCS Knob
    Sensor_TCSKnob.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_04, &Sensor_TCSKnob.sensorValue, &Sensor_TCSKnob.fresh);
    
<<<<<<< HEAD
	//?? - For future use ---------------------------------------------------
=======
    //Sensor_TPS1.ioErr_signalGet = IO_PWD_PulseGet(IO_PWM_01, &Sensor_TPS1.sensorValue);

    
    //Brake Position Sensor ---------------------------------------------------
    ubyte2 sensorValueBPS0 = (ubyte2)Sensor_BPS0.sensorValue;
    Sensor_BPS0.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_02, &sensorValueBPS0, &Sensor_BPS0.fresh);

    //TCS Knob
    Sensor_TCSKnob.ioErr_signalGet = IO_ADC_Get(IO_ADC_5V_04, &Sensor_TCSKnob.sensorValue, &Sensor_TCSKnob.fresh);
    
    //?? - For future use ---------------------------------------------------
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
    //?? - For future use ---------------------------------------------------
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
    //IO_ADC_Get(IO_ADC_5V_03, &Sensor_BPS1.sensorValue, &Sensor_BPS1.fresh);

    //Shock pots ---------------------------------------------------
    /*IO_ADC_Get(IO_ADC_5V_04, &Sensor_WPS_FL.sensorValue, &Sensor_WPS_FL.fresh);
    IO_ADC_Get(IO_ADC_5V_05, &Sensor_WPS_FR.sensorValue, &Sensor_WPS_FR.fresh);
    IO_ADC_Get(IO_ADC_5V_06, &Sensor_WPS_RL.sensorValue, &Sensor_WPS_RL.fresh);
    IO_ADC_Get(IO_ADC_5V_07, &Sensor_WPS_RR.sensorValue, &Sensor_WPS_RR.fresh);
<<<<<<< HEAD
<<<<<<< HEAD
	*/

    //Wheel speed sensors ---------------------------------------------------
	Sensor_WSS_FL.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_FL.sensorValue);
	Sensor_WSS_FR.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FR.sensorValue);
	Sensor_WSS_RL.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RL.sensorValue);
	Sensor_WSS_RR.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_RR.sensorValue);
=======
    */

    //Wheel speed sensors ---------------------------------------------------
=======
    */

    //Wheel speed sensors ---------------------------------------------------
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
    Sensor_WSS_FL.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_10, &Sensor_WSS_FL.sensorValue);
    Sensor_WSS_FR.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_08, &Sensor_WSS_FR.sensorValue);
    Sensor_WSS_RL.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_11, &Sensor_WSS_RL.sensorValue);
    Sensor_WSS_RR.ioErr_signalGet = IO_PWD_FreqGet(IO_PWD_09, &Sensor_WSS_RR.sensorValue);
<<<<<<< HEAD
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485

    //Switches / Digital ---------------------------------------------------
    ubyte2 sensorValueRTDButton = (ubyte2)Sensor_RTDButton.sensorValue;
    
<<<<<<< HEAD
<<<<<<< HEAD
	Sensor_RTDButton.ioErr_signalGet = IO_DI_Get(IO_DI_00, &sensorValueRTDButton);
	Sensor_EcoButton.ioErr_signalGet = IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
	Sensor_TCSSwitchUp.ioErr_signalGet = IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchUp.sensorValue);
	Sensor_TCSSwitchDown.ioErr_signalGet = IO_DI_Get(IO_DI_03, &Sensor_TCSSwitchDown.sensorValue);
	Sensor_HVILTerminationSense.ioErr_signalGet = IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
	Sensor_LVBattery.ioErr_signalGet = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);
=======
    Sensor_RTDButton.ioErr_signalGet = IO_DI_Get(IO_DI_00, &sensorValueRTDButton);
    Sensor_EcoButton.ioErr_signalGet = IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
    Sensor_TCSSwitchUp.ioErr_signalGet = IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchUp.sensorValue);
    Sensor_TCSSwitchDown.ioErr_signalGet = IO_DI_Get(IO_DI_03, &Sensor_TCSSwitchDown.sensorValue);
    Sensor_HVILTerminationSense.ioErr_signalGet = IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    Sensor_LVBattery.ioErr_signalGet = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
    Sensor_RTDButton.ioErr_signalGet = IO_DI_Get(IO_DI_00, &sensorValueRTDButton);
    Sensor_EcoButton.ioErr_signalGet = IO_DI_Get(IO_DI_01, &Sensor_EcoButton.sensorValue);
    Sensor_TCSSwitchUp.ioErr_signalGet = IO_DI_Get(IO_DI_02, &Sensor_TCSSwitchUp.sensorValue);
    Sensor_TCSSwitchDown.ioErr_signalGet = IO_DI_Get(IO_DI_03, &Sensor_TCSSwitchDown.sensorValue);
    Sensor_HVILTerminationSense.ioErr_signalGet = IO_DI_Get(IO_DI_07, &Sensor_HVILTerminationSense.sensorValue);

    //Other stuff ---------------------------------------------------
    //Battery voltage (at VCU internal electronics supply input)
    Sensor_LVBattery.ioErr_signalGet = IO_ADC_Get(IO_ADC_UBAT, &Sensor_LVBattery.sensorValue, &Sensor_LVBattery.fresh);
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485


}

void Light_set(Light light, float4 percent)
{
    ubyte2 duty = 65535 * percent;
    bool power = duty > 5000 ? TRUE : FALSE; //Even though it's a lowside output, TRUE = on

    switch (light)
    {
    //PWM devices
<<<<<<< HEAD
<<<<<<< HEAD
	case Light_brake:
		// IO_PWM_SetDuty(IO_PWM_02, duty, NULL);  //Pin 117
        IO_DO_Set( IO_DO_08, power); // Pin 120 as DO
		break;
=======
=======
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
    case Light_brake:
        IO_PWM_SetDuty(IO_PWM_00, duty, NULL);  //Pin 116
        IO_PWM_SetDuty(IO_PWM_01, duty, NULL);
        IO_PWM_SetDuty(IO_PWM_02, duty, NULL);
        IO_PWM_SetDuty(IO_PWM_03, duty, NULL);
        IO_PWM_SetDuty(IO_PWM_04, duty, NULL);
        IO_PWM_SetDuty(IO_PWM_06, duty, NULL);
        IO_PWM_SetDuty(IO_PWM_07, duty, NULL);
        break;
<<<<<<< HEAD
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485

    case Cooling_waterPump:
        IO_PWM_SetDuty(IO_PWM_05, duty, NULL);
        break;

    case Cooling_motorFans:
        IO_DO_Set(IO_DO_03, power);
        break;

    case Cooling_batteryFans:
        IO_DO_Set(IO_DO_04, power);
        break;

        //--------------------------------------------
        //These devices moved from PWM to DIO

<<<<<<< HEAD
<<<<<<< HEAD
	case Light_dashTCS:
=======
    case Light_dashTCS:
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
    case Light_dashTCS:
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
        //IO_PWM_SetDuty(IO_PWM_03, duty, NULL);  //Pin 105
        IO_DO_Set(IO_ADC_CUR_00, power);
        break;

<<<<<<< HEAD
<<<<<<< HEAD
	case Light_dashEco:
		//IO_PWM_SetDuty(IO_PWM_04, duty, NULL);  //Pin 116
=======
    case Light_dashEco:
        //IO_PWM_SetDuty(IO_PWM_04, duty, NULL);  //Pin 116
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
        IO_DO_Set(IO_ADC_CUR_01, power);
        break;

<<<<<<< HEAD
	case Light_dashError:
=======
    case Light_dashEco:
        //IO_PWM_SetDuty(IO_PWM_04, duty, NULL);  //Pin 116
        IO_DO_Set(IO_ADC_CUR_01, power);
        break;

    case Light_dashError:
>>>>>>> 6ef510f80dd9bc757deec698135e7378c7d022db
=======
    case Light_dashError:
>>>>>>> 88ace18b2ca8495d00261f11c5d0420104c62485
        //IO_PWM_SetDuty(IO_PWM_05, duty *.6, NULL);  //Pin 104
        IO_DO_Set(IO_ADC_CUR_02, power);
        break;

    case Light_dashRTD:
        //IO_PWM_SetDuty(IO_PWM_06, duty * .25, NULL);  //Pin 115
        IO_DO_Set(IO_ADC_CUR_03, power);
        break;
    }

}

/*****************************************************************************
* Output Calculations
******************************************************************************
* Takes properties from devices (such as raw sensor values [ohms, voltage],
* MCU/BMS CAN messages, etc), performs calculations with that data, and updates
* the relevant objects' properties.
*
* This includes sensor calculations, motor controller control calculations,
* traction control, BMS/safety calculations, etc.
* (May need to split this up later)
*
* For example: GetThrottlePosition() takes the raw TPS voltages from the TPS
* sensor objects and returns the throttle pedal percent.  This function does
* NOT update the sensor objects, but it would be acceptable for another
* function in this file to do so.
*
******************************************************************************
* To-do:
*
******************************************************************************
* Revision history:
* 2015-11-16 - Rusty Pedrosa -
*****************************************************************************/
