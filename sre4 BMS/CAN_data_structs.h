
#ifndef CAN_DATA_STRUCTS_
#define CAN_DATA_STRUCTS_

// CAN Message IDs
 #define BSPD_ON_OFF_MID 0x520
 #define BSPD_SET_CURRENT_MID 0x521
 #define RTD_MID 0x522
#define CHARGER_MID 0x18FF50E5

//data frame for message send from BMS
typedef union
	{
		uint8_t dword[8];
		struct
		{
			//voltage (2 Bytes)
			uint8_t voltage_to_charger_MSB: 8;
			uint8_t voltage_to_charger_LSB: 8;

			//current (2 Bytes)
			uint8_t current_to_charger_MSB: 8;
			uint8_t current_to_charger_LSB: 8;

			//control byte
			uint8_t control: 8; //0 = on, 1 = off

			//reserved 3 bytes at the end (byte 5 - 7)
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
		}__attribute__((packed));
	}data_frame_BMS;

//data frame for message from Charger
typedef union
	{
		uint8_t dword[8];
		struct
		{
			//voltage (2 Bytes)
			uint8_t voltage_from_charger_MSB: 8;
			uint8_t voltage_from_charger_LSB: 8;

			//current (2 Bytes)
			uint8_t current_from_charger_MSB: 8;
			uint8_t current_from_charger_LSB: 8;

			//status (1 byte)
			union
			{
				uint8_t status_byte; //0 = on, 1 = off
				struct
				{
					uint8_t bit0 : 1;
					uint8_t bit1 : 1;
					uint8_t bit2 : 1;
					uint8_t bit3 : 1;
					uint8_t bit4 : 1;
					uint8_t reserved : 3;
				}__attribute__((packed));
			}__attribute__((packed));

			//reserved 3 bytes at the end (byte 5 - 7)
			uint8_t : 8;
			uint8_t : 8;
			uint8_t : 8;
		}__attribute__((packed));
	}data_frame_charger;

#endif
