/**
  ******************************************************************************
  * File Name          : lgs_bluetooth.c
  * Description        : Implementation file
  *
  *  Created on: 29.12.2018
  *  Author: Tobias Tiedemann
  ******************************************************************************
  */


/**
 * Includes
 */
#include "lgs_bluetooth.h"
#include "lgs_datamanagement.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hci.h"
#include "hci_le.h"
#include "hci_tl.h"
#include "hci_const.h"
#include "link_layer.h"

#include "bluenrg_utils.h"
#include "stm32l0_nucleo_l053r8.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "sm.h"
#include "stm32l0xx_hal_tim.h"

//Symbol: Wird für Evaluationsboard kompiliert?
#define EVALUATIONBOARD


/**
 * MAKRO DEFINITIONS
 */
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

//Data Object Bluetooth Data
s_environmentData m_environmentData;

//Handles, saved as a struct
struct s_handles
{
	//Service Handles
	uint16_t	m_gapServiceHandle;				//Service Handle GAP
	//--
	uint16_t 	m_environmentServiceHandle;		//Service Handle Environment
	uint16_t	m_configurationServiceHandle;	//Service Handle Configuration
	uint16_t 	m_fsmServiceHandle;				//Service Handle FSM

	//Characteristic Data Handles
	uint16_t    m_deviceNameCharHandle;			//Characteristic Handle Device Name
	uint16_t	m_appearanceCharHandle;			//Characteristic Handle Apperance
	//--
	uint16_t 	m_environmentCharBrightHandle;	//Characteristic Handle Bright Flag
	uint16_t 	m_environmentCharTempHandle;	//Characteristic Handle Temperature
	uint16_t 	m_environmentCharVOCHandle;		//Characteristic Handle VOC
	uint16_t 	m_environmentCharCO2Handle;		//Characteristic Handle CO2
	uint16_t 	m_environmentCharHumidityHandle;//Characteristic Handle Air Humidity
	uint16_t 	m_environmentCharPressureHandle;//Characteristic Handle Air Pressure

	//Characteristic Settings Handles
	uint16_t    m_configurationsRepRateHandle; 	//Characteristic Handle Reprate Setting
	uint16_t	m_configurationsOutputActive;	//Characteristic Handle Output Flag
	uint16_t	m_configurationsCriticTemperature; //Characteristic Handle kritische Temperatur
	uint16_t	m_configurationsCriticVOC;		//Characteristic Handle kritischer VOC Wert
	uint16_t	m_configurationsCriticCo2;		//Characteristic Handle kritischer CO2 Wert
	uint16_t	m_configurationsCriticHumidity;	//Characteristic Handle kritische Feuchte
	uint16_t	m_configurationsCriticPressure;	//Characteristic Handle kritischer Druck

	//Characteristic FSM Handles
	uint16_t	m_fsmRequestHandle;				//Characteristic Handle FSM Request
	uint16_t	m_fsmNotifyReadyHandle;			//Characteristic Handle FSM NotifyReady Paket
	uint16_t	m_fsmReadPaketHandle;			//Characteristic Handle FSM Paket
}m_serviceHandles;



//General Variables
uint8_t 	set_connectable = TRUE;
uint16_t 	connection_handle = 0;
int     	connected;
uint8_t  	notification_enabled = FALSE;
uint8_t 	bnrg_expansion_board = IDB05A1; //IDB5 expansion board --> Für initBLEModul
uint8_t 	bdaddr[BDADDR_SIZE];

//Ticks Speicher (f. Timer)
uint32_t	m_startTicks = 0U;

//FSM Speicher: Der angeforderte Datentyp
uint8_t 	m_requestedDataType = 0U;
uint32_t	m_fsmTimeoutTick = 0U;
uint32_t	m_fsmTickSendIndicateReady = 0U;
union indicatePaket
{
	uint16_t u16[2];
	uint32_t u32;
}m_indicatePaket;

Service_UUID_t 	service_uuid;
Char_UUID_t 	char_uuid;


//Service UUID
#define GET_ENVIRONMENT_SERVICE_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x7d,0x36,0xee,0xd5,0xca,0x05,0x42,0xf3,0x86,0x7e,0x4d,0x80,0x0a,0x45,0x9c,0xa1)
#define GET_CONFIGURATION_SERVICE_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x7d,0x36,0xee,0xd5,0xca,0x05,0x42,0xf3,0x86,0x7e,0x4d,0x80,0x0a,0x45,0x9c,0xa2)
#define GET_FSM_READDATA_SERVICE_UUID(uuid_struct)	\
	COPY_UUID_128(uuid_struct,0xad,0x42,0xa5,0x90,0xb7,0xaf,0x40,0x82,0xb8,0xf4,0xb4,0xb4,0x87,0x98,0xe6,0x96)

//Daten UUID
#define GET_ENVIRONMENT_CHAR_BRIGHT_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0xc5,0x09,0x56,0xf6,0xcb,0x78,0x48,0x7e,0x95,0x66,0xb8,0x83,0xff,0x3e,0x5d,0x53)
#define GET_ENVIRONMENT_CHAR_TEMP_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0xb3,0x31,0x02,0xeb,0x43,0xa0,0x4d,0xa1,0x81,0x83,0xed,0x16,0x9c,0x0f,0x17,0x20)
#define GET_ENVIRONMENT_CHAR_VOC_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x6b,0xb0,0x14,0xe9,0xa0,0xc1,0x47,0xb7,0x93,0x9d,0xf9,0x7b,0x8e,0x4f,0x78,0x77)
#define GET_ENVIRONMENT_CHAR_CO2_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x4e,0x1f,0xca,0xdd,0xcd,0xbf,0x46,0xbc,0x8f,0xaa,0x4b,0x06,0x32,0x0c,0xfa,0x2c)
#define GET_ENVIRONMENT_CHAR_HUMIDITY_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x4e,0x31,0x1c,0xb9,0xa6,0x8b,0x44,0xb7,0xaa,0x97,0xa5,0x91,0x19,0x0a,0xa0,0x8e)
#define GET_ENVIRONMENT_CHAR_PRESSURE_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x66,0x6b,0x7e,0x99,0xe9,0x73,0x48,0x60,0x90,0x06,0xc7,0x8c,0xb9,0x5d,0xa5,0xaa)

//Settings UUID
#define GET_ENVIRONMENT_CHAR_SETTINGS_REPRATE_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x69)
#define GET_ENVIRONMENT_CHAR_SETTINGS_CRITTEMP_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6a)
#define GET_ENVIRONMENT_CHAR_SETTINGS_CRITPRES_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6b)
#define GET_ENVIRONMENT_CHAR_SETTINGS_CRITCO2_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6c)
#define GET_ENVIRONMENT_CHAR_SETTINGS_CRITHUM_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6d)
#define GET_ENVIRONMENT_CHAR_SETTINGS_CRITVOC_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6e)
#define GET_ENVIRONMENT_CHAR_SETTINGS_OUTPUTACT_UUID(uuid_struct)   \
	COPY_UUID_128(uuid_struct,0x28,0x6c,0xc2,0x04,0x4b,0x3f,0x4f,0x82,0x8e,0xbb,0x66,0x73,0x72,0xb1,0x56,0x6f)

//FSM ReadData UUID
#define GET_FSM_READDATA_CHAR_REQUEST_UUID(uuid_struct)	\
	COPY_UUID_128(uuid_struct,0xad,0x42,0xa5,0x90,0xb7,0xaf,0x40,0x82,0xb8,0xf4,0xb4,0xb4,0x87,0x98,0xe6,0x97)
#define GET_FSM_READDATA_CHAR_NOTIFYREADY_UUID(uuid_struct)	\
	COPY_UUID_128(uuid_struct,0xad,0x42,0xa5,0x90,0xb7,0xaf,0x40,0x82,0xb8,0xf4,0xb4,0xb4,0x87,0x98,0xe6,0x98)
#define GET_FSM_READDATA_CHAR_READPAKET_UUID(uuid_struct)	\
	COPY_UUID_128(uuid_struct,0xad,0x42,0xa5,0x90,0xb7,0xaf,0x40,0x82,0xb8,0xf4,0xb4,0xb4,0x87,0x98,0xe6,0x99)

/**
 * Private function prototypes
 */
void 		LGS_Process(void);
void 		LGS_SetAddress(uint8_t* bdaddr, uint8_t hwVersion, uint16_t fwVersion);
tBleStatus  LGS_AddServices(void);
void 		LGS_BLE_IndicateReadProcessReady(void);
tBleStatus 	LGS_UpdateStackData(void);

void 		GAP_DisconnectionComplete_CB(void);
void 		GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void 		Read_Request_CB(uint16_t handle);
void 		Write_Request_CB(uint16_t handle, uint8_t data_length, uint8_t* data);




/**
 * DEFINITION Public LGS_BLE_Init
 * -> Initialize the peripherals and the BLE Stack
 * -> Aufruf in main()
 */
void LGS_BLE_Init(void)
{
	uint8_t  hwVersion;						//HW Version BlueNRG
	uint16_t fwVersion;						//FW Version BlueNRG
	int ret;								//Verarbeitung von Rückgabewerten
	const char * name = "LGS";				//Device Name

#ifdef EVALUATIONBOARD
	BSP_LED_Init(LED2);
#endif

	//Init HCI:
	hci_init(LGS_UserNotify, NULL);				//Initialize HCI (Host-Controller-Interface) Callback
	getBlueNRGVersion(&hwVersion, &fwVersion);	//Get Hard-/Firmware Version

	//Set MAC Adress
	hci_reset(); 								//Reset HCI (Notwendig um MAC Adresse zu ändern)
	HAL_Delay(100);								//100ms warten
	LGS_SetAddress(bdaddr,hwVersion, fwVersion);//Call LGS_SetAddress to define MAC Adress
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
	                                  CONFIG_DATA_PUBADDR_LEN,
	                                  bdaddr);	//Write MAC Adress to Adapter
	if (ret) while(1);							//Fehler? -> while(1)

	//Init General Attributes Profile Service
	ret = aci_gatt_init();  					//Init GATT
	if (ret) while(1);							//Fehler? -> while(1)

	//Init General Access Profile Service
	ret = aci_gap_init_IDB05A1(
			GAP_PERIPHERAL_ROLE_IDB05A1, 			//ROLE: PERIPHERAL/BROADCASTER/CENTRAL/OBSERVER
			0, 										//Privacy Enabled Flag
			0x03, 									//Device Name Length: LGS
			&m_serviceHandles.m_gapServiceHandle,	//GAP Service Handle
			&m_serviceHandles.m_deviceNameCharHandle,//Name Characteristic Handle
			&m_serviceHandles.m_appearanceCharHandle);//Apperance Characteristic Handle
	if (ret) while(1);							//Fehler? -> while(1)

	//Set Device Name
	ret = aci_gatt_update_char_value(
			m_serviceHandles.m_gapServiceHandle, 	//GAP Service Handle
			m_serviceHandles.m_deviceNameCharHandle,//Name Characteristic Handle
			0,                					//CharValOffset; Muss 0 sein, damit nächster Wert genommen wird:
			strlen(name), 						//Länge des Characteristic Werts
			(uint8_t *)name);					//Der Characteristic Wert
	if (ret) while(1);							//Fehler? -> while(1)

	//Set Authentication Requrements
	ret = aci_gap_set_auth_requirement(
			MITM_PROTECTION_REQUIRED,			//MITM Mode: REQUIRED/NOT_REQUIRED
			OOB_AUTH_DATA_ABSENT,				//OOb Data present? ABSENT/PRESENT
			NULL,								//OOB Data
			7,									//Minimum size of the encryption key to be used during the pairing process
			16,									//Maximum size of the encryption key to be used during the pairing process
			USE_FIXED_PIN_FOR_PAIRING,			//•USE_FIXED_PIN_FOR_PAIRING / •DONOT_USE_FIXED_PIN_FOR_PAIRING
			123456,								//PIN
			BONDING);							//Bonding Mode: BONDING/NO_BONDING
	if (ret) while(1);							//Fehler? -> while(1)


	//###############################
	//BLE STACK IST NUN INITIALISIERT
	//###############################

	//Weitere Services Initialisieren
	ret = LGS_AddServices();			//Init Environment Service
	if (ret) while(1);							//Fehler? -> while(1)

	//Set Output Power Level
	(void)aci_hal_set_tx_power_level(
			1,									//Enable High Power Flag: 0/1
			PA_LEVEL);							//PA_Level: 0-7; Default: 4


	//Startzeitpunkt auf 0 setzen:
	m_startTicks = 0U;
}

/**
 * BlueNRG-MS background task
 * -> Triggered in Main Loop
 */
void LGS_BLE_Process(void)
{
	LGS_Process();
	hci_user_evt_proc();
}

/**
 * Define the MAC Adress of the Device
 */
void LGS_SetAddress(uint8_t* bdaddr, uint8_t hwVersion, uint16_t fwVersion)
{
	//Random
	uint8_t i;

	/* Initialize a random seed */
	srand (HAL_GetTick() + hwVersion + fwVersion);

	for (i=0; i<5; i++) {
		bdaddr[i] = rand()&0xFF;
	}
	bdaddr[i] = 0xD0;
}

/**
 * Specific LGS Handling
 */
void LGS_Process(void)
{
	//Initial den Discovery Modus setzen
	if (set_connectable)
	{
		LGS_SetDiscoverableMode();
		set_connectable = FALSE;
		LGS_BLE_UpdateDefaultSettings(); //for readonly
	}

	//Beispielapplikation togglet hier die LED
#ifdef EVALUATIONBOARD
	BSP_LED_Toggle(LED2);
#endif

	if (connected) //Besteht eine Verbindung zu einem Smartphone?
	{
		//Mindest-Delay abgelaufen?
		if((HAL_GetTick() - m_startTicks) > (((uint32_t)LGS_CYCLIC_SEND_INTERVAL_DEFAULT) * 1000U))
		{
			//Update verfügbar?
			if(m_environmentData.m_isUpdateAvailable)
			{
				LGS_UpdateStackData();
				m_environmentData.m_isUpdateAvailable = FALSE;

				m_startTicks = HAL_GetTick();
			}
		}

		if(LGS_DATAMANAGEMENT_ISPROCESSACTIVE())
		{
			//Timeout für FSM?
			if((HAL_GetTick() - m_fsmTimeoutTick) > FSM_TIMEOUT)
			{
				//Prozess abbrechen
				LGS_DATAMANAGEMENT_ReadProcessFinished();
			}

			//Delay für sende NotifyReady abgelaufen?
			if((m_fsmTickSendIndicateReady > 0U)
					&& ((HAL_GetTick() - m_fsmTickSendIndicateReady) > FSM_DELAYINDICATEREADY))
			{
				LGS_BLE_IndicateReadProcessReady();
				m_fsmTickSendIndicateReady = 0U;
			}
		}
	}
	else
	{
		if(LGS_DATAMANAGEMENT_ISPROCESSACTIVE())
		{
			LGS_DATAMANAGEMENT_ReadProcessFinished();
		}
	}
}

/*
 * Generiert zufällige Sensordaten (halbwegs sinnvoll)
 */
void LGS_GenerateRandomData(void)
{
	srand(HAL_GetTick()); //Set Seed for Random Generator

	//Toggle Bright Flag
	if(m_environmentData.m_environmentBright == 1)
	{
		m_environmentData.m_environmentBright = 0;
	}
	else
	{
		m_environmentData.m_environmentBright = 1;
	}

	//Random Temperature
	m_environmentData.m_environmentTemperature 	= 15 	+ 	((uint64_t)rand()*10)	/RAND_MAX; //15-25 °C
	m_environmentData.m_environmentAirPressure 	= 980 	+ 	((uint64_t)rand()*40)	/RAND_MAX; //980-1020 mBar
	m_environmentData.m_environmentCO2 			= 400 	+ 	((uint64_t)rand()*20)	/RAND_MAX; //400-420 ppm
	m_environmentData.m_environmentAirHumidity 	=  20   +	((uint64_t)rand()*60)	/RAND_MAX; //20-80 %
	m_environmentData.m_environmentVOC 			= 120   +	((uint64_t)rand()*100)	/RAND_MAX; //20-220 ppb

	m_environmentData.m_isUpdateAvailable = TRUE;
}

/**
 * Updated alle Daten im BLE Stack
 */
tBleStatus LGS_UpdateStackData(void)
{
	tBleStatus status;

	//Bright Flag
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharBrightHandle,	//Characteristic Handle
			0, 												//Offset
			2+sizeof(m_environmentData.m_environmentBright), 	//LEN
			&m_environmentData.m_environmentBright);		//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//Temperature
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharTempHandle,	//Characteristic Handle
			0, 												//Offset
			2+sizeof(m_environmentData.m_environmentTemperature), //LEN
			&m_environmentData.m_environmentTemperature);		//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//Pressure
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharPressureHandle,	//Characteristic Handle
			0, 													//Offset
			2+sizeof(m_environmentData.m_environmentAirPressure), //LEN
			&m_environmentData.m_environmentAirPressure);		//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//CO2
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharCO2Handle,	//Characteristic Handle
			0, 												//Offset
			2+sizeof(m_environmentData.m_environmentCO2), 	//LEN
			&m_environmentData.m_environmentCO2);			//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//Humidity
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharHumidityHandle,//Characteristic Handle
			0, 													//Offset
			2+sizeof(m_environmentData.m_environmentAirHumidity), //LEN
			&m_environmentData.m_environmentAirHumidity);		//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//VOC
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_environmentServiceHandle, 	//Service Handle
			m_serviceHandles.m_environmentCharVOCHandle,	//Characteristic Handle
			0, 												//Offset
			2+sizeof(m_environmentData.m_environmentVOC), 	//LEN
			&m_environmentData.m_environmentVOC);			//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;

	//Output Active Flag:
	status = aci_gatt_update_char_value(
			m_serviceHandles.m_configurationServiceHandle, 	//Service Handle
			m_serviceHandles.m_configurationsOutputActive,	//Characteristic Handle
			0, 												//Offset
			2+sizeof(m_environmentData.m_outputActive), 	//LEN
			&m_environmentData.m_outputActive);				//const void *  charValue
	if(status != BLE_STATUS_SUCCESS) goto fail;


	return BLE_STATUS_SUCCESS;

	fail:
	  return BLE_STATUS_ERROR;
}


/*
 * Updated die Settings
 */
void LGS_BLE_UpdateDefaultSettings(void)
{
	//Reprate:
	(void)aci_gatt_update_char_value(
			m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
			m_serviceHandles.m_configurationsRepRateHandle,  			//Characteristic Handle
			0, 															//Offset
			2+sizeof(m_environmentData.m_repRateBT), 					//LEN
			&m_environmentData.m_repRateBT);							//const void *  charValue

	//Critic Temperature:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsCriticTemperature,  		//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_criticTemperature), 			//LEN
				&m_environmentData.m_criticTemperature);					//const void *  charValue

	//Critic VOC:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsCriticVOC,  				//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_criticVOC), 					//LEN
				&m_environmentData.m_criticVOC);							//const void *  charValue

	//Critic CO2:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsCriticCo2,  				//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_criticCo2), 					//LEN
				&m_environmentData.m_criticCo2);							//const void *  charValue

	//Critic Humidity:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsCriticHumidity,  			//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_criticHumidity), 				//LEN
				&m_environmentData.m_criticHumidity);						//const void *  charValue

	//Critic Pressure:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsCriticPressure,  			//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_criticPressure), 				//LEN
				&m_environmentData.m_criticPressure);						//const void *  charValue

	//Output Active Flag:
	(void)aci_gatt_update_char_value(
				m_serviceHandles.m_configurationServiceHandle, 				//Service Handle
				m_serviceHandles.m_configurationsOutputActive,  			//Characteristic Handle
				0, 															//Offset
				2+sizeof(m_environmentData.m_outputActive), 				//LEN
				&m_environmentData.m_outputActive);							//const void *  charValue

	m_environmentData.m_isUpdateAvailable = TRUE;
}


/**
 * Setzt das Device in Connectable Mode
 */
void LGS_SetDiscoverableMode(void)
{
	uint8_t ret;
	const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, LGS_MOD_NAME};

#define LEN (22)							//Length of manuf_data
	uint8_t manuf_data[LEN] = {
			0x02,	0x0A,	0x04, 			//Length: 2 Byte; Type: TransmissionPower; Value: 4dBm
			0x0A,	0x09,	LGS_MOD_NAME, 	//Length: 10Byte; Type: Full Device Name; Value: LGS_MOD_NAME
			0x07, 	0xFF,					//Length: 7 Byte; Type: SKD Version; Value: MAC Adress
			bdaddr[0], /* BLE MAC start */
			bdaddr[1],
			bdaddr[2],
			bdaddr[3],
			bdaddr[4],
			bdaddr[5]  /* BLE MAC stop */
	};

	hci_le_set_scan_resp_data(0, NULL);

	PRINTF("Set General Discoverable Mode.\n");

	//The device will be discoverable until the Host issue Aci_Gap_Set_Non_Discoverable command:
	ret = aci_gap_set_discoverable(
			ADV_IND,							//AdvType: Connectable undirected advertising
			(ADV_INTERVAL_MIN_MS*1000)/625,		//Minimum advertising interval
			(ADV_INTERVAL_MAX_MS*1000)/625,		//Maximum advertising interval
			STATIC_RANDOM_ADDR,					//Type of our address used during advertising (PUBLIC_ADDR,STATIC_RANDOM_ADDR).
			NO_WHITE_LIST_USE,					//Filter policy
			sizeof(local_name),					//Length of LocalName array.
			local_name,							//Array containing the Local Name AD data. First byte is the AD type
			0,									//Length of ServiceUUIDList array.
			NULL,								//ServiceUUIDList
			0,									//Slave connection interval minimum value suggested by Peripheral.
			0);									//Slave connection interval maximum value suggested by Peripheral.
	//-> If SlaveConnIntervMin and SlaveConnIntervMax are not 0x0000, Slave Connection Interval Range AD structure will be
	//added in advertising data. Connection interval is defined in the following manner:
	//connIntervalmin = Slave_Conn_Interval_Min x 1.25ms Slave_Conn_Interval_Min range:
	//0x0006 to 0x0C80 Value of 0xFFFF indicates no specific minimum.
	// ---> beide 0 lassen

	//Update advertising data:
	aci_gap_update_adv_data(
			LEN, 				//LEN
			manuf_data);		//Data Struct
#undef LEN						//Delete Symbol LEN; not further needed

	if(ret != BLE_STATUS_SUCCESS)
	{
		PRINTF("aci_gap_set_discoverable() failed: 0x%02x\r\n", ret);
	}
	else
		PRINTF("aci_gap_set_discoverable() --> SUCCESS\r\n");
}



/**
 * @brief  Add the Environment Service
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus LGS_AddServices(void)
{
  tBleStatus ret;
  int32_t NumberOfRecords=6;		// NUMBER OF RECORDS
  uint8_t uuid[16];

  //###################################################################################################################################
  //Add Service
  //###################################################################################################################################
  GET_ENVIRONMENT_SERVICE_UUID(uuid);
  memcpy(&service_uuid.Service_UUID_128, uuid, 16);		//Dest, Source, Length
  ret = aci_gatt_add_serv(
		  UUID_TYPE_128, 								//UUID Type: 16/128 bit
		  service_uuid.Service_UUID_128, 				//16-bit or 128-bit UUID based on the UUID Type field; requests const uint8_t *
		  PRIMARY_SERVICE,								//Primary or secondary service
          1+3*NumberOfRecords, 							//Maximum number of attribute records(including the service declaration itself)
		  &m_serviceHandles.m_environmentServiceHandle);//Service Handle
  if (ret != BLE_STATUS_SUCCESS)
  {
    goto fail;
  }

  //###################################################################################################################################
  //Add READONLY Characteristics to New Service
  //###################################################################################################################################

  //Bright Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_BRIGHT_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle,//Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentBright),			//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
          ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
          GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
          16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharBrightHandle);			//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  //Temperature Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_TEMP_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle, //Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentTemperature),		//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharTempHandle);			//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //VOC Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_VOC_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle, //Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentVOC),				//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharVOCHandle);			//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //CO2 Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_CO2_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle, //Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentCO2),				//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharCO2Handle);			//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //Humidity Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_HUMIDITY_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle, //Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentAirHumidity),		//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharHumidityHandle);		//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //Air Pressure Char
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_PRESSURE_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_environmentServiceHandle, //Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_environmentAirPressure),		//Länge
		  CHAR_PROP_NOTIFY,											//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_environmentCharPressureHandle);		//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }





  //###################################################################################################################################
  //Add CONFIGURATION Service
  //###################################################################################################################################
  NumberOfRecords = 7; //Records for CONFIGURATION SERVICE

  GET_CONFIGURATION_SERVICE_UUID(uuid);
  memcpy(&service_uuid.Service_UUID_128, uuid, 16);		//Dest, Source, Length
  ret = aci_gatt_add_serv(
		  UUID_TYPE_128, 									//UUID Type: 16/128 bit
		  service_uuid.Service_UUID_128, 					//16-bit or 128-bit UUID based on the UUID Type field; requests const uint8_t *
		  PRIMARY_SERVICE,									//Primary or secondary service
		  1+3*NumberOfRecords, 								//Maximum number of attribute records(including the service declaration itself)
		  &m_serviceHandles.m_configurationServiceHandle);	//Service Handle
  if (ret != BLE_STATUS_SUCCESS)
  {
	  goto fail;
  }

  //Repetition Rate
    //###################################################################################################################################
  	GET_ENVIRONMENT_CHAR_SETTINGS_REPRATE_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
  		  UUID_TYPE_128,											//UUID Type: 16/128 bit
  		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
  		  2+sizeof(m_environmentData.m_repRateBT),					//Länge
  		  CHAR_PROP_WRITE | CHAR_PROP_READ,							//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
  		  // Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
  		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
  		  GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
  		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
  		  // Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
  		  16,														//encryKeySize 7-16bit
  		  0,														//If the attribute has a variable length value field (1) or not (0).
  		  &m_serviceHandles.m_configurationsRepRateHandle);			//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
  	  goto fail;
    }

  //Critic Temperature
    //###################################################################################################################################
    GET_ENVIRONMENT_CHAR_SETTINGS_CRITTEMP_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
    		UUID_TYPE_128,											//UUID Type: 16/128 bit
			char_uuid.Char_UUID_128,								//Requests const uint8_t *
			2+sizeof(m_environmentData.m_criticTemperature),		//Länge
			CHAR_PROP_WRITE | CHAR_PROP_READ,						//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
			// Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
			ATTR_PERMISSION_NONE,									//Security permissions for the added characteristic
			GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
			// Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
			16,														//encryKeySize 7-16bit
			0,														//If the attribute has a variable length value field (1) or not (0).
			&m_serviceHandles.m_configurationsCriticTemperature);	//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
    	goto fail;
    }

    //Critic VOC
    //###################################################################################################################################
    GET_ENVIRONMENT_CHAR_SETTINGS_CRITVOC_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
    		UUID_TYPE_128,											//UUID Type: 16/128 bit
			char_uuid.Char_UUID_128,								//Requests const uint8_t *
			2+sizeof(m_environmentData.m_criticVOC),				//Länge
			CHAR_PROP_WRITE | CHAR_PROP_READ,						//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
			// Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
			ATTR_PERMISSION_NONE,									//Security permissions for the added characteristic
			GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
			// Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
			16,														//encryKeySize 7-16bit
			0,														//If the attribute has a variable length value field (1) or not (0).
			&m_serviceHandles.m_configurationsCriticVOC);			//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
    	goto fail;
    }

    //Critic CO2
    //###################################################################################################################################
    GET_ENVIRONMENT_CHAR_SETTINGS_CRITCO2_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
    		UUID_TYPE_128,											//UUID Type: 16/128 bit
			char_uuid.Char_UUID_128,								//Requests const uint8_t *
			2+sizeof(m_environmentData.m_criticCo2),				//Länge
			CHAR_PROP_WRITE | CHAR_PROP_READ,						//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
			// Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
			ATTR_PERMISSION_NONE,									//Security permissions for the added characteristic
			GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
			// Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
			16,														//encryKeySize 7-16bit
			0,														//If the attribute has a variable length value field (1) or not (0).
			&m_serviceHandles.m_configurationsCriticCo2);			//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
    	goto fail;
    }

    //Critic Humidity
    //###################################################################################################################################
    GET_ENVIRONMENT_CHAR_SETTINGS_CRITHUM_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
    		UUID_TYPE_128,											//UUID Type: 16/128 bit
			char_uuid.Char_UUID_128,								//Requests const uint8_t *
			2+sizeof(m_environmentData.m_criticHumidity),			//Länge
			CHAR_PROP_WRITE | CHAR_PROP_READ,						//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
			// Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
			ATTR_PERMISSION_NONE,									//Security permissions for the added characteristic
			GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
			// Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
			16,														//encryKeySize 7-16bit
			0,														//If the attribute has a variable length value field (1) or not (0).
			&m_serviceHandles.m_configurationsCriticHumidity);		//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
    	goto fail;
    }

    //Critic Pressure
    //###################################################################################################################################
    GET_ENVIRONMENT_CHAR_SETTINGS_CRITPRES_UUID(uuid);
    memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
    ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
    		UUID_TYPE_128,											//UUID Type: 16/128 bit
			char_uuid.Char_UUID_128,								//Requests const uint8_t *
			2+sizeof(m_environmentData.m_criticPressure),			//Länge
			CHAR_PROP_WRITE | CHAR_PROP_READ,						//Bitwise OR values of Characteristic Properties; def: CHAR_PROP_WRITE
			// Alternativ ggf: CHAR_PROP_WRITE_WITHOUT_RESP ?
			ATTR_PERMISSION_NONE,									//Security permissions for the added characteristic
			GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
			// Alternativ: GATT_NOTIFY_ATTRIBUTE_WRITE ?
			16,														//encryKeySize 7-16bit
			0,														//If the attribute has a variable length value field (1) or not (0).
			&m_serviceHandles.m_configurationsCriticPressure);		//Handle of the Characteristic that has been added.
    if (ret != BLE_STATUS_SUCCESS) {
    	goto fail;
    }

  //Output Active Flag
  //###################################################################################################################################
  GET_ENVIRONMENT_CHAR_SETTINGS_OUTPUTACT_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_configurationServiceHandle,//Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(m_environmentData.m_outputActive),				//Länge
		  CHAR_PROP_NOTIFY,											//CHAR_PROP_NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_configurationsOutputActive);			//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }




  //###################################################################################################################################
  //Add FSM DataRead Service
  //###################################################################################################################################
  NumberOfRecords = 3; //Records for CONFIGURATION SERVICE

  GET_FSM_READDATA_SERVICE_UUID(uuid);
  memcpy(&service_uuid.Service_UUID_128, uuid, 16);		//Dest, Source, Length
  ret = aci_gatt_add_serv(
		  UUID_TYPE_128, 									//UUID Type: 16/128 bit
		  service_uuid.Service_UUID_128, 					//16-bit or 128-bit UUID based on the UUID Type field; requests const uint8_t *
		  PRIMARY_SERVICE,									//Primary or secondary service
		  1+3*NumberOfRecords, 								//Maximum number of attribute records(including the service declaration itself)
		  &m_serviceHandles.m_fsmServiceHandle);			//Service Handle
  if (ret != BLE_STATUS_SUCCESS)
  {
	  goto fail;
  }

  //Request Message Android FSM
  //###################################################################################################################################
  GET_FSM_READDATA_CHAR_REQUEST_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_fsmServiceHandle,	//Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2+sizeof(uint8_t),										//Länge: 8bit Selector
		  CHAR_PROP_WRITE | CHAR_PROP_READ,							//READ/WRITE (Write: W-Request, Read: R-Request)
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP |
		  	  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,			//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_fsmRequestHandle);					//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //Notify Ready Paket for OP Android FSM
  //###################################################################################################################################
  GET_FSM_READDATA_CHAR_NOTIFYREADY_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_fsmServiceHandle,	//Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t *
		  2*sizeof(uint16_t),										//Länge: 16bit Anzahl Pakete (für Read-Prozess), 16bit Anzahl Daten pro Paket
		  CHAR_PROP_NOTIFY,											//NOTIFY
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_DONT_NOTIFY_EVENTS,									//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_fsmNotifyReadyHandle);				//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }

  //Datenpaket Android FSM
  //###################################################################################################################################
  GET_FSM_READDATA_CHAR_READPAKET_UUID(uuid);
  memcpy(&char_uuid.Char_UUID_128, uuid, 16); //Dest, Source, Length
  ret =  aci_gatt_add_char(m_serviceHandles.m_fsmServiceHandle,	//Handle of the service to which the characteristic has to be added.
		  UUID_TYPE_128,											//UUID Type: 16/128 bit
		  char_uuid.Char_UUID_128,									//Requests const uint8_t
		  FSM_VALUECOUNT_16BIT*sizeof(uint16_t),					//Länge: 10*2Byte | 20*1Byte #MAX ALLOWED: 512Byte (BLE Spec.)
		  CHAR_PROP_READ,											//Read
		  ATTR_PERMISSION_NONE,										//Security permissions for the added characteristic
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,				//Bit mask gattEvtMask
		  16,														//encryKeySize 7-16bit
		  0,														//If the attribute has a variable length value field (1) or not (0).
		  &m_serviceHandles.m_fsmReadPaketHandle);					//Handle of the Characteristic that has been added.
  if (ret != BLE_STATUS_SUCCESS) {
	  goto fail;
  }



  return BLE_STATUS_SUCCESS;

  //###################################################################################################################################
fail:
  return BLE_STATUS_ERROR;
}


/**
 * Sendet Notify, dass Read Process gestartet werden kann
 */
void LGS_BLE_IndicateReadProcessReady(void)
{
	//1: Temperatur lesen
	//2: Feuchte lesen
	//3: Druck lesen
	//4: Co2 lesen
	//5: Voc lesen
	if((m_requestedDataType == 1U) || (m_requestedDataType == 2U))
	{
		//8bit Wert requested
		m_indicatePaket.u16[0] = LGS_DATAMANAGEMENT_MAX_DATAELEMENTS / FSM_VALUECOUNT_8BIT; //Anzahl Pakete
		m_indicatePaket.u16[1] = FSM_VALUECOUNT_8BIT; //Anzahl Daten pro Paket
	}
	else
	{
		//16bit Wert requested
		m_indicatePaket.u16[0] = LGS_DATAMANAGEMENT_MAX_DATAELEMENTS / FSM_VALUECOUNT_16BIT; //Anzahl Pakete
		m_indicatePaket.u16[1] = FSM_VALUECOUNT_16BIT; //Anzahl Daten pro Paket
	}

	uint32_t transmitData = m_indicatePaket.u32;

	//Notify Process ready; Payload: Paket Count
	(void)aci_gatt_update_char_value(
			m_serviceHandles.m_fsmServiceHandle, 			//Service Handle
			m_serviceHandles.m_fsmNotifyReadyHandle,		//Characteristic Handle
			0, 												//Offset
			2*sizeof(uint16_t),								//LEN
			&transmitData);									//const void *  charValue

	m_fsmTimeoutTick = HAL_GetTick();	//Timeout initialisieren
}


/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void LGS_UserNotify(void * pData)
{
	hci_uart_pckt *hci_pckt = pData;
	/* obtain event packet */
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

	if(hci_pckt->type != HCI_EVENT_PKT)
		return;

	switch(event_pckt->evt)
	{
	case EVT_DISCONN_COMPLETE:
	{
		GAP_DisconnectionComplete_CB();
	}
	break;

	case EVT_LE_META_EVENT:
	{
		evt_le_meta_event *evt = (void *)event_pckt->data;

		switch(evt->subevent)
		{
		case EVT_LE_CONN_COMPLETE:
		{
			evt_le_connection_complete *cc = (void *)evt->data;
			GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
		}
		break;
		}
	}
	break;

	case EVT_VENDOR:
	{
		evt_blue_aci *blue_evt = (void*)event_pckt->data;
		switch(blue_evt->ecode)
		{
		// Read Request empfangen
		case EVT_BLUE_GATT_READ_PERMIT_REQ:
		{
			evt_gatt_read_permit_req *pr = (void*)blue_evt->data;
			Read_Request_CB(pr->attr_handle);
			break;
		}


		// Write Request empfangen
		case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
		{
			evt_gatt_write_permit_req *pr = (void*)blue_evt->data;
			Write_Request_CB(pr->attr_handle, pr->data_length, pr->data);
			break;
		}

		}

	}
	break;
	}
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{
  connected = TRUE;
  connection_handle = handle;

  PRINTF("Connected to device:");
  for(uint32_t i = 5; i > 0; i--){
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\n", addr[0]);
}


/*******************************************************************************
* Function Name  : Read_Request_CB.
* Description    : Update the sensor values.
* Input          : Handle of the characteristic to update.
* Return         : None.
*******************************************************************************/
void Read_Request_CB(uint16_t handle)
{
	tBleStatus ret;

	if(connection_handle !=0)
	{
		//Update nötig?
		if(m_serviceHandles.m_fsmReadPaketHandle == (handle - 1))
		{
			if(LGS_DATAMANAGEMENT_ISPROCESSACTIVE())
			{
				//Reset Timeout-StartTick
				m_fsmTimeoutTick = HAL_GetTick();

				//Paket fertig machen
				//1: Temperatur lesen
				//2: Feuchte lesen
				//3: Druck lesen
				//4: Co2 lesen
				//5: Voc lesen
				if(m_requestedDataType == 1U)
				{
					//Temperatur lesen
					int8_t data[FSM_VALUECOUNT_8BIT];
					for(uint16_t i = 0U; i < FSM_VALUECOUNT_8BIT; i++)
					{
						data[i] = (LGS_DATAMANAGEMENT_ReadNextDataElement())->m_temperature;
					}
					(void)aci_gatt_update_char_value(
							m_serviceHandles.m_fsmServiceHandle, 	//Service Handle
							m_serviceHandles.m_fsmReadPaketHandle,  //Characteristic Handle
							0, 										//Offset
							FSM_VALUECOUNT_8BIT*sizeof(int8_t), 	//LEN
							&data);									//const void *  charValue
				}
				else if(m_requestedDataType == 2U)
				{
					//Feuchte lesen
					uint8_t data[FSM_VALUECOUNT_8BIT];
					for(uint16_t i = 0U; i < FSM_VALUECOUNT_8BIT; i++)
					{
						data[i] = (LGS_DATAMANAGEMENT_ReadNextDataElement())->m_humidity;
					}
					(void)aci_gatt_update_char_value(
							m_serviceHandles.m_fsmServiceHandle, 	//Service Handle
							m_serviceHandles.m_fsmReadPaketHandle,  //Characteristic Handle
							0, 										//Offset
							FSM_VALUECOUNT_8BIT*sizeof(uint8_t), 	//LEN
							&data);									//const void *  charValue
				}
				else if(m_requestedDataType == 3U)
				{
					//Druck lesen
					uint16_t data[FSM_VALUECOUNT_16BIT];
					for(uint16_t i = 0U; i < FSM_VALUECOUNT_16BIT; i++)
					{
						data[i] = (LGS_DATAMANAGEMENT_ReadNextDataElement())->m_pressure;
					}
					(void)aci_gatt_update_char_value(
							m_serviceHandles.m_fsmServiceHandle, 	//Service Handle
							m_serviceHandles.m_fsmReadPaketHandle,  //Characteristic Handle
							0, 										//Offset
							FSM_VALUECOUNT_16BIT*sizeof(uint16_t), 	//LEN
							&data);									//const void *  charValue
				}
				else if(m_requestedDataType == 4U)
				{
					//Co2 lesen
					uint16_t data[FSM_VALUECOUNT_16BIT];
					for(uint16_t i = 0U; i < FSM_VALUECOUNT_16BIT; i++)
					{
						data[i] = (LGS_DATAMANAGEMENT_ReadNextDataElement())->m_co2;
					}
					(void)aci_gatt_update_char_value(
							m_serviceHandles.m_fsmServiceHandle, 	//Service Handle
							m_serviceHandles.m_fsmReadPaketHandle,  //Characteristic Handle
							0, 										//Offset
							FSM_VALUECOUNT_16BIT*sizeof(uint16_t), 	//LEN
							&data);									//const void *  charValue
				}
				else //m_requestedDataType MUSS 5 sein
				{
					//Voc lesen
					uint16_t data[FSM_VALUECOUNT_16BIT];
					for(uint16_t i = 0U; i < FSM_VALUECOUNT_16BIT; i++)
					{
						data[i] = (LGS_DATAMANAGEMENT_ReadNextDataElement())->m_voc;
					}
					(void)aci_gatt_update_char_value(
							m_serviceHandles.m_fsmServiceHandle, 	//Service Handle
							m_serviceHandles.m_fsmReadPaketHandle,  //Characteristic Handle
							0, 										//Offset
							FSM_VALUECOUNT_16BIT*sizeof(uint16_t), 	//LEN
							&data);									//const void *  charValue
				}
			}
		}
		else
		{
			//Für alle anderen Handles ist kein Update nötig, tue nichts
		}

		ret = aci_gatt_allow_read(connection_handle);
		if (ret != BLE_STATUS_SUCCESS)
		{
			PRINTF("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
		}
	}
}


/*******************************************************************************
* Function Name  : Write_Request_CB.
* Description    : Handle Write Request
* Input          : - Handle of the characteristic to write.
* 				   - Length of Data received
* 				   - Received Data
* Return         : None.
*******************************************************************************/
void Write_Request_CB(uint16_t handle, uint8_t data_length, uint8_t* data)
{
	tBleStatus ret;

	if(connection_handle !=0)
	{
		uint8_t write_status = ERR_WRITE_STATUS_OK;
		uint8_t error_code	 = ERR_CODE_NO_ERROR;

		if(m_serviceHandles.m_configurationsRepRateHandle == (handle - 1))
		{
			uint16_t receivedRepRate = *((uint16_t*)data);
			if((receivedRepRate <= LGS_CYCLIC_SAVE_INTERVAL_MAX)
					&& (receivedRepRate >= LGS_CYCLIC_SAVE_INTERVAL_MIN))
			{
				//OK
				m_environmentData.m_repRateBT = receivedRepRate;
				m_environmentData.m_isNewSaveIntervalAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_configurationsCriticTemperature == (handle - 1))
		{
			uint16_t receivedCritTemp = *((int8_t*)data);
			if((receivedCritTemp <= LGS_CYCLIC_CRITICTEMP_MAX)
					&& (receivedCritTemp >= LGS_CYCLIC_CRITICTEMP_MIN))
			{
				//OK
				m_environmentData.m_criticTemperature = receivedCritTemp;
				m_environmentData.m_isNewCriticDataAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_configurationsCriticVOC == (handle - 1))
		{
			uint16_t receivedCritVOC = *((uint16_t*)data);
			if((receivedCritVOC <= LGS_CYCLIC_CRITICVOC_MAX)
					&& (receivedCritVOC >= LGS_CYCLIC_CRITICVOC_MIN))
			{
				//OK
				m_environmentData.m_criticVOC = receivedCritVOC;
				m_environmentData.m_isNewCriticDataAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_configurationsCriticCo2 == (handle - 1))
		{
			uint16_t receivedCritCO2 = *((uint16_t*)data);
			if((receivedCritCO2 <= LGS_CYCLIC_CRITICCO2_MAX)
					&& (receivedCritCO2 >= LGS_CYCLIC_CRITICCO2_MIN))
			{
				//OK
				m_environmentData.m_criticCo2 = receivedCritCO2;
				m_environmentData.m_isNewCriticDataAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_configurationsCriticHumidity == (handle - 1))
		{
			uint16_t receivedCritHumidity = *((uint8_t*)data);
			if((receivedCritHumidity <= LGS_CYCLIC_CRITICHUMIDITY_MAX)
					&& (receivedCritHumidity >= LGS_CYCLIC_CRITICHUMIDITY_MIN))
			{
				//OK
				m_environmentData.m_criticHumidity = receivedCritHumidity;
				m_environmentData.m_isNewCriticDataAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_configurationsCriticPressure == (handle - 1))
		{
			uint16_t receivedCritPressure = *((uint16_t*)data);
			if((receivedCritPressure <= LGS_CYCLIC_CRITICPRESSURE_MAX)
					&& (receivedCritPressure >= LGS_CYCLIC_CRITICPRESSURE_MIN))
			{
				//OK
				m_environmentData.m_criticPressure = receivedCritPressure;
				m_environmentData.m_isNewCriticDataAvailable = 1U;
			}
			else
			{
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_VALUE_NOK;
			}
		}
		else if(m_serviceHandles.m_fsmRequestHandle == (handle - 1))
		{
			m_requestedDataType = *((uint8_t*)data);

			if(m_requestedDataType == 0U)
			{
				//Prozess abgeschlossen
				LGS_DATAMANAGEMENT_ReadProcessFinished();
			}
			else if((m_requestedDataType < 6U) && (m_requestedDataType > 0U))
			{
				//1: Temperatur lesen
				//2: Feuchte lesen
				//3: Druck lesen
				//4: Co2 lesen
				//5: Voc lesen
				LGS_DATAMANAGEMENT_StartReadProcess();
				m_fsmTimeoutTick = HAL_GetTick();
				m_fsmTickSendIndicateReady = HAL_GetTick();
			}
			else
			{
				//Prozess nicht definiert
				write_status = ERR_WRITE_STATUS_NOK;
				error_code	 = ERR_CODE_PROCESS_NOT_DEFINED;
			}
		}
		else
		{
			write_status = ERR_WRITE_STATUS_NOK;
			error_code = ERR_CODE_UNKNOWN_ATTR;
		}

		ret = aci_gatt_write_response(
				connection_handle,
				handle,
				write_status, 	//Write Status
				error_code,		//Fehlercode
				data_length,
				data
		);

		if (ret != BLE_STATUS_SUCCESS)
		{
			PRINTF("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
		}
	}
}

























