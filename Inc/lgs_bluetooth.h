/**
  ******************************************************************************
  * File Name          : lgs_bluetooth.h
  * Description        : Header file
  *
  *  Created on: 29.12.2018
  *  Author: Tobias Tiedemann
  ******************************************************************************
  */

#ifndef LGS_BLUETOOTH_H_
#define LGS_BLUETOOTH_H_

/**
 * Includes
 */
#include <stdint.h>

/**
 * Makros / Settings
 */
#define IDB04A1 0
#define IDB05A1 1	//Default Extension Board
#define LGS_MOD_NAME   'L','G','S','.','t','b','1','.','0'
#define BDADDR_SIZE        6
#define PA_LEVEL		   7 //(Power Output Level: 0-7)
#define ADV_INTERVAL_MIN_MS  	 			1000
#define ADV_INTERVAL_MAX_MS  	 			1200

#define ERR_CODE_UNKNOWN_ATTR				0x10
#define ERR_CODE_LENGTH_NOK					0x11
#define ERR_CODE_VALUE_NOK					0x12
#define	ERR_CODE_PROCESS_NOT_DEFINED		0x14
#define	ERR_CODE_OTHER_PROCESS_ACTIVE		0x15
#define ERR_CODE_NO_ERROR					0x0
#define ERR_WRITE_STATUS_OK					0x0
#define ERR_WRITE_STATUS_NOK				0x1

//Grenzwerte
#define LGS_CYCLIC_SAVE_INTERVAL_MAX		0xFFFF  //Viele viele Minuten
#define LGS_CYCLIC_SAVE_INTERVAL_MIN		1	    //1s
#define LGS_CYCLIC_CRITICTEMP_MAX			100		//100°C
#define LGS_CYCLIC_CRITICTEMP_MIN			-100	//-100°C
#define LGS_CYCLIC_CRITICVOC_MAX			10000 	//10.000 ppb
#define LGS_CYCLIC_CRITICVOC_MIN			0		//0 ppb
#define LGS_CYCLIC_CRITICCO2_MAX			10000 	//10.000 ppm
#define LGS_CYCLIC_CRITICCO2_MIN			0		//0 ppm
#define LGS_CYCLIC_CRITICHUMIDITY_MAX		101		//101%
#define LGS_CYCLIC_CRITICHUMIDITY_MIN		0		//0%
#define LGS_CYCLIC_CRITICPRESSURE_MAX		3000	//3000 mBar
#define LGS_CYCLIC_CRITICPRESSURE_MIN		0		//0mBar


//Sendeabstand:
#define LGS_CYCLIC_SEND_INTERVAL_DEFAULT 	3U    	//3s

//Defaultwerte:
#define LGS_DEFAULT_CRITIC_TEMPERATURE		30		//30°C
#define LGS_DEFAULT_CRITIC_VOC				3000	//3000ppb
#define LGS_DEFAULT_CRITIC_CO2				1000	//1000ppm
#define LGS_DEFAULT_CRITIC_HUMIDITY			90		//90%
#define LGS_DEFAULT_CRITIC_PRESSURE			3000	//3000mBar (kann nicht erreicht werden -> irrelevant)
#define LGS_DEFAULT_OUTPUT_ACTIVE			0		//Ausgang aktiv Flag per Default nicht aktiv


//FSM Konstanten:
#define FSM_TIMEOUT							20000					//20s
#define FSM_VALUECOUNT_16BIT				30U						//30 * 2 * 8bit = 480bit Payload
#define FSM_VALUECOUNT_8BIT					60U						//60 * 1 * 8bit = 480bit Payload
#define FSM_DELAYINDICATEREADY				20						//50ms


/**
 * Typedefs / Struct definitions
 */
/** Documentation for C union Service_UUID_t */
typedef union Service_UUID_t_s {
  uint16_t Service_UUID_16;
  uint8_t Service_UUID_128[16];
} Service_UUID_t;

/** Documentation for C union Char_UUID_t */
typedef union Char_UUID_t_s {
  uint16_t Char_UUID_16;
  uint8_t Char_UUID_128[16];
} Char_UUID_t;

/**
 * Function Prototypes
 */
void LGS_SetDiscoverableMode(void);
void LGS_UserNotify(void * pData);

void LGS_BLE_Init(void);
void LGS_BLE_Process(void);
void LGS_BLE_UpdateDefaultSettings(void);
void LGS_BLE_IndicateReadProcessReady(void);

void LGS_GenerateRandomData(void);

extern uint8_t Application_Max_Attribute_Records[];



/**
 * DATA STRUCT BLUETOOTH
 */
/**
 * Data
 */
//Sensor Data; save as struct
typedef struct
{
	//Data
	//Aktuelle Messdaten; Wird zyklisch ebenfalls in EEPROM gespeichert zum Abrufen
	uint8_t 	m_environmentBright; 		//Flag; Gibt an ob Umgebung hell (=1) oder dunkel (=0) ist
	int8_t  	m_environmentTemperature;	//Temperatur; Gemessen in [1]°C
	uint16_t 	m_environmentVOC;			//Volatile Organic Compound; Gemessen in [1]PPB
	uint16_t	m_environmentCO2;			//CO2; Gemessen in [1]PPM
	uint8_t		m_environmentAirHumidity;	//Luftfeuchte; Gemessen in [1]%
	uint16_t	m_environmentAirPressure;	//Luftdruck; gemessen in [1]mBar

	//Settings Data
	//Einstellbar über Bluetooth und in EEPROM gespeichert:
	uint16_t	m_repRateBT;				//Wiederholrate in [1]s, mit der die Daten gesendet werden
	uint8_t 	m_outputActive;				//Ausgang aktiv? ja(1), nein (0)
	int8_t		m_criticTemperature;		//Temperatur, über der der Ausgang geschaltet wird
	uint16_t	m_criticVOC;				//VOC Wert, über dem der Ausgang geschaltet wird
	uint16_t	m_criticCo2;				//Co2 Wert, über dem der Ausgang geschaltet wird
	uint8_t		m_criticHumidity;			//Feuchtigkeitswert, über dem der Ausgang geschaltet wird
	uint16_t	m_criticPressure;			//Druckwert, über dem der Ausgang geschaltet wird

	//Update?
	uint8_t 	m_isUpdateAvailable;		//Muss ein Update/Notify gemacht werden?
	uint8_t 	m_isNewCriticDataAvailable; //Muss in das EEPROM geschrieben werden?
	uint8_t		m_isNewSaveIntervalAvailable; //Hat sich das Speicherintervall geändert?
} s_environmentData;

extern s_environmentData m_environmentData; //Datenstruktur ist definiert in "lgs_bluetooth.c"



#endif /* LGS_BLUETOOTH_H_ */
