/*
 * lgs_datamanagement.c
 *
 *  Created on: 19.01.2019
 *      Author: TobXtreme
 */

#include "lgs_datamanagement.h"
#include "lgs_bluetooth.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_flash_ex.h"

//General:
typedef uint8_t BOOL;

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif


/**
 * ##############################################################################
 * 	EEPROM Positionen
 * 	##############################################################################
 */
#define EEPROM_POSITION_EEPROMINITIALIZED	0UL	//1 Byte Länge (Adr. 0)
#define EEPROM_POSITION_SAVEINTERVAL		4UL	//4 Byte Länge (Adr. 4,5,6,7)
#define EEPROM_POSITION_CURRENTDATAELEMENT	8UL	//2 Byte Länge (Adr. 8,9)
//Adressen 7-20 -> Frei; Reserve
#define EEPROM_POSITION_DATASTART			60UL	//Ab Position 60
#define EEPROM_POSITION_DATAEND				1976UL	//Bis inkl. Position 1979 (1976: letztes adressiertes WORD)
//Adressen 1941 - 2047 -> Frei; Reserve
/**
 * ##############################################################################
 * ##############################################################################
 */

//Hilfsfunktionen (Private)
uint8_t   readDataUint8(uint32_t Position);
uint16_t  readDataUint16(uint32_t Position);
uint32_t  readDataUint32(uint32_t Position);

void	  writeDataUint8(uint8_t data, uint32_t Position);
void	  writeDataUint16(uint16_t data, uint32_t Position);
void	  writeDataUint32(uint32_t data, uint32_t Position);

void 	  WriteNewDataElement();


//Daten:
s_dataElement m_dataElement; 		//Gelesenes Element
s_dataElement m_dataElementToWrite;	//Zu schreibendes Element
//Das letzte geschriebene Data Element (0-LGS_DATAMANAGEMENT_MAX_DATAELEMENTS):
uint16_t	  m_currentDataElementPosition;

BOOL	 	  m_readProcessActive;
uint16_t	  m_currentDataElementPositionReadProcess;
uint32_t 	  m_startTicksSaveData = 0U;
uint32_t	  m_saveInterval = 0xFFFFFFFFU;


/**
 * Hilfs-Union:
 */
union dataUnion
{
	uint32_t 	u32data;
	uint16_t	u16data	[2];
	uint8_t		u8data	[4];
	int8_t		s8data	[4];
}dataJoinUnion;

/*
 * Init Funktion;
 * -> Liest letzte Position, in die ein Datenelement geschrieben wurde
 *    (das aktuellste Datenelement)
 *
 * IN:
 * - uint32_t estimatedSaveFrequency
 *   -> Gibt Zeit in [1]ms an, nach der ein Element gespeichert wird
 *   -> Ändert sich der Wert, wird das EEPROM resetted
 */
void LGS_DATAMANAGEMENT_Init(uint32_t estimatedSaveInterval)
{
	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();	//Nicht sicher ob notwendig?

	//Gleiche SaveInterval und Initialized Flag mit dem gespeicherten Interval ab:
	if((readDataUint32(EEPROM_POSITION_SAVEINTERVAL) != estimatedSaveInterval) //Anderes Interval
			|| (readDataUint8(EEPROM_POSITION_EEPROMINITIALIZED) == 0U))	   //Noch nicht initialisiert
	{
		//Saveinterval nicht gleich oder noch nicht initialisiert, EEPROM Reset
		LGS_DATAMANAGEMENT_ClearData();

		//Konfigurationswerte speichern:
		writeDataUint8(1U, EEPROM_POSITION_EEPROMINITIALIZED); //Initialisiert Flag
		writeDataUint32(estimatedSaveInterval, EEPROM_POSITION_SAVEINTERVAL); //Saveinterval
		writeDataUint16(0U, EEPROM_POSITION_CURRENTDATAELEMENT); //CurrentDataElement - 0
		m_currentDataElementPosition = 0UL;						 //CurrentDataElement - 0
	}
	else
	{
		m_currentDataElementPosition = readDataUint16(EEPROM_POSITION_CURRENTDATAELEMENT);
	}

	m_readProcessActive = FALSE;
	m_startTicksSaveData = 0U;
	m_saveInterval = estimatedSaveInterval;
}

/**
 * Process-Funktion;
 * -> Speichert automatisch die aktuellen Daten weg
 */
void LGS_DATAMANAGEMENT_Process(void)
{
	if((HAL_GetTick() - m_startTicksSaveData) > m_saveInterval)
	{
		m_dataElementToWrite.m_temperature 	= m_environmentData.m_environmentTemperature;
		m_dataElementToWrite.m_humidity 	= m_environmentData.m_environmentAirHumidity;
		m_dataElementToWrite.m_pressure 	= m_environmentData.m_environmentAirPressure;
		m_dataElementToWrite.m_co2 			= m_environmentData.m_environmentCO2;
		m_dataElementToWrite.m_voc 			= m_environmentData.m_environmentVOC;

		WriteNewDataElement();
		m_startTicksSaveData = HAL_GetTick();
	}
}

/*
 * IsProcessActive-Funktion
 * -> Gibt Flag zurück, ob gerade ein Prozess aktiv ist
 */
uint8_t LGS_DATAMANAGEMENT_ISPROCESSACTIVE(void)
{
	return m_readProcessActive;
}

/*
 * ClearData Funktion
 * -> Löscht alle Daten aus dem EEPROM
 */
void LGS_DATAMANAGEMENT_ClearData(void)
{
	for(uint32_t pos = 0U; pos <= (DATA_EEPROM_END - DATA_EEPROM_BASE); pos = pos + 4)
	{
		HAL_FLASHEx_DATAEEPROM_Unlock();
		(void)HAL_FLASHEx_DATAEEPROM_Erase(DATA_EEPROM_BASE + pos);
		HAL_FLASHEx_DATAEEPROM_Lock();
	}
}

/*
 * StartReadProcess Funktion
 * -> Startet den Leseprozess
 */
void LGS_DATAMANAGEMENT_StartReadProcess(void)
{
	if(m_readProcessActive == FALSE) //Noch kein Leseprozess aktiv?
	{
		m_readProcessActive = TRUE;

		if((m_currentDataElementPosition + 1U) > LGS_DATAMANAGEMENT_MAX_DATAELEMENTS)
		{
			m_currentDataElementPositionReadProcess = 0U;
		}
		else
		{
			m_currentDataElementPositionReadProcess = m_currentDataElementPosition + 1U;
		}
	}
}
void LGS_DATAMANAGEMENT_ReadProcessFinished(void)
{
	m_readProcessActive = FALSE;
}

/*
 * ReadNextDataElement Funktion
 * -> Liest das nächste Datenelement vom EEPROM
 * 	  (Während eines Leseprozesses)
 */
s_dataElement* LGS_DATAMANAGEMENT_ReadNextDataElement(void)
{
	if(m_readProcessActive == TRUE)
	{
		//Elemente lesen
		uint32_t startPositionEEPROM =
				((uint32_t)m_currentDataElementPositionReadProcess) * sizeof(s_dataElement)
					+ EEPROM_POSITION_DATASTART;


		dataJoinUnion.u32data = readDataUint32(startPositionEEPROM);
		m_dataElement.m_temperature = dataJoinUnion.s8data[0];
		m_dataElement.m_humidity    = dataJoinUnion.u8data[1];
		m_dataElement.m_voc         = dataJoinUnion.u16data[1];

		dataJoinUnion.u32data = readDataUint32(startPositionEEPROM + 4);
		m_dataElement.m_co2 = 		  dataJoinUnion.u16data[0];
		m_dataElement.m_pressure =    dataJoinUnion.u16data[1];

		//Nächstes Element
		if((m_currentDataElementPositionReadProcess + 1U) > LGS_DATAMANAGEMENT_MAX_DATAELEMENTS)
		{
			m_currentDataElementPositionReadProcess = 0U;
		}
		else
		{
			m_currentDataElementPositionReadProcess++;
		}
	}
	else
	{
		//Vorgang wurde abgebrochen, mache nichts.
		m_dataElement.m_temperature = 0;
		m_dataElement.m_humidity = 0U;
		m_dataElement.m_pressure = 0U;
		m_dataElement.m_co2 = 0U;
		m_dataElement.m_voc = 0U;
	}

	return &m_dataElement; //Pointer auf Datenelement zurückgeben
}

/*
 * WriteNewDataElement Funktion
 * -> Schreibt ein neues Datenelement in das EEPROM
 */
void WriteNewDataElement()
{
	if(m_readProcessActive == FALSE)
	{
		//Nächstes Element:
		if((m_currentDataElementPosition + 1U) > LGS_DATAMANAGEMENT_MAX_DATAELEMENTS)
		{
			m_currentDataElementPosition = 0U;
		}
		else
		{
			m_currentDataElementPosition++;
		}
		//Aktuelle Position speichern:
		writeDataUint16(
				m_currentDataElementPosition,
				EEPROM_POSITION_CURRENTDATAELEMENT);


		//Schreibvorgang für das Datenelement:
		uint32_t startPositionEEPROM =
				((uint32_t)m_currentDataElementPosition) * sizeof(s_dataElement)
				+ EEPROM_POSITION_DATASTART;

		//2* 32bit schreiben
		//1: Temperatur, Feuchte, VOC:
		dataJoinUnion.s8data[0] = m_dataElementToWrite.m_temperature;
		dataJoinUnion.u8data[1] = m_dataElementToWrite.m_humidity;
		dataJoinUnion.u16data[1]= m_dataElementToWrite.m_voc;

		writeDataUint32(
				dataJoinUnion.u32data,
				startPositionEEPROM);

		//2: CO2, Druck
		dataJoinUnion.u16data[0]= m_dataElementToWrite.m_co2;
		dataJoinUnion.u16data[1]= m_dataElementToWrite.m_pressure;

		writeDataUint32(
				dataJoinUnion.u32data,
				startPositionEEPROM + 4);

	}
	else
	{
		//Leseprozess gerade aktiv, ignoriere Writebefehl
	}
}


/***
 * HILFSFUNKTIONEN
 */
uint8_t   readDataUint8(uint32_t Position)
{
	uint8_t readValue = 0xFFU;

	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		uint8_t *data = (uint8_t *)(DATA_EEPROM_BASE + Position);
		readValue = *data;
	}

	return readValue;
}
uint16_t  readDataUint16(uint32_t Position)
{
	uint16_t readValue = 0xFFFFU;

	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		uint16_t *data = (uint16_t *)(DATA_EEPROM_BASE + Position);
		readValue = *data;
	}

	return readValue;
}
uint32_t  readDataUint32(uint32_t Position)
{
	uint32_t readValue = 0xFFFFFFFFU;

	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		uint32_t *data = (uint32_t *)(DATA_EEPROM_BASE + Position);
		readValue = *data;
	}

	return readValue;
}
void writeDataUint8(uint8_t data, uint32_t Position)
{
	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		if(HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK)
		{
			(void)HAL_FLASHEx_DATAEEPROM_Program(
					FLASH_TYPEPROGRAMDATA_BYTE,
					(DATA_EEPROM_BASE + Position),
					data);
			(void)HAL_FLASHEx_DATAEEPROM_Lock();
		}
	}
}
void writeDataUint16(uint16_t data, uint32_t Position)
{
	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		if(HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK)
		{
			(void)HAL_FLASHEx_DATAEEPROM_Program(
					FLASH_TYPEPROGRAMDATA_HALFWORD,
					(DATA_EEPROM_BASE + Position),
					data);
			(void)HAL_FLASHEx_DATAEEPROM_Lock();
		}
	}
}
void writeDataUint32(uint32_t data, uint32_t Position)
{
	if(Position <= (DATA_EEPROM_END - DATA_EEPROM_BASE))
	{
		if(HAL_FLASHEx_DATAEEPROM_Unlock() == HAL_OK)
		{
			(void)HAL_FLASHEx_DATAEEPROM_Program(
					FLASH_TYPEPROGRAMDATA_WORD,
					(DATA_EEPROM_BASE + Position),
					data);
			(void)HAL_FLASHEx_DATAEEPROM_Lock();
		}
	}
}








