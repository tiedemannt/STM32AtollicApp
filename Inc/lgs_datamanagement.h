/*
 * lgs_datamanagement.h
 *
 *  Created on: 19.01.2019
 *      Author: TobXtreme
 */

#ifndef LGS_DATAMANAGEMENT_H_
#define LGS_DATAMANAGEMENT_H_


#include <stdint.h>


#define LGS_DATAMANAGEMENT_MAX_DATAELEMENTS	240U  	//Maximal 240*8Byte = 1920Byte -> Sollte für FSM ein vielfaches von 20 sein

#define DATAMANAGEMENT_INTERVAL_TEST 10000U			//Test: Alle 10s Wert speichern
#define DATAMANAGEMENT_INTERVAL_10PERHOUR 360000U	//Alle 6min = 10/h


/**
 * Dataelement Struct (64bit Size)
 */
typedef struct
{
	//Data Elements:
	int8_t 		m_temperature;
	uint8_t		m_humidity;
	uint16_t	m_voc;
	uint16_t	m_co2;
	uint16_t	m_pressure;
} s_dataElement;

/*
 * Public Functions
 */
void LGS_DATAMANAGEMENT_Init(uint32_t estimatedSaveInterval);
void LGS_DATAMANAGEMENT_ClearData(void);
void LGS_DATAMANAGEMENT_Process(void);

//Funktionen für Lesen
void 			LGS_DATAMANAGEMENT_StartReadProcess(void);
void 			LGS_DATAMANAGEMENT_ReadProcessFinished(void);
s_dataElement* 	LGS_DATAMANAGEMENT_ReadNextDataElement(void);
uint8_t			LGS_DATAMANAGEMENT_ISPROCESSACTIVE(void);

#endif /* LGS_DATAMANAGEMENT_H_ */
