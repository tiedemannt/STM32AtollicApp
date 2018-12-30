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
#define ADV_INTERVAL_MIN_MS  	 1000
#define ADV_INTERVAL_MAX_MS  	 1200
#define LGS_CYCLIC_SEND_INTERVAL 500


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

extern uint8_t Application_Max_Attribute_Records[];

#endif /* LGS_BLUETOOTH_H_ */
