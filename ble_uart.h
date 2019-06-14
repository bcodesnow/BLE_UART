#ifndef __ble_uart_h
#define __ble_uart_h

#define USED_ON_MICRO 1

/* ------------------------------------------------------------------- */

#ifdef USED_ON_MICRO
	#include "bluenrg_types.h"
	#include "bluenrg_gatt_server.h"
	#include "bluenrg_gap.h"
	#include "string.h"
	#include "bluenrg_gap_aci.h"
	#include "bluenrg_gatt_aci.h"
	#include "hci_const.h"
	#include "bluenrg_hal_aci.h"
	#include "bluenrg_aci_const.h"   
	#include "hci.h"
	#include "hci_le.h"
	#include "sm.h"
	#include <stdlib.h>

	/* For enabling the capability to handle BlueNRG Congestion */
	#define ACC_BLUENRG_CONGESTION
	#ifdef ACC_BLUENRG_CONGESTION
	/* For defining how many events skip when there is a congestion */
	#define ACC_BLUENRG_CONGESTION_SKIP 30
	#endif /* ACC_BLUENRG_CONGESTION */

	/* Stack Init Modes:*/
	#define DISCOVERY_MODE 		0x01
	#define	ADV_MODE			 		0x02
	#define BLE_UART_MODE 		0x03
	#define USE_SAFETY_DELAY 	1
	#define MAX_BLE_UART_TRIES 255

#endif // USED ON MICRO

/* ------------------------------------------------------------------- */

#define CHAR_MAX_PAYLOAD 20

/* Simple Protocol: */
// todo: only one cmd id for each "topic"

/*1st byte:
	CMD BYTE
**/
#define TRIGGERED                       0x06
#define GET_STATE                       0x03
#define STOP				  									0x02
#define START				  									0x01
#define IGNORE_LAST_X                   0x07 // not implemented yet
#define DATA_COLLECTED                  0x08 // used when data is saved to sd..
#define WRITE_CATCH_SUCCESS             0x09

#define REQUEST_SENSORDATA              0x0A // available when data is in ram
#define SENDING_SENSORDATA_FINISHED     0x0B // emitted when all the files were sent
#define SENSORDATA_AVAILABLE						0x0C // emitted wehn data can be requested.

#define ALIVE                           0x10
#define HUGE_CHUNK_START                0x77
#define	HUGE_CHUNK_FINISH               0x88
#define SWITCH_RECEIVE_MODE             0x99

#define TURN_ON_SD_LOGGING              0x66
#define TURN_ON_BLE_SENDING             0x55

#define TS_MSG                          0xAA

/* ------------------------------------------------------------------- */

/* TS_MSG                               -> byte[1] CMD
   TIME SYNC MSG FROM SERVER OR CLIENT  -> byte[2] timeStampOnServer / knownProcessingTime
                                        -> byte[3] timeStampOnServer / knownProcessingTime
                                        -> byte[4] timeStampOnServer / knownProcessingTime
                                        -> byte[5] timeStampOnServer / knownProcessingTime
*/

#define TS_CMD_TIMESTAMP_IN_PAYLOAD             	1u
#define TS_CMD_LAST_ONE_WAS_GOOD_ONE            	2u
#define TS_CMD_ACK                              	3u
#define TS_CMD_KNOWN_PROCESSING_TIME_IN_PAYLOAD 	4u
#define TS_CMD_SYNC_START                       	5u
#define TS_CMD_SYNC_FINISH                      	6u

/* IGNORE_LAST_X 		-> byte[1] how many to ignore */

/* ALIVE 							-> byte[1] mainState 
											-> byte[2] subState
											-> byte[3] fileIndex
											-> byte[4] lastError
											-> byte[5] sdOn
											-> byte[6] bleTransferOn
*/

/* HUGE_CHUNK_START		-> byte[1] bytesToSend HBYTE 
											-> byte[2] bytesToSend LBYTE
											-> byte[3] TYPE
											-> byte[4] writePointer HBYTE
											-> byte[5] writePointer LBYTE
											-> byte[6] Used Charactertics as Channels (count)
*/

#define TYPE_AUD 1u
#define	TYPE_ACC 2u
#define TYPE_GYR 3u
#define TYPE_MAG 4u
#define TYPE_PRS 5u

/* HUGE_CHUNK_FINISH	-> byte[1] maxRepeatCount */

/* SWITCH_RECEIVE_MODE		-> byte[1] 0x55
													-> byte[2] 0xFF
													-> byte[3] 0x55
													-> byte[4] 0xFF
													-> byte[4] 0x55											
													-> byte[5] MODE
*/

#define MODE_CMD 1u
#define MODE_HUGE_CHUNK 2u

/* TURN_ON_SD_LOGGING	-> byte[1] ON / OFF */


/* ------------------------------------------------------------------- */

#ifdef USED_ON_MICRO
	#define BLE_POLL_DELAY 8
	
	typedef struct t_ble_uart_struct
	{
		uint8_t state;
		uint32_t len;
		uint8_t rx_data[20];
		uint8_t tx_data[20];
	} t_ble_uart_struct;

	extern void init_blue_nrg_ms_stack(uint8_t mode_select);
	extern void ble_uart_setPtrOfMainstate( uint8_t* state_ptr );
	extern void HCI_Event_CB(void *pckt);
	extern uint8_t connected;
	extern volatile uint32_t pollDelay;
	tBleStatus ble_uart_tx (uint8_t *data, uint16_t len);
	extern void hci_reinit(void);
	extern void ble_poll( void );
	extern uint8_t ble_uart_tx_huge_chunk ( uint8_t* buff,	uint16_t bytesToWrite, uint8_t* maxRepeatCount );
	extern uint8_t ble_uart_tx_huge_chunk_start ( uint8_t type, uint16_t bytesToWrite, uint16_t arg2 );
	extern uint8_t ble_uart_tx_huge_chunk_finish ( uint8_t type, uint8_t repeatCount );
#endif
	
/* ------------------------------------------------------------------- */

#endif
