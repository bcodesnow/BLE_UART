#include <stdio.h>
#include "ble_uart.h"
//#include "main.h"

#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"

#include "mci_catch_detection.h"
#include "datalog_application.h"
#include "mci_hw_gpio.h"
#include "mci_hw_timer.h"


// todo: clean up this file!

uint8_t connected = FALSE;
static uint16_t connection_handle = 0;
int32_t helper_u32 = 0;

static t_ble_uart_struct ble_uart_struct;

uint16_t ble_UART_serv_Handle, TXCharHandle, RXCharHandle;

uint8_t* ptrToMainstate = NULL;
uint8_t copyToBuffer = FALSE;

static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);

// elminate this.. not needed..
void ble_uart_setPtrOfMainstate( uint8_t* state_ptr ) 
{
	ptrToMainstate = state_ptr;
}


void ble_poll( void )
{
	pollDelay = HAL_GetTick();
	uint32_t d = HAL_GetTick();  
  do
  {
		if(HCI_ProcessEvent)
		{					
			HCI_ProcessEvent=0;
			hci_user_evt_proc();
		}
    d=HAL_GetTick();
  }while( d - pollDelay < BLE_POLL_DELAY );	
}

#ifdef ACC_BLUENRG_CONGESTION
	#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
	static int32_t breath;

	/* @brief  Update the value of a characteristic avoiding (for a short time) to
	 *         send the next updates if an error in the previous sending has
	 *         occurred.
	 * @param  servHandle The handle of the service
	 * @param  charHandle The handle of the characteristic
	 * @param  charValOffset The offset of the characteristic
	 * @param  charValueLen The length of the characteristic
	 * @param  charValue The pointer to the characteristic
	 * @retval tBleStatus Status
	 */
	tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
																						 uint16_t charHandle,
																						 uint8_t charValOffset,
																						 uint8_t charValueLen,   
																						 const uint8_t *charValue)
	{
		tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
		
		if (breath > 0) 
		{
			breath--;
		} 
		else 
		{
			ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
			
			if (ret != BLE_STATUS_SUCCESS)
			{
				breath = ACC_BLUENRG_CONGESTION_SKIP;
			}
		}
		
		return (ret);
	}

#else /* ACC_BLUENRG_CONGESTION */
	#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */

tBleStatus ble_uart_tx (uint8_t *data, uint16_t len)
{
	/*
	tBleStatus aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
              const void *charValue)
	*/
	//return aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 0, len, data);
	return safe_aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 0, len, data);
	/* 
	Why +1?! Or Why Not?!
	The offset from which the attribute value has to be updated. If
	this is set to 0, and the attribute value is of variable length, then
	the length of the attribute will be set to the Char_Value_Length. If
	the Val_Offset is set to a value greater than 0, then the length of
	the attribute will be set to the maximum length as specified for
	the attribute while adding the characteristic.
	*/
}

void hci_reinit(void)
{
	hci_init(HCI_Event_CB, NULL);	
}

void init_blue_nrg_ms_stack(uint8_t mode_select)
{	
	uint8_t hwVersion = 0;
	uint16_t fwVersion = 0;	
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle = 0;
	
	uint8_t bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
	/* 0x6c 6f 66 61 73 7a*/
	uint8_t status = 0;

	hci_init(HCI_Event_CB, NULL);
	getBlueNRGVersion(&hwVersion, &fwVersion);
	// reset BlueNRG
	status = aci_gatt_init();
	//status = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);	

	// as gap_init_seems to overwrite the ADDR we need to set it after calling gap_init
	status = hci_le_set_random_address(bdaddr);


	if (mode_select == DISCOVERY_MODE)
	{
	  status = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

		status = aci_gap_start_general_discovery_proc(0x004 /*2.5ms */, 0x004 /* 2.5ms */, 0x01 /* private address */, 0x00 /* do not filter duplicates */ );
		//aci_gap_terminate_gap_procedure(
	
		static int debugval1 = 0;
		switch (status)
		{
			case 0x00: /* SUCCESS */
				debugval1++; /*just for the debugger to jump in */
				break;
			case 0x12: /* INVALID PARAM */
				debugval1--; /*just for the debugger to jump in */
				break;
			case 0x0C: /* ERR CMD DISALLOWED */
				debugval1--; /*just for the debugger to jump in */
				break;
		}
	}
	if (mode_select == ADV_MODE)
	{
		status = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
		const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'C','a','t','c','h','?','!' };
		/* As scan response data, a proprietary 128bits Service UUID is used.
		This 128bits data cannot be inserted within the advertising packet
		(ADV_IND) due its length constraints (31 bytes). */
		/* 
			AD Type description:
			0x11: length
			0x06: 128 bits Service UUID type
			0x8a,0x97,0xf7,0xc0,0x85,0x06,0x11,0xe3,0xba,0xa7,0x08,0x00,0x20,0x0c,0x9a,0x66: 128 bits Service UUID 
		*/
		uint8_t ServiceUUID_Scan[18]= {0x11,0x06,0x8a,0x97,0xf7,0xc0,0x85,0x06,0x11,0xe3,0xba,0xa7,0x08,0x00,0x20,0x0c,0x9a,0x66};
		/* Enable scan response to be sent when GAP peripheral receives
		scan requests from GAP Central performing general
		discovery procedure(active scan) */
			
		hci_le_set_scan_resp_data(18 , ServiceUUID_Scan);
		
		/* 
		Put the GAP peripheral in general discoverable mode:
		Advertising_Event_Type: ADV_IND (undirected scannable and connectable); / ADV_SCAN_IND (scannable but not connectable)
		Adv_Interval_Min: 0;
		Adv_Interval_Max: 0;
		Address_Type: PUBLIC_ADDR (public address: 0x00);
		Adv_Filter_Policy: NO_WHITE_LIST_USE (no whit list is used);
		Local_Name_Length: 8
		Local_Name: BlueNRG;
		Service_Uuid_Length: 0 (no service to be advertised);
		Service_Uuid_List: NULL;
		Slave_Conn_Interval_Min: 0 (Slave connection internal minimum value);
		Slave_Conn_Interval_Max: 0 (Slave connection internal maximum value).
		*/
            
    //HAL_Delay(2500);
		//status = aci_gap_set_discoverable(ADV_SCAN_IND, 0x0020, 0x0020, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
   		status = aci_gap_set_discoverable(ADV_SCAN_IND, 0x0001, 0x0001, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

    //GPIO_SET_PC0;
    //GPIO_SET_PC1;
	}
	
	
	if (mode_select == BLE_UART_MODE)
	{
		// changed to peripheral only
	  status = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
		const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'C','a','t','c','h','?','!' };
		
		const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
		const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
		const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
		
		aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &ble_UART_serv_Handle);
		
		// Add TX Characteristic with Notification property (The Central device gets notified)
		/*ServHandle, Char_UUID_Type, Char_UUID_16, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &CharHandle*/
		aci_gatt_add_char(ble_UART_serv_Handle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &TXCharHandle);
		
		// Set RX Cbaracteristic
		/* The Central can write it without resp, no security permissions(ATTR_PERMISSION_NONE), a GATT event mask (0x01), 
		16 as keyencryption size, and variable-length characteristic (1). 
		The characteristic handle is returned. */
		/*ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle)*/
		aci_gatt_add_char(ble_UART_serv_Handle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE, 16, 1, &RXCharHandle);
		//aci_gatt_update_char_value_ext_IDB05A1
		aci_hal_set_tx_power_level(1,4);
		// go discoverable (peripheral role, wait for a central device to connect)
		aci_gap_set_discoverable(ADV_IND, 0, 0, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,8, local_name, 0, NULL, 0,0);

	}
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

// MODIFY THE CONNECTION PARAMETERS	
	aci_l2cap_connection_parameter_update_request(handle,
                                              8 /* interval_min - 8 tested*/,
                                              8 /* interval_max - 8 tested */,
                                              0   /* slave_latency */,
                                              500 /*timeout_multiplier - 400 tested..*/);
	
  //sprintf( t_addr[0],"addr:%x:%x:%x:%x:%x:%x\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
  //ConnectionBleStatus=0;
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
}

/**
* @brief  This function is called when there is a change on the gatt attribute
* With this function it's possible to understand if one application 
* is subscribed or not to the one service
* @param uint16_t att_handle Handle of the attribute
* @param uint8_t *att_data attribute data
* @param uint8_t data_length length of the data
* @retval None
*/
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
	
}


/**
* @brief  This function is called when there is a Bluetooth Read request
* @param  uint16_t handle Handle of the attribute
* @retval None
*/
void Read_Request_CB(uint16_t handle)
{
//  if(handle == XXX )
//  {
//	}
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
* @brief  This function is called whenever there is an ACI event to be processed.
* @note   Inside this function each event must be identified and correctly
*         parsed.
* @param  void *pckt Pointer to the ACI packet
* @retval None
*/
void HCI_Event_CB(void *pckt)
{
	static uint32_t rec_ts; // received time stamp
	static uint8_t t_msg[8];

  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
  {
    return;
  }
  
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

			case EVT_LE_ADVERTISING_REPORT: /* BlueNRG-MS stack */
			{
				#ifdef SCANNING_TEST
				uint8_t found_devices_cnt = (uint8_t) *(evt->data); /* evt->data[0] is number of reports (On BlueNRG-MS is always max 1!) */
				le_advertising_info *pr = (void *)(evt->data+1); 
				uint8_t byte_wurst[18];
				for (int i=0; i<18; i++)
				{
					byte_wurst[i] = pr->data_RSSI[i]; /* just4debug */ 
				}
				byte_wurst[0] = 0x00;
                
                if (byte_wurst[5] == 'C' && byte_wurst[6] == 'a' && byte_wurst[7] == 't')
                {
                    GPIO_TOGGLE_PC0;
                    GPIO_TOGGLE_PC1;                   
                }
				/* le_advertising_info parameters:
				pr->evt_type: event type (advertising packets types);
				pr->bdaddr_type: type of the peer address (PUBLIC_ADDR,RANDOM_ADDR);
				pr->bdaddr: address of the peer device found during scanning;
				pr->length: length of advertising or scan response data;
				pr->data_RSSI[]: length advertising or scan response data + RSSI.
				RSSI is last octect (signed integer).
				*/
				/* Add user code for decoding the le_advertising_info event data based
				on the specific pr->evt_type (ADV_IND, SCAN_RSP, ..)*/
				#endif
			}

			break;
			}
    }
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode)
      {
				case EVT_BLUE_GATT_READ_PERMIT_REQ:
					{
						evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
						Read_Request_CB(pr->attr_handle);                    
					}
					break;
				case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
					{
						// WE RECEIVED STUFF
						evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
						if (copyToBuffer)
						{
							ble_uart_struct.len = evt->data_length;
							memcpy(&ble_uart_struct.rx_data[0], ( uint8_t *) evt->att_data, ble_uart_struct.len);
						}
						char helper_char;
						switch( *(evt->att_data) )
						{
							case TRIGGERED:
							 if ( *ptrToMainstate == CDSM_STATE_READY_TO_BE_TRIGGERED )
									// trig src -> evt->att_data[1]
							 		rec_ts = 0;
									rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[2] ) ) << 8;  
									rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[3] ) ) << 8;  
									rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[4] ) ) << 8;
									rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[5] ) ) << 8;
									//
									helper_u32 /* the trigger was X ms before! */ = get_diff_in_us_to_current_ts(rec_ts) / 1000;
							 		if ( helper_u32 > CD_POST_TRIGGER_BUFFERED_DATA_IN_MS )
									{
										*ptrToMainstate = CDSM_STATE_ERROR;
									}
									else
									{
										*ptrToMainstate = CDSM_STATE_TRIGGERED;										
									}
									//
									writeIndexWhenDataShouldBeSaved_MAG = ( magneto.writeIndex + ( (CD_POST_TRIGGER_BUFFERED_DATA_IN_MS - helper_u32 ) / 10 ) ) % R_BUFF_SIZE_100HZ;
									writeIndexCalculatedFlag_MAG = 1;
									//
									writeIndexWhenDataShouldBeSaved_ACC = ( accelero.writeIndex + (CD_POST_TRIGGER_BUFFERED_DATA_IN_MS - helper_u32 ) )  % R_BUFF_SIZE_1KHZ;
									writeIndexCalculatedFlag_MAG = 1;							 
								break;
							  //
							case STOP:
									*ptrToMainstate = CDSM_STATE_STOPPING;
								break;		
								//
							case START:
								if ( *ptrToMainstate == CDSM_STATE_STOPPED )
									*ptrToMainstate = CDSM_STATE_RESTARTING;
								break;
								//
							case IGNORE_LAST_X:
								// not implemented yet
								helper_char =  evt->att_data[1];
							  UNUSED(helper_char);
								break;
								//
							case WRITE_CATCH_SUCCESS:								
								if (DATALOG_SD_Init())
								{
									*ptrToMainstate = CDSM_STATE_ERROR;
									sm_struct.lastError = 0xFE;
									break;
								}	
								write_catch_success_to_sd( evt->att_data[1] );
								*ptrToMainstate = CDSM_STATE_RESTARTING;		
								deinitSD();
								break;
								//
							case GET_STATE:
								safe_aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 1, 1, ptrToMainstate);
								//aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 1, 1, ptrToMainstate);							
								break;
								//
							case REQUEST_SENSORDATA:
								if (sm_struct.subState == CDSM_SUBSTATE_HOLDING_DATA)
									sm_struct.subState = CDSM_SUBSTATE_SENDING_DATA_OVER_BLE;
								break;
								//
							case TURN_ON_SD_LOGGING:
								if (sm_struct.mainState == CDSM_STATE_STOPPED)
								{
									if (evt->att_data[1])
										sm_struct.isSDenabled = TRUE;
									else
										sm_struct.isSDenabled = FALSE;
								}
								break;
								//
							case TURN_ON_BLE_SENDING:
								if (sm_struct.mainState == CDSM_STATE_STOPPED)
								{
									if (evt->att_data[1])
										sm_struct.isSendingOverBLEenabled = TRUE;
									else
										sm_struct.isSendingOverBLEenabled = FALSE;
								}
								break;
								//
							case TS_MSG:
								switch ( evt->att_data[1] )
								{
									case TS_CMD_TIMESTAMP_IN_PAYLOAD:
										rec_ts = 0;
										rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[3] ) ) << 8;  
										rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[4] ) ) << 8;  
										rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[5] ) ) << 8;
										rec_ts = ( ( uint32_t ) ( rec_ts | evt->att_data[6] ) ) << 8;  

										set_ts_offset( get_us_time_stamp() - rec_ts );
									
									  t_msg[0] = TS_MSG;
										t_msg[1] = TS_CMD_KNOWN_PROCESSING_TIME_IN_PAYLOAD;
										t_msg[2] = evt->att_data[2]; // idx
										t_msg[3] = 0;
										t_msg[4] = 0;
										t_msg[5] = 0;
										t_msg[6] = 15; // some fake processing time..
										ble_uart_tx (&t_msg[0], 7);									
										break;
										//
									case TS_CMD_SYNC_START:
										if (sm_struct.mainState != CDSM_STATE_STOPPED)
												sm_struct.mainState = CDSM_STATE_STOPPING;				
										t_msg[0] = TS_MSG;
										t_msg[1] = TS_CMD_ACK; // idx
										ble_uart_tx (&t_msg[0], 2);				
										break;
										//
									case TS_CMD_LAST_ONE_WAS_GOOD_ONE:
										t_msg[0] = TS_MSG;
										t_msg[1] = TS_CMD_ACK; // idx
										ble_uart_tx (&t_msg[0], 2);			
										// Todo: add to states that we are synced!
										break;
										//
								}
								break;
								//
						}
					}
					break;
				case EVT_BLUE_GATT_PROCEDURE_TIMEOUT: 
				{
				}
				break;
				
				case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
					{
//					evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
						/*
						This event can be generated during a "Discover Characteristics By UUID" procedure or a
						"Read using Characteristic UUID" procedure						
						The attribute value will be a service declaration as defined in Bluetooth Core v4.1spec
						(vol.3, Part G, ch. 3.3.1), when a "Discover Characteristics By UUID" has been started. It will
						be the value of the Characteristic if a* "Read using Characteristic UUID" has been
						performed.
						*/
//						if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
//						{
//							tx_handle = resp->attr_handle;
//							PRINTF("TX Char Handle %04X\n", tx_handle);
//						}
//						else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
//						{
//							rx_handle = resp->attr_handle;
//							PRINTF("RX Char Handle %04X\n", rx_handle);
//						}
					}
					break;  
					
				case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
					{
						/* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
//						evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
						
//						if (APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
//						{
//							APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
//						}
//						else if (APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
//						{
//							APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
//						}
					}
					break;   
					
					/* New event available on BlueNRG-MS FW stack 7.1.b to know when there is a buffer available on GATT pool */
				case EVT_BLUE_GATT_TX_POOL_AVAILABLE:
					{                  
						/* We could use this to continue sending! */
					}
					break;
      }
    }
    break;
  }
}

// this blocks as hell
uint8_t ble_uart_tx_huge_chunk ( uint8_t* buff,	uint16_t bytesToWrite, uint8_t* maxRepeatCount)
{
	tBleStatus bs;
	uint16_t packet_count = bytesToWrite / CHAR_MAX_PAYLOAD;
	uint16_t i = 0;
	uint16_t k = 0;
	uint16_t bytes_in_last_package = bytesToWrite % packet_count;
	uint8_t mrc = 0;
	
	// send it
	for ( ; i< packet_count; i++ )
	{
		for ( k = 0; k < MAX_BLE_UART_TRIES; k++ )
		{
			bs = ble_uart_tx ( ( buff + ( i * CHAR_MAX_PAYLOAD ) ) , CHAR_MAX_PAYLOAD);
			
			if (bs == BLE_STATUS_SUCCESS)
				break;
					
			if(HCI_ProcessEvent)
			{					
				HCI_ProcessEvent=0;
				hci_user_evt_proc();
			}
			else
			{
				#ifdef USE_SAFETY_DELAY
					HAL_Delay(1);
				#endif
			}
		}
		if ( k > mrc)
			mrc = k;
	}
	// send last if remaining
	if ( bytes_in_last_package != 0)
	{
		for ( k = 0; k < MAX_BLE_UART_TRIES; k++ )
		{
			bs = ble_uart_tx ( ( buff + ( ( i + 1 ) * CHAR_MAX_PAYLOAD ) ) , bytes_in_last_package);
			
			if (bs == BLE_STATUS_SUCCESS)
				break;
					
			if(HCI_ProcessEvent)
			{					
				HCI_ProcessEvent=0;
				hci_user_evt_proc();
			}
			else
			{
				#ifdef USE_SAFETY_DELA
					HAL_Delay(1);
				#endif
			}
		}
		if ( k > mrc)
			mrc = k;		
	}
	*maxRepeatCount = mrc;
	
	// send a receive mode switch frame

	ble_uart_struct.tx_data[0] = SWITCH_RECEIVE_MODE;
	ble_uart_struct.tx_data[1] = 0x55;
	ble_uart_struct.tx_data[2] = 0xFF;
	ble_uart_struct.tx_data[3] = 0x55;
	ble_uart_struct.tx_data[4] = 0xFF;
	ble_uart_struct.tx_data[5] = 0x55;
	ble_uart_struct.tx_data[6] = MODE_CMD;

	for ( k = 0; k < MAX_BLE_UART_TRIES; k++ )
		{
			if ( ble_uart_tx (&ble_uart_struct.tx_data[0], 7) == BLE_STATUS_SUCCESS)
				return 0;
			else
			{
				if(HCI_ProcessEvent)
				{					
					HCI_ProcessEvent=0;
					hci_user_evt_proc();
				}
				#ifdef USE_SAFETY_DELAY
					HAL_Delay(1);
				#endif
			}
			if ( k > mrc)
				mrc = k;						
		}	
	return 0;
}

/* HUGE_CHUNK_START		-> byte[1] bytesToSend HBYTE 
											-> byte[2] bytesToSend LBYTE
											-> byte[3] TYPE
											-> byte[4] Used Charactertics as Channels (count)
*/
uint8_t ble_uart_tx_huge_chunk_start ( uint8_t type, uint16_t bytesToWrite, uint16_t arg2 )
{
	uint8_t k = 0;
	ble_uart_struct.tx_data[0] = HUGE_CHUNK_START;
	ble_uart_struct.tx_data[1] = ( bytesToWrite >> 8 ) & 0xFF;	
	ble_uart_struct.tx_data[2] = bytesToWrite & 0xFF;	
	ble_uart_struct.tx_data[3] = type;
	ble_uart_struct.tx_data[4] = ( arg2 >> 8 ) & 0xFF;
	ble_uart_struct.tx_data[5] = arg2 & 0xFF;
	
	for ( k = 0; k < MAX_BLE_UART_TRIES; k++ )
		{
			if ( ble_uart_tx (&ble_uart_struct.tx_data[0], 6) == BLE_STATUS_SUCCESS)
				return 0;
			else
			{
				if(HCI_ProcessEvent)
				{					
					HCI_ProcessEvent=0;
					hci_user_evt_proc();
				}
				#ifdef USE_SAFETY_DELAY
					HAL_Delay(1);
				#endif			
			}
		}
	return 0xFF;
}

/* HUGE_CHUNK_FINISH	-> byte[1] maxRepeatCount */
uint8_t ble_uart_tx_huge_chunk_finish ( uint8_t type, uint8_t repeatCount )
{
	uint8_t k = 0;
	ble_uart_struct.tx_data[0] = HUGE_CHUNK_FINISH;
	ble_uart_struct.tx_data[1] = repeatCount;	

	for ( k = 0; k < MAX_BLE_UART_TRIES; k++ )
		{
			if ( ble_uart_tx (&ble_uart_struct.tx_data[0], 2) == BLE_STATUS_SUCCESS)
				return 0;
			else
			{
				if(HCI_ProcessEvent)
				{						
					HCI_ProcessEvent=0;
					hci_user_evt_proc();
				}
				#ifdef USE_SAFETY_DELAY
					HAL_Delay(1);
				#endif
			}
		}
	return 0xFF;
	
}
