/**
******************************************************************************
* @project Catch Detection
* @author  kB
******************************************************************************
*/

/* Includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "datalog_application.h"
#include "mci_catch_detection.h"
#include "mci_hw_timer.h"
#include "mci_hw_gpio.h"
#include "ble_uart.h"
#include "SensorTile_audio.h" 
#include "SensorTile.h"

/* FatFs */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

BSP_AUDIO_Init_t MicParams;

r_buff_axes_100hz magneto;
r_buff_axes_1khz accelero;
r_buff_axes_1khz gyro;
r_buff_float_100hz pressure;
r_buff_audio audio;

volatile uint32_t timeStampWhenTriggered_ACC = 0;
volatile uint32_t timeStampWhenTriggered_MAG = 0;
volatile uint16_t writeIndexWhenDataShouldBeSaved_MAG = 0;
volatile uint16_t writeIndexWhenDataShouldBeSaved_ACC = 0;
volatile uint8_t writeIndexCalculatedFlag_MAG = 0;
volatile uint8_t writeIndexCalculatedFlag_ACC = 0;

volatile uint8_t triggerSource = 0;
static int32_t helper_u32 = 0;

static uint32_t errorDelay, aliveDelay;
volatile uint32_t pollDelay;
catch_detection_state_machine_t sm_struct;

static uint8_t t_msg[8];
const FIL emptyFIL;
FIL helperFIL;
uint8_t maxRepeatCount;


int main( void )
{

  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

	/* Init Bluetooth */
	ble_uart_setPtrOfMainstate( &sm_struct.mainState );
  init_blue_nrg_ms_stack(BLE_UART_MODE);

  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  /* Initialize and enable the available sensors */
  init_sensors();
	
	init_gpio_leds();
	
  /* Configure Audio Input peripheral - DFSDM */  
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = AUDIO_CHANNELS;
  MicParams.Device = AUDIO_IN_DIGITAL_MIC;
  MicParams.SampleRate = AUDIO_SAMPLING_FREQUENCY;
  MicParams.Volume = AUDIO_VOLUME_INPUT;
  BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams); 

	sm_struct.mainState = CDSM_STATE_INIT;
	sm_struct.isSendingOverBLEenabled = TRUE;

	/* Prepare to launch */
	init_tim3(); //512ms
	init_tim4(); //1ms
 	init_tim5(); //100ms TODO: only init them here and move the start to state init them here
	stop_all_timers();
	init_time_stamp();
	
	aliveDelay = HAL_GetTick();
	errorDelay = HAL_GetTick();

	while (1)
	{	
		switch ( sm_struct.mainState )
		{
			case CDSM_STATE_INIT:
				if ( BSP_AUDIO_IN_Record( BSP_AUDIO_IN_INSTANCE, (uint8_t *) PCM_Buffer, AUDIO_IN_BUFFER_SIZE ) ) /* Start Microphone acquisition */
				{
					sm_struct.mainState = CDSM_STATE_ERROR;
					sm_struct.lastError = 0x10;
					break;
				}
				if (sm_struct.isSDenabled)
				{
					SD_IO_CS_Init(); // Only CS Pin Config
					if (DATALOG_SD_Init())
					{
						sm_struct.mainState = CDSM_STATE_ERROR;
						sm_struct.lastError = 0x11;
						break;
					}
					if ( sm_struct.currentFileIndex == 0 )
					{
						// fresh boot
						if (determinate_current_log_file_index ( &helperFIL, "AUD_N", AUDIO ) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x12;
							break;					
						}
					}
					else
					{
						// just handling an error
						sm_struct.currentFileIndex++;
					}
				}
				start_all_timers();
				sm_struct.mainState = CDSM_STATE_RUNNING;
				break;
				//
			case CDSM_STATE_RUNNING:
				break;
				//
			case CDSM_STATE_READY_TO_BE_TRIGGERED:
				errorDelay = HAL_GetTick();
				break;
				//
			case CDSM_STATE_TRIGGERED:
				if(HAL_GetTick() - errorDelay > MAX_MS_IN_STATE)
				{
					sm_struct.mainState = CDSM_STATE_ERROR;
					sm_struct.lastError = 0x13;
				}
				break;
				//
			case CDSM_STATE_POST_TRIGGER_DATA_COLLECTED:				
				switch (sm_struct.subState)
				{					
					case CDSM_SUBSTATE_STOPPING:
						BSP_AUDIO_IN_Pause(BSP_AUDIO_IN_INSTANCE);
						stop_all_timers();
					
						if ( sm_struct.isSDenabled )
						{
							if (DATALOG_SD_Init())
							{
								sm_struct.mainState = CDSM_STATE_ERROR;
								sm_struct.lastError = 0x11;
								break;
							}
							if ( sm_struct.isSendingOverBLEenabled )
								sm_struct.subState = CDSM_SUBSTATE_HOLDING_DATA; //skip the holding data and sending over ble states..
							else
								sm_struct.subState = CDSM_SUBSTATE_SAVING_DATA_TO_SD;
						}
						else
						{
							sm_struct.subState = CDSM_SUBSTATE_ENTERING_HOLDING_DATA;
						}							
						break;
						//
					case CDSM_SUBSTATE_ENTERING_HOLDING_DATA:
						sm_struct.notifyHoldingData = TRUE;
						sm_struct.subState = CDSM_SUBSTATE_HOLDING_DATA;
						break;
						//
					case CDSM_SUBSTATE_HOLDING_DATA:				
						// holding data, it can be requested	
						break;
						//
					case CDSM_SUBSTATE_SENDING_DATA_OVER_BLE:						
						if ( connected )
						{
							ble_uart_tx_huge_chunk_start( TYPE_AUD, (uint16_t) ( R_BUFF_SIZE_AUDIO*2 ), audio.writeIndex);
							ble_poll();
							ble_uart_tx_huge_chunk ( (uint8_t*) &audio.buff[0],  (uint16_t) ( R_BUFF_SIZE_AUDIO*2 ), &maxRepeatCount );
							ble_poll();
							ble_uart_tx_huge_chunk_finish(TYPE_AUD, maxRepeatCount);
							ble_poll();
							//
							ble_uart_tx_huge_chunk_start( TYPE_MAG, R_BUFF_SIZE_100HZ*6, magneto.writeIndex);
							ble_poll();
							ble_uart_tx_huge_chunk ( (uint8_t*) &magneto.buff[0], R_BUFF_SIZE_100HZ*6, &maxRepeatCount );
							ble_poll();
							ble_uart_tx_huge_chunk_finish(TYPE_MAG, maxRepeatCount);
							ble_poll();
							//
							ble_uart_tx_huge_chunk_start( TYPE_PRS, R_BUFF_SIZE_100HZ*4, pressure.writeIndex);
							ble_poll();
							ble_uart_tx_huge_chunk ( (uint8_t*) &pressure.buff[0], R_BUFF_SIZE_100HZ*4, &maxRepeatCount );
							ble_poll();
							ble_uart_tx_huge_chunk_finish(TYPE_PRS, maxRepeatCount);
							ble_poll();
							//
							ble_uart_tx_huge_chunk_start( TYPE_ACC, R_BUFF_SIZE_1KHZ*6, accelero.writeIndex);
							ble_poll();
							ble_uart_tx_huge_chunk ( (uint8_t*) &accelero.buff[0], R_BUFF_SIZE_1KHZ*6, &maxRepeatCount );
							ble_poll();
							ble_uart_tx_huge_chunk_finish(TYPE_ACC, maxRepeatCount);
							ble_poll();
							//
							ble_uart_tx_huge_chunk_start( TYPE_GYR, R_BUFF_SIZE_1KHZ*6, gyro.writeIndex);
							ble_poll();
							ble_uart_tx_huge_chunk ( (uint8_t*) &gyro.buff[0], R_BUFF_SIZE_1KHZ*6, &maxRepeatCount );
							ble_poll();
							ble_uart_tx_huge_chunk_finish(TYPE_GYR, maxRepeatCount);
							ble_poll();

							t_msg[0] = SENDING_SENSORDATA_FINISHED;
							t_msg[1] = 0xFF;
							ble_uart_tx (&t_msg[0], 2);
						}
						if (sm_struct.isSDenabled)
							sm_struct.subState = CDSM_SUBSTATE_SAVING_DATA_TO_SD;
						else
							sm_struct.mainState = CDSM_STATE_STOPPING;
						break;
						//
					case CDSM_SUBSTATE_SAVING_DATA_TO_SD:
						helperFIL = emptyFIL;
						if ( create_log_file(&helperFIL, "AUD_N", AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x01;
							break;
						}
						if ( write_audio_r_buff_to_sd(&helperFIL, &audio, CONSIDER_TRIGGER, DELAY_IN_BETWEEN) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x02; //FAILS
							break;
						}
						if ( close_log_file(&helperFIL, AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x03;
							break;
						}
						//
						helperFIL = emptyFIL;
						if ( create_log_file(&helperFIL, "MAG_N", NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x04;
							break;
						}	
						if ( write_axes_100hz_r_buff_to_sd(&helperFIL, &magneto, CONSIDER_TRIGGER, DELAY_IN_BETWEEN) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x05;			
							break;
						}
						if ( close_log_file(&helperFIL, NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x06;
							break;							
						}
						//
						helperFIL = emptyFIL;
						if ( create_log_file(&helperFIL, "PRS_N", NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x07;
							break;
						}	
						if ( write_axes_100hz_r_buff_float_to_sd(&helperFIL, &pressure, CONSIDER_TRIGGER, DELAY_IN_BETWEEN) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x08;
							break;
						}					
						if ( close_log_file(&helperFIL, NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x09;
							break;
						}
						//
						helperFIL = emptyFIL;
						if ( create_log_file(&helperFIL, "ACC_N", NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0A; //FAILS
							break;
						}					
						if ( write_axes_1khz_r_buff_to_sd(&helperFIL, &accelero, CONSIDER_TRIGGER, DELAY_IN_BETWEEN) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0B; // FAILS
							break;
						}					
						if ( close_log_file(&helperFIL, NOT_AUDIO ) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0C;
							break;
						}
						//
						helperFIL = emptyFIL;
						if ( create_log_file(&helperFIL, "GYR_N", NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0D; // FAILS
							break;							
						}							
						if ( write_axes_1khz_r_buff_to_sd(&helperFIL, &gyro, CONSIDER_TRIGGER, DELAY_IN_BETWEEN) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0E;
							break;
						}					
						if ( close_log_file(&helperFIL, NOT_AUDIO) )
						{
							sm_struct.mainState = CDSM_STATE_ERROR;
							sm_struct.lastError = 0x0F;
							break;
						}
						sm_struct.subState++;																			
						break;
						//						
					case CDSM_SUBSTATE_SENDING_DATA_COLLECTED:
						deinitSD(); // the state is only reached if sd is enabled in the sm struct
						if (connected)
						{
							t_msg[0] = DATA_COLLECTED;
							t_msg[1] = sm_struct.currentFileIndex;
							ble_uart_tx (&t_msg[0], 2);
						}
						sm_struct.mainState = CDSM_STATE_STOPPING;
						break;
						//
					default:
						sm_struct.mainState = CDSM_STATE_ERROR;
						break; //technically not needed..
						//
				} // subState switch ends
			break;
			//
  		case CDSM_STATE_STOPPING:
					stop_all_timers();
					set_all_buffer_pointers_to_zero();
					sm_struct.mainState = CDSM_STATE_STOPPED;
					sm_struct.subState = CDSM_SUBSTATE_STOPPING;
			break;
			//
			case CDSM_STATE_STOPPED:
				// wait for start or wait for confirmation
				// Test -> sm_struct.mainState = CDSM_STATE_RESTARTING;
				break;
			//
			case CDSM_STATE_RESTARTING:
				start_all_timers();
				BSP_AUDIO_IN_Resume(BSP_AUDIO_IN_INSTANCE);
				sm_struct.currentFileIndex++;
				sm_struct.mainState = CDSM_STATE_RUNNING;
				break;
			//	
			case CDSM_STATE_ERROR:
				stop_all_timers();
				BSP_AUDIO_IN_Stop(BSP_AUDIO_IN_INSTANCE);
				set_all_buffer_pointers_to_zero();
				if (sm_struct.isSDenabled)
					deinitSD();
				sm_struct.subState = CDSM_SUBSTATE_STOPPING;
				sm_struct.mainState = CDSM_STATE_INIT;
				break;
			//
		} // mainState switch ends;
		//
		//
		if(HCI_ProcessEvent)
		{					
			HCI_ProcessEvent=0;
			hci_user_evt_proc();
		}		
		//
		if ( sm_struct.notifyTriggered && connected )
		{
			// notify the central about it -> this blocks possibly too long to make this work in the isr..
			t_msg[0] = TRIGGERED;
			t_msg[1] = sm_struct.currentFileIndex;
			if ( triggerSource == MAGNETO_TRIG )
			{
				helper_u32 = timeStampWhenTriggered_MAG;
			}
			else
			{
				helper_u32 = timeStampWhenTriggered_ACC; // accelero trigger
			}
			t_msg[2] = ( helper_u32 >> 24 ) & 0xFF ; 
			t_msg[3] = ( helper_u32 >> 16 ) & 0xFF ;
			t_msg[4] = ( helper_u32 >> 8 ) & 0xFF ;
			t_msg[5] = helper_u32 & 0xFF;
			t_msg[6] = triggerSource;

			if ( ble_uart_tx (&t_msg[0], 2) == BLE_STATUS_SUCCESS) // if the buffer is full.. BLE_STATUS_SUCCESS
				sm_struct.notifyTriggered = 0;
		}
		//
		if ( sm_struct.notifyHoldingData && connected )
		{
			t_msg[0] = SENSORDATA_AVAILABLE;
			t_msg[1] = 0xFF;;
			if ( ble_uart_tx (&t_msg[0], 2) == BLE_STATUS_SUCCESS)
				sm_struct.notifyHoldingData = 0;
		}
		//					
		if(HAL_GetTick() - aliveDelay > ALIVE_MSG_PERIOD)
		{
			aliveDelay = HAL_GetTick();									
			if ( connected )
			{
				// send out an alive message every X sec
				t_msg[0] = ALIVE;
				t_msg[1] = sm_struct.mainState;
				t_msg[2] = sm_struct.subState;
				t_msg[3] = sm_struct.currentFileIndex;
				t_msg[4] = sm_struct.lastError;
				t_msg[5] = sm_struct.isSDenabled;
				t_msg[6] = sm_struct.isSendingOverBLEenabled;
				ble_uart_tx (&t_msg[0], 7);
			}
		}
	} 			
//
}
