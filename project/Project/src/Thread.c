
#include "cmsis_os.h"  // CMSIS RTOS header file
#include "Board_LED.h"
#include "UART_driver.h"
#include "stdint.h"                     // data type definitions
#include "stdio.h"                      // file I/O functions
#include "rl_usb.h"                     // Keil.MDK-Pro::USB:CORE
#include "rl_fs.h"                      // Keil.MDK-Pro::File System:CORE
#include "stm32f4xx_hal.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "math.h"
#include "arm_math.h" // header for DSP library
#include <stdio.h>

// LED constants
#define LED_Green   0
#define LED_Orange  1
#define LED_Red     2
#define LED_Blue    3

#define NUM_CHAN 2 																// number of audio channels
#define NUM_POINTS 1024 														// number of points per channel
#define BUF_LEN NUM_CHAN*NUM_POINTS 											// length of the audio buffer

#define Show_Files_char "1"
#define Play_File_char "4"
#define Stop_File_char "5"



// WAVE file header format
typedef struct WAVHEADER {
	unsigned char riff[4];						// RIFF string
	uint32_t overall_size;				// overall size of file in bytes
	unsigned char wave[4];						// WAVE string
	unsigned char fmt_chunk_marker[4];		// fmt string with trailing null char
	uint32_t length_of_fmt;					// length of the format data
	uint16_t format_type;					// format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
	uint16_t channels;						// no.of channels
	uint32_t sample_rate;					// sampling rate (blocks per second)
	uint32_t byterate;						// SampleRate * NumChannels * BitsPerSample/8
	uint16_t block_align;					// NumChannels * BitsPerSample/8
	uint16_t bits_per_sample;				// bits per sample, 8- 8bits, 16- 16 bits etc
	unsigned char data_chunk_header [4];		// DATA string or FLLR string
	uint32_t data_size;						// NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
} WAVHEADER;


FILE *file_Selected;
char *selected_FileID;
/* buffer used for audio play */
int16_t Audio_Buffer1[BUF_LEN];
int16_t Audio_Buffer2[BUF_LEN];


osSemaphoreDef(Sema_0);
osSemaphoreId(Sema_ID);


// Command queue from Rx_Command to Controller
osMessageQId mid_COMDQueue; 														// message queue for commands to Thread
osMessageQDef (COMDQueue, 1, uint32_t); 											// message queue object




enum commands{
  ListFiles,
  SendComplete,
  SendFiles,
  StoppedPlay,
  Play_File,
  PlayingFile
};

// State Machine definitions
enum state{
  NoState,
  Idle,
  List,
  Play_State
};


//////////////////////////////////////////////////////////
// thread function
void Control (void const *argument);
osThreadId tid_Control; 															// thread id
osThreadDef (Control, osPriorityNormal, 1, 0); 										// thread object

// Command queue from Rx_Command to Controller
osMessageQId mid_CMDQueue; 															// message queue for commands to Thread
osMessageQDef (CMDQueue, 1, uint32_t); 												// message queue object

// Command queue from FS
osMessageQId mid_Command_FSQueue;													// message queue for commands to Thread
osMessageQDef (Command_FSQueue, 1, uint32_t); 										// message queue object

// UART receive thread
void Rx_Command (void const *argument);  											// thread function
osThreadId tid_RX_Command;  														// thread id
osThreadDef (Rx_Command, osPriorityNormal, 1, 0); 									// thread object

// FS thread
void FS (void const *argument);                             						// thread function
osThreadId tid_FS;                                          						// thread id
osThreadDef (FS, osPriorityNormal, 1, 0);                   						// thread object


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// State Machine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Process_Event(uint16_t event){
  static uint16_t   Current_State = NoState;                                     	// Current state of the SM
  switch(Current_State)
  {
    case NoState:
      // Next State
      Current_State = Idle;                                                        	// Goes to next state

      LED_Off(LED_Green);
      LED_Off(LED_Orange);
      LED_Off(LED_Blue);
      LED_On(LED_Red);                                                          	// Entry Action
      break;

      // Idle State part of State Machine
    case Idle:

      if(event == ListFiles){
          Current_State = List;                                                    	// Goes to next state
          LED_Off(LED_Red);                                                        	// Exit Action
          LED_Off(LED_Green);
          LED_Off(LED_Orange);
          LED_On(LED_Blue);                                                        	// Entry Action
          osMessagePut(mid_Command_FSQueue, SendFiles, osWaitForever);             	// Send ListFiles message via Queue
      }
      if(event == Play_File)
      {
    	  Current_State = Play_State;
    	  LED_Off(LED_Blue);
    	  LED_Off(LED_Green);
    	  LED_Off(LED_Red);
    	  LED_On(LED_Orange);
    	  osMessagePut(mid_Command_FSQueue, PlayingFile, osWaitForever);
      }
      break;

      // List State part of State Machine
    case List:

      if(event == SendComplete)
      {
          Current_State = Idle;                                               // Goes to next state
          LED_Off(LED_Blue);                                                	// Exit action
          LED_Off(LED_Red);
          LED_Off(LED_Orange);
          LED_On(LED_Green);                                                    // Entry action
      }
      break;

    case Play_State:

    	if(event == StoppedPlay)
    	{
    		Current_State = Idle;                                               // Goes to next state
    		LED_Off(LED_Blue);                                                	// Exit action
    		LED_On(LED_Red);
    		LED_Off(LED_Orange);
    		LED_Off(LED_Green);
    	}
    	break;

  } // end case(Current_State)
} // Process_Event

// Initialize the threads
void Init_Thread (void)
{
   LED_Initialize(); 																// Initialize the LEDs
   UART_Init(); 																	// Initialize the UART

  // Create queues
   mid_CMDQueue = osMessageCreate (osMessageQ(CMDQueue), NULL); 					// create msg queue
  if (!mid_CMDQueue)return;															// Message Queue object not created, handle failure
  mid_Command_FSQueue = osMessageCreate (osMessageQ(Command_FSQueue), NULL); 		// create msg queue
  if (!mid_Command_FSQueue)return; 													// Message Queue object not created, handle failure

	Sema_ID = osSemaphoreCreate (osSemaphore(Sema_0), 0);					// Initialize the Semaphore
	mid_COMDQueue = osMessageCreate (osMessageQ(COMDQueue), NULL); 			// create msg queue
		if (!mid_COMDQueue)return;



  // Create threads
   tid_RX_Command = osThreadCreate (osThread(Rx_Command), NULL);
  if (!tid_RX_Command) return;
   tid_Control = osThreadCreate (osThread(Control), NULL);
  if (!tid_Control) return;
  tid_FS = osThreadCreate (osThread(FS), NULL);
  if (!tid_FS) return;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Control Function
void Control(void const *arg)
{
  osEvent evt; 																		// Receive message object
  Process_Event(0); 																// Initialize the State Machine
   while(1)
   {
    evt = osMessageGet (mid_CMDQueue, osWaitForever); 								// receive command
      if (evt.status == osEventMessage) 											// check for valid message
      {
    	  Process_Event(evt.value.v);												// Process the event
      }
   }
}



// RX_Command Function
	// Gets button pressed from GUI
	// Sends CMDQueue
void Rx_Command (void const *argument)
{
   char rx_char[2]={0,0};
   char name[80];
   while(1)
   {
      UART_receive(rx_char, 1); 													// Wait for command from PC GUI
      if(!strcmp(rx_char,Show_Files_char))											// List Files received
      {
    	  osMessagePut (mid_CMDQueue, ListFiles, osWaitForever);
      }
      if(!strcmp(rx_char,Play_File_char))										// Selected File received
      {
    	  UART_receivestring(name,80);
    	  selected_FileID = name;
    	  osMessagePut (mid_CMDQueue, Play_File, osWaitForever);
    	  rx_char[1] = 0;
      }
      if(!strcmp(rx_char,Stop_File_char))
      {
    	  osMessagePut (mid_CMDQueue, StoppedPlay, osWaitForever);
      }
   }
} // end Rx_Command



// FS Function
	// Gets Command_FSQueue
	// Sends CMDQueue
	// Interacts with USB
	// Interacts with DISCO-PC
void FS(void const *arg)
{
  osEvent evt; 																		// Receive message object

//Setting up and mounting the USB
	   usbStatus ustatus; 															// USB driver status variable
	   uint8_t drivenum = 0; 														// Using U0: drive number
	   char *drive_name = "U0:"; 													// USB drive name

	   fsStatus fstatus; 															// file system status variable
	   fsFileInfo info;
		WAVHEADER header;
		size_t rd;
		uint32_t i;
		static uint8_t rtrn = 0;
		uint8_t rdnum = 1; // read buffer number


	   LED_On(LED_Green);
	   ustatus = USBH_Initialize (drivenum); 										// initialize the USB Host
	   if (ustatus == usbOK)														// loop until the device is OK, may be delay from Initialize
	   {
		   ustatus = USBH_Device_GetStatus (drivenum); 								// get the status of the USB device
		   while(ustatus != usbOK)
		   {
			   ustatus = USBH_Device_GetStatus (drivenum); 							// get the status of the USB device
		   }
	   // initialize the drive
		   fstatus = finit (drive_name);
		   if (fstatus != fsOK)														// handle the error, finit didn't work
		   {
		   }

	   // Mount the drive
		   fstatus = fmount (drive_name);
		   if (fstatus != fsOK)														// handle the error, fmount didn't work
		   {
		   }
	   }

	while(1)
	{
    evt = osMessageGet (mid_Command_FSQueue, osWaitForever); 						// wait for message
		if (evt.status == osEventMessage) 											// check for valid message
		{
			if( evt.value.v == SendFiles)
			{
				// Sending start of the file
				char *StartFileList_msg = "2\n";
				char *EndFileList_msg = "3\n";
				UART_send(StartFileList_msg,2); 									// Send start string

				// Sending the file names on the USB
				info.fileID = 0;

				while (ffind ("U0:*.*", &info) == fsOK)
				{
					if (info.fileID != 4)
					{
						UART_send(info.name, strlen(info.name));
						UART_send("\n",1);
					}
				}

				//Sending the end of the file
				UART_send(EndFileList_msg,2); 										// Send start string

				osMessagePut (mid_CMDQueue, SendComplete, osWaitForever);
			}
		}



		while(selected_FileID != NULL)
		{
			file_Selected = fopen (selected_FileID,"r");// open a file on the USB device
			if (file_Selected != NULL)
			{
				fread((void *)&header, sizeof(header), 1, file_Selected);
			// initialize the audio output
				rtrn = BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, 0x49, 44100);
				if (rtrn != AUDIO_OK)return;

				fread((void *) Audio_Buffer1, sizeof(Audio_Buffer1), 1, file_Selected);
				BSP_AUDIO_OUT_Play((uint16_t *)Audio_Buffer1, BUF_LEN*2);

			//Make loop to do Part 3
				while(!feof(file_Selected))
				{

				//Writing Data to buffer 2
					fread((void *) Audio_Buffer2, sizeof(Audio_Buffer2), 1, file_Selected);; 					// Left channel

				//Send Buffer2 to DMA send through a queue
					osMessagePut(mid_COMDQueue, 2, osWaitForever);
					osSemaphoreWait(Sema_ID, osWaitForever);

				//Writing Data to buffer 1
					fread((void *) Audio_Buffer1, sizeof(Audio_Buffer1), 1, file_Selected);
					BSP_AUDIO_OUT_Play((uint16_t *)Audio_Buffer1, BUF_LEN*2);
				//Send Buffer1 to DMA send through a queue
					osMessagePut(mid_COMDQueue, 1, osWaitForever);
					osSemaphoreWait(Sema_ID, osWaitForever);
				}

				BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
				fclose (file_Selected); // close the file
				//selected_FileID = NULL;

			}// end if file opened
		}
		osMessagePut (mid_CMDQueue, StoppedPlay, osWaitForever);
	}
}



/* User Callbacks: user has to implement these functions if they are needed. */

/* This function is called when the requested data has been completely transferred. */

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	osEvent evt;

	evt = osMessageGet(mid_COMDQueue,0);
	if (evt.status == osEventMessage)
	{
		if (evt.value.v == 1)
		{
			BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)Audio_Buffer1, BUF_LEN);
		}
		if (evt.value.v == 2)
		{
			BSP_AUDIO_OUT_ChangeBuffer((uint16_t *)Audio_Buffer2, BUF_LEN);
		}
	}
	osSemaphoreRelease(Sema_ID);

}

/* This function is called when half of the requested buffer has been transferred. */

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{

}

/* This function is called when an Interrupt due to transfer error or peripheral error occurs. */

void BSP_AUDIO_OUT_Error_CallBack(void)
{
	while(1)
	{

	}
}
