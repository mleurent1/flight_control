//------ Include ------//

#include "windows.h"
#include "ftd2xx.h"
#include "mex.h"

//------ Macro and type defines ------//

#define ERROR_IF(condition, message, arg) \
	if (condition) \
		{ \
			mexPrintf("ERROR during ""%s"": ",CommandName); \
			mexPrintf(message, arg); \
			mexPrintf("\n");\
			plhs[0] = mxCreateDoubleScalar(-1); \
			return; \
		}
#define RETURN_ERROR   plhs[0] = mxCreateDoubleScalar(-1); return;
#define RETURN_SUCCESS plhs[0] = mxCreateDoubleScalar(0); return;

#define USB_TRANSFER_SIZE_MPSSE   65535 // in multiple of 64 bytes (default 4096)
#define USB_TRANSFER_SIZE_SERIAL  384 // in multiple of 64 bytes (default 4096)
#define DATA_TRANSFER_SIZE_MPSSE  (USB_TRANSFER_SIZE_MPSSE*62/64)
#define DATA_TRANSFER_SIZE_SERIAL (USB_TRANSFER_SIZE_SERIAL*62/64)
#define USB_LATENCY_TIMER_MPSSE   2 // in ms (default 16)
#define USB_LATENCY_TIMER_SERIAL  4 // in ms (default 16)

#define DEVICES_NB_MAX 8

#define PIN_TXD_MOSI 0x01
#define PIN_RXD_MISO 0x02
#define PIN_CTS_CSN  0x08
#define PIN_DTR_SCLK 0x10

#define PIN_CLK  0x01
#define PIN_MOSI 0x02
#define PIN_MISO 0x04
#define PIN_CSN  0x08

//------ Global variables ------//

static unsigned int DEVICE_INDEX = 0; // Index of current device
static FT_HANDLE ftHandle [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // Handle of FTDI devices
static bool DEVICE_OPEN [DEVICES_NB_MAX] = {false,false,false,false,false,false,false,false}; // Devices opened
static BYTE GPIOL_DIR [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // GPIOL pin direction
static BYTE GPIOL [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // GPIOL state
static BYTE GPIOH_DIR [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // GPIOH pin direction
static BYTE GPIOH [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // GPIOH state
static unsigned int CLOCK_DIVIDER [DEVICES_NB_MAX] = {0,0,0,0,0,0,0,0}; // Frequency = Master clock/(1+2*CLOCK_DIVIDER)
static bool CLOCK_POLARITY [DEVICES_NB_MAX] = {false,false,false,false,false,false,false,false}; // 1: rising edge, 0: falling edge
static bool CLOCK_PHASE [DEVICES_NB_MAX] = {false,false,false,false,false,false,false,false}; // 1: rising edge, 0: falling edge
static bool HIGH_SPEED_DEVICE [DEVICES_NB_MAX] = {false,false,false,false,false,false,false,false};

//------ USB transaction function ------//

int USB_transaction(FT_HANDLE ftHandle, BYTE * OutputBuffer, DWORD NumBytesToSend, BYTE * InputBuffer, DWORD NumBytesToRead, DWORD * NumBytesRead)
{
	FT_STATUS ftStatus;
	unsigned long int ReadTimeoutCounter;
	DWORD NumBytesSent;
	DWORD NumBytesToRead1;
	
	// Purge USB receive and transmit buffer
	ftStatus = FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX); 
	
	// USB Write
	ftStatus = FT_Write(ftHandle, OutputBuffer, NumBytesToSend, &NumBytesSent);
	if (NumBytesSent != NumBytesToSend)
	{
		mexPrintf("Write timeout\n");
		return -1;
	}
	
	// Check USB Rx buffer status
	ReadTimeoutCounter = 0;
	ftStatus = FT_GetQueueStatus(ftHandle, &NumBytesToRead1);
	while ((NumBytesToRead1 != NumBytesToRead) && (ftStatus == FT_OK) && (ReadTimeoutCounter < 500))
	{
		Sleep(1);
		ftStatus |= FT_GetQueueStatus(ftHandle, &NumBytesToRead1);
		ReadTimeoutCounter++;
	}
	if (NumBytesToRead1 != NumBytesToRead)
	{
		mexPrintf("Read timeout\n");
		return -1;
	}
	
	// USB Read
	ftStatus = FT_Read(ftHandle, InputBuffer, NumBytesToRead1, NumBytesRead);
	if (*NumBytesRead != NumBytesToRead1)
	{
		mexPrintf("Read timeout\n");
		return -1;
	}
	
	return 0;
}

//------ Main function ------//

void mexFunction (int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[])
{
	//------ Local variables ------//
   
    char * CommandName; // Matlab command
	FT_STATUS ftStatus; // Result of each D2XX call
	DWORD NumDevs; // The number of devices
	BYTE OutputBuffer[65536]; // Buffer to hold MPSSE commands and data to be sent to the FT2232H
	BYTE InputBuffer[65536]; // Buffer to hold data read from the FT2232H
	DWORD NumBytesToSend; // Index to the output buffer
	DWORD NumBytesSent; // Count of actual bytes sent - used with FT_Write
	DWORD NumBytesToRead; // Number of bytes available to read in the driver's input buffer
	DWORD NumBytesRead; // Count of actual bytes read - used with FT_Read
	// Duplicate 6 previous variables
	BYTE OutputBuffer1[65536];
	BYTE InputBuffer1[65536];
	DWORD NumBytesToSend1;
	DWORD NumBytesSent1;
	DWORD NumBytesToRead1;
	DWORD NumBytesRead1;
	FT_DEVICE_LIST_INFO_NODE * DevInfo;
	unsigned int BitPos;
	bool BitDir;
	bool BitVal;
	BYTE SpiAddress;
	BYTE SpiChannel;
	unsigned int SpiNumBytesToSend;
	unsigned int SpiNumBytesToRead;
	double * DataArray;
	double ClkFreq;
	double MClkFreq; // Master clock
	unsigned int DevIdx[DEVICES_NB_MAX];
	unsigned int DevIdxNb;
	int i,j,k;
	int Status;
	bool CommandEchod;
	BYTE DataBit;
	
	//------ Help ------//
	
	if (nrhs < 1)
	{
		mexPrintf("ftdi()\n");
		mexPrintf("	Print this readme.\n");
		mexPrintf("ftdi('enumerate')\n");
		mexPrintf("	Print the list and details of available FTDI devices.\n");
		mexPrintf("	Bit 0 of Flags indicates device is open, bit 1 indicates device is a high-speed one.\n");
		mexPrintf("	Return number of devices.\n");
		mexPrintf("ftdi('open', [index])\n");
		mexPrintf("	Open FTDI device identified by index.\n");
		mexPrintf("	Index can be a array of indexes. In this case, at the end, set the first device as the current device\n");
		mexPrintf("	If no index is specified, open device 0.\n");
		mexPrintf("ftdi('close', [index])\n");
		mexPrintf("	Close FTDI device identified by index.\n");
		mexPrintf("	Index can be a array of indexes.\n");
		mexPrintf("	If no index is specified, close all opened devices.\n");
		mexPrintf("ftdi('device', index)\n");
		mexPrintf("	Set device identified by index as the current device in use.\n");
		mexPrintf("	Return current device index.\n");
		mexPrintf("ftdi('set_SyncBB')\n");
		mexPrintf("	Set the device in Synchronous Bit Bang mode.\n");
		mexPrintf("ftdi('set_MPSSE')\n");
		mexPrintf("	Set the device in MPSSE mode.\n");
		mexPrintf("ftdi('clock', frequency)\n");
		mexPrintf("	Set the MPSSE clock frequency in Hz of the current device. \n");
		mexPrintf("	Return frequency of the current device.\n");
		mexPrintf("ftdi('clock_polarity', value)\n");
		mexPrintf("	Set the initial level of the MPSSE clock before a transaction.\n");
		mexPrintf("	Return polarity of the current device.\n");
		mexPrintf("ftdi('clock_phase', value)\n");
		mexPrintf("	Set the edge of the MPSSE clock on which the slave is latching data.\n");
		mexPrintf("	0: falling, 1:rising\n");
		mexPrintf("	Return phase of the current device.\n");
		mexPrintf("ftdi('GPIOL', bit, [direction, state])\n");
		mexPrintf("	Set one bit of low-byte GPIO.\n");
		mexPrintf("	Direction is 0 for IN and 1 for OUT.\n");
		mexPrintf("	Return current [direction, state].\n");
		mexPrintf("ftdi('GPIOH', bit, [direction, state])\n");
		mexPrintf("	Set one bit of high-byte GPIO.\n");
		mexPrintf("	Direction is 0 for IN and 1 for OUT.\n");
		mexPrintf("	Return current [direction, state].\n");
		mexPrintf("ftdi('print_GPIO')\n");
		mexPrintf("	Print the state of all GPIOs.\n");
		mexPrintf("	Returns 0 for success, -1 for fail.\n");
		mexPrintf("ftdi('SPI', [burst_data])\n");
		mexPrintf("	SPI transaction for MPSSE.\n");
		mexPrintf("	Return the read burst data.\n");
		mexPrintf("ftdi('SPI2', [burst_data])\n");
		mexPrintf("	SPI transaction for Synchronous Bit Bang.\n");
		mexPrintf("	Return the read burst data.\n");
		mexPrintf("ftdi('pin')\n");
		mexPrintf("	Print the pinout.\n");
		mexPrintf("\n");
		mexPrintf("Return 0 for success and -1 for failure\n");
		return;
	}

	CommandName = (char *) mxArrayToString(prhs[0]);
	
	//------ Enumerate ------//
	
	if (strcmp(CommandName, "enumerate") == 0)
	{
		// Get the number of FTDI devices
		ftStatus = FT_CreateDeviceInfoList(&NumDevs);
		ERROR_IF(ftStatus != FT_OK , "Failed to get number of devices", NULL)
		
		// Print information on FTDI devices
		if (NumDevs > 0)
		{
			DevInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*NumDevs);
			ftStatus = FT_GetDeviceInfoList(DevInfo, &NumDevs);
			ERROR_IF(ftStatus != FT_OK , "Failed to get list of devices", NULL)
			for (i = 0; i < (int)NumDevs; i++)
			{
				mexPrintf("Device %d:\n",i);
				mexPrintf("	Flags       = 0x%x\n",	DevInfo[i].Flags);
				mexPrintf("	Type        = 0x%x\n",	DevInfo[i].Type);
				mexPrintf("	ID          = 0x%x\n",	DevInfo[i].ID);
				mexPrintf("	LocId       = 0x%x\n", 	DevInfo[i].LocId);
				mexPrintf("	SerialNumber= %s\n",	DevInfo[i].SerialNumber);
				mexPrintf("	Description = %s\n",	DevInfo[i].Description);
				mexPrintf("	ftHandle    = 0x%x\n",	DevInfo[i].ftHandle);
			}
			
		}
		
		// Return number of devices
		plhs[0] = mxCreateDoubleScalar(NumDevs);
		return;
	}
	
	//------ Open ------//
	
	else if (strcmp(CommandName, "open") == 0)
	{
		// Get the number of FTDI devices
		ftStatus = FT_CreateDeviceInfoList(&NumDevs);
		ERROR_IF(ftStatus != FT_OK , "Failed to get number of devices", NULL)
		ERROR_IF(NumDevs == 0 , "No device to open", NULL)
		
		// Get information on FTDI devices
		DevInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*NumDevs);
		ftStatus = FT_GetDeviceInfoList(DevInfo, &NumDevs);
		ERROR_IF(ftStatus != FT_OK , "Failed to get list of devices", NULL)
		
		// Get list of devices to open
		if (nrhs < 2)
		{
			DevIdxNb = 1;
			DevIdx[0] = 0;
		}
		else
		{
			ERROR_IF(mxGetM(prhs[1]) > 1 , "Device indexes must be a line vector", NULL)
			DevIdxNb = (unsigned int)mxGetN(prhs[1]);
			ERROR_IF(DevIdxNb > DEVICES_NB_MAX, "Cannot support more than %d devices", DEVICES_NB_MAX)
			DataArray = mxGetPr(prhs[1]);
			for (i = 0; i < DevIdxNb; i++)
			{
				DevIdx[i] = (unsigned int)DataArray[i];
			}
		}
			
		// Open devices
		for (i = 0; i < DevIdxNb; i++)
		{
			ERROR_IF(DevIdx[i] > (unsigned int)NumDevs-1, "Cannot open device %d: Not available", DevIdx[i])
			DEVICE_INDEX = DevIdx[i];
		
			// Open
			ftStatus = FT_Open(DEVICE_INDEX, &ftHandle[DEVICE_INDEX]);
			ERROR_IF(ftStatus != FT_OK, "Failed to open device %d", DEVICE_INDEX)
			
			// Check if it is a high speed device
			HIGH_SPEED_DEVICE[DEVICE_INDEX] = ((DevInfo[DEVICE_INDEX].Flags & 0x02) > 0) ? true : false;
			
			// Reset to default settings
			GPIOL_DIR     [DEVICE_INDEX] = 0;
			GPIOL         [DEVICE_INDEX] = 0;
			GPIOH_DIR     [DEVICE_INDEX] = 0;
			GPIOH         [DEVICE_INDEX] = 0;
			CLOCK_DIVIDER [DEVICE_INDEX] = 0;
			CLOCK_POLARITY[DEVICE_INDEX] = false;
			CLOCK_PHASE   [DEVICE_INDEX] = false;
			
			// Device init
			ftStatus  = FT_ResetDevice(ftHandle[DEVICE_INDEX]); // Reset device
			//ftStatus |= FT_Purge(ftHandle[DEVICE_INDEX], FT_PURGE_RX | FT_PURGE_TX); // Purge USB receive and transmit buffer
			ftStatus |= FT_SetChars(ftHandle[DEVICE_INDEX], false, 0, false, 0); // Disable event/error characters
			ftStatus |= FT_SetTimeouts(ftHandle[DEVICE_INDEX], 500, 500); // R/W timeouts (ms)
			ERROR_IF(ftStatus < 0, "Failed to initialise device %d", DEVICE_INDEX)
			
			DEVICE_OPEN[DEVICE_INDEX] = true;
			mexPrintf("Opened device %d: %s\n", DEVICE_INDEX, DevInfo[DEVICE_INDEX].Description);
		}
		
		DEVICE_INDEX = DevIdx[0]; // Use first opened device
		RETURN_SUCCESS
	}
	
	//------ Close ------//
	
	else if (strcmp(CommandName, "close") == 0)
	{
		// Get list of devices to close
		if (nrhs < 2) 
		{
			// Close all opened devices if no specified devices
			DevIdxNb = 0;
			for (i = 0; i < DEVICES_NB_MAX; i++)
			{
				if (DEVICE_OPEN[i] == true)
				{
					DevIdxNb++;
					DevIdx[DevIdxNb-1] = i;
				}
			}
		}
		else
		{
			ERROR_IF(mxGetM(prhs[1]) > 1 , "Device indexes must be a line vector", NULL)
			DevIdxNb = (unsigned int)mxGetN(prhs[1]);
			ERROR_IF(DevIdxNb > DEVICES_NB_MAX, "Cannot support more than %d devices", DEVICES_NB_MAX)
			DataArray = mxGetPr(prhs[1]);
			for (i = 0; i < DevIdxNb; i++)
			{
				DevIdx[i] = (unsigned int)DataArray[i];
			}
		}
		
		// Reset and close devices
		for (i = 0; i < DevIdxNb; i++)
		{
			if (DEVICE_OPEN[DevIdx[i]] == true)
			{
				DEVICE_INDEX = DevIdx[i];
				ftStatus  = FT_ResetDevice(ftHandle[DEVICE_INDEX]);
				ftStatus |= FT_Close(ftHandle[DEVICE_INDEX]);
				ERROR_IF(ftStatus != FT_OK , "Failed to close device %d", i);
				mexPrintf("Closed device %d\n", DEVICE_INDEX);
				DEVICE_OPEN[DEVICE_INDEX] = false;
			}
		}
		
		RETURN_SUCCESS
	}
	
	//------ Device in use ------//
	
	else if (strcmp(CommandName, "device") == 0)
	{
		if (nrhs > 1)
		{
			DevIdx[0] = (unsigned int)mxGetScalar(prhs[1]);
			ERROR_IF(DevIdx[0] > DEVICES_NB_MAX, "Do not support device index greater than %d", DEVICES_NB_MAX)
			ERROR_IF(DEVICE_OPEN[DevIdx[0]] == false, "Device %d not opened or not existing", DevIdx)
			DEVICE_INDEX = DevIdx[0];
		}
		plhs[0] = mxCreateDoubleScalar((double)DEVICE_INDEX);
		return;
	}
	
	//------ Send bytes ------//
	
	else if (strcmp(CommandName, "write") == 0)
	{
		// Argument parsing: command, [bytes to write]
		ERROR_IF(nrhs < 2, "Missing data to write", NULL)
		ERROR_IF(mxGetM(prhs[1]) > 1 , "Data to write must be a line vector", NULL)
		NumBytesToSend1 = (unsigned int) mxGetN(prhs[1]);
		ERROR_IF(NumBytesToSend1 > DATA_TRANSFER_SIZE_SERIAL, "Write size must not exceed %d bytes", DATA_TRANSFER_SIZE_MPSSE)
		DataArray = mxGetPr(prhs[1]);
		
		NumBytesToSend = 0;
		for (i = 0; i < NumBytesToSend1; i++)
			OutputBuffer[NumBytesToSend++] = (BYTE)(DataArray[i]);
		
		ftStatus = FT_Write(ftHandle[DEVICE_INDEX], OutputBuffer, NumBytesToSend, &NumBytesSent);
		ERROR_IF(NumBytesSent != NumBytesToSend, "Write timeout", NULL)
	}
	
	//------ Read bytes ------//
	
	else if (strcmp(CommandName, "read") == 0)
	{
		ftStatus  = FT_GetQueueStatus(ftHandle[DEVICE_INDEX], &NumBytesToRead);
		ftStatus |= FT_Read(ftHandle[DEVICE_INDEX], InputBuffer, NumBytesToRead, &NumBytesRead);
		ERROR_IF(NumBytesRead != NumBytesToRead, "Read timeout", NULL)
		
		// Mex output
		plhs[0] = mxCreateDoubleMatrix(1, NumBytesRead, mxREAL);
		DataArray = mxGetPr(plhs[0]);
		for (i = 0; i < NumBytesRead; i++)
			DataArray[i] = (double)InputBuffer[i];
		return;
	}
	
	//------ Serial initialisation ------//
	
	else if (strcmp(CommandName, "set_serial") == 0)
	{
		ftStatus  = FT_SetBitMode(ftHandle[DEVICE_INDEX], 0, 0); // Reset mode to setting in EEPROM
		ftStatus |= FT_SetUSBParameters(ftHandle[DEVICE_INDEX], USB_TRANSFER_SIZE_SERIAL, USB_TRANSFER_SIZE_SERIAL); // USB request transfer sizes
		ftStatus |= FT_SetLatencyTimer(ftHandle[DEVICE_INDEX], USB_LATENCY_TIMER_SERIAL); // Latency timer
		ftStatus |= FT_SetBaudRate(ftHandle[DEVICE_INDEX], 115200);
		ftStatus |= FT_SetDataCharacteristics(ftHandle[DEVICE_INDEX], FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
		ftStatus |= FT_SetFlowControl(ftHandle[DEVICE_INDEX], FT_FLOW_NONE, NULL, NULL);
		ERROR_IF(ftStatus != FT_OK, "Failed to initialize Serial mode", NULL)
	}
	
	//------ Synchronous Bit Bang initialisation ------//

	else if (strcmp(CommandName, "set_SyncBB") == 0)
	{
		ftStatus  = FT_SetBitMode(ftHandle[DEVICE_INDEX], 0, 0); // Reset mode to setting in EEPROM
		ftStatus |= FT_SetBitMode(ftHandle[DEVICE_INDEX], PIN_TXD_MOSI | PIN_CTS_CSN | PIN_DTR_SCLK, 4); // Enable synchronous bit bang mode
		ftStatus |= FT_SetUSBParameters(ftHandle[DEVICE_INDEX], USB_TRANSFER_SIZE_SERIAL, USB_TRANSFER_SIZE_SERIAL); // USB request transfer sizes
		ftStatus |= FT_SetLatencyTimer(ftHandle[DEVICE_INDEX], USB_LATENCY_TIMER_SERIAL); // Latency timer
		ftStatus |= FT_SetDivisor(ftHandle[DEVICE_INDEX], 0); // Maximum baudrate
		ERROR_IF(ftStatus != FT_OK, "Failed to initialize Synchronous Bit Bang mode", NULL)
			
		// Set CTS/CSN to 1, DTR/CLK and TXD/MOSI to 0
		OutputBuffer[0] = PIN_CTS_CSN;
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 1, InputBuffer, 1, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
	
		mexPrintf("Device set in Synchronous Bit Bang mode\n");
		RETURN_SUCCESS
	}
	
	//------ MPSSE initialisation ------//
	
	else if (strcmp(CommandName, "set_MPSSE") == 0)
	{
		ftStatus  = FT_SetBitMode(ftHandle[DEVICE_INDEX], 0, 0); // Reset mode to setting in EEPROM
		ftStatus |= FT_SetBitMode(ftHandle[DEVICE_INDEX], 0, 2); // Enable MPSSE mode
		ftStatus |= FT_SetUSBParameters(ftHandle[DEVICE_INDEX], USB_TRANSFER_SIZE_MPSSE, USB_TRANSFER_SIZE_MPSSE); // USB request transfer sizes
		ftStatus |= FT_SetLatencyTimer(ftHandle[DEVICE_INDEX], USB_LATENCY_TIMER_MPSSE); // Latency timer
		ERROR_IF(ftStatus != FT_OK, "Failed to initialize MPSSE mode", NULL)
		
		//------ Synchronize the MPSSE by sending bad commands AA ------//
		
		// Send command
		OutputBuffer[0] = 0xAA;
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 1, InputBuffer, 2, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		// Check if we received echo flag followed by the bad command
		CommandEchod = false;
		for (i = 0; i < NumBytesRead - 1; i++) 
		{
			if ((InputBuffer[i] == 0xFA) && (InputBuffer[i+1] == 0xAA))
			{
				CommandEchod = true;
				break;
			}
		}
		ERROR_IF(CommandEchod == false, "Failed to synchronise MPSSE with bad command AA", NULL)
		
		//------ Synchronize the MPSSE by sending bad commands AB ------//
		
		// Send command
		OutputBuffer[0] = 0xAB;
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 1, InputBuffer, 2, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		// Check if we received echo flag followed by the bad command
		CommandEchod = false;
		for (i = 0; i < NumBytesRead - 1; i++) 
		{
			if ((InputBuffer[i] == 0xFA) && (InputBuffer[i+1] == 0xAB))
			{
				CommandEchod = true;
				break;
			}
		}
		ERROR_IF(CommandEchod == false, "Failed to synchronise MPSSE with bad command AB", NULL)
	
		//------ MPSSE settings for SPI ------//
		
		// IO setup
		GPIOL_DIR[DEVICE_INDEX] = PIN_CLK | PIN_MOSI | PIN_CSN;
		GPIOL[DEVICE_INDEX] = (CLOCK_POLARITY[DEVICE_INDEX] ? PIN_CLK : 0) | PIN_MOSI | PIN_CSN;
		
		// Prepare all commands for 1 transaction
		OutputBuffer[ 0] = 0x80; // Configure low-byte of port
		OutputBuffer[ 1] = GPIOL[DEVICE_INDEX]; // Initial state
		OutputBuffer[ 2] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		OutputBuffer[ 3] = 0x82; // Configure high-byte of port
		OutputBuffer[ 4] = GPIOH[DEVICE_INDEX]; // Initial state
		OutputBuffer[ 5] = GPIOH_DIR[DEVICE_INDEX]; // Direction
		OutputBuffer[ 6] = 0x8A; // Disable divide by 5 of master clock
		OutputBuffer[ 7] = 0x97; // Turn off adaptive clocking
		OutputBuffer[ 8] = 0x8D; // Disable three-phase clocking
		OutputBuffer[ 9] = 0x86; // Set clock divisor
		OutputBuffer[10] = CLOCK_DIVIDER[DEVICE_INDEX] & 0xFF; // Set 0xValueL of clock divisor
		OutputBuffer[11] = (CLOCK_DIVIDER[DEVICE_INDEX] >> 8) & 0xFF; // Set 0xValueH of clock divisor
		OutputBuffer[12] = 0x85; // Ensure internal loopback is off
		
		// Send commands
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 13, InputBuffer, 0, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		mexPrintf("Device set in MPSSE mode\n");
		RETURN_SUCCESS
	}
	
	//------ Clock frequency ------//
	
	else if (strcmp(CommandName, "clock") == 0)
	{
		MClkFreq = HIGH_SPEED_DEVICE[DEVICE_INDEX] ? 60e6 : 12e6;
		
		if (nrhs > 1)
		{
			ClkFreq = mxGetScalar(prhs[1]);
			
			double ClkFreqTmp;
			for (i = 0; i < 65536; i++)
			{
				ClkFreqTmp = MClkFreq / (2.0 * (1.0 + (double)i));
				if (ClkFreqTmp <= ClkFreq)
				{
					CLOCK_DIVIDER[DEVICE_INDEX] = i;
					break;
				}
			}
			
			// Send command
			OutputBuffer[0] = 0x86; // Set clock divisor
			OutputBuffer[1] = CLOCK_DIVIDER[DEVICE_INDEX] & 0xFF; // Set 0xValueL of clock divisor
			OutputBuffer[2] = (CLOCK_DIVIDER[DEVICE_INDEX] >> 8) & 0xFF; // Set 0xValueH of clock divisor
			Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 3, InputBuffer, 0, &NumBytesRead);
			ERROR_IF(Status < 0, "USB transaction failed", NULL)
		}
		
		ClkFreq = MClkFreq / (2.0 * (1.0 + (double)CLOCK_DIVIDER[DEVICE_INDEX]));
		
		// Mex output
		plhs[0] = mxCreateDoubleScalar(ClkFreq);
		return;
	}
		
	//------ Clock polarity ------//
	
	else if (strcmp(CommandName, "clock_polarity") == 0)
	{
		if (nrhs > 1)
			CLOCK_POLARITY[DEVICE_INDEX] = (bool)mxGetScalar(prhs[1]);
		
		// Mex output
		plhs[0] = mxCreateDoubleScalar((double)CLOCK_POLARITY[DEVICE_INDEX]);
		
		return;
	}
	
	//------ Clock phase ------//
	
	else if (strcmp(CommandName, "clock_phase") == 0)
	{
		if (nrhs > 1)
			CLOCK_PHASE[DEVICE_INDEX] = (bool)mxGetScalar(prhs[1]);
		plhs[0] = mxCreateDoubleScalar((double)CLOCK_PHASE[DEVICE_INDEX]);
		return;
	}
	
	//------ GPIO low byte ------//
	
	else if (strcmp(CommandName, "GPIOL") == 0)
	{
		ERROR_IF(nrhs < 2, "Missing bit position and/or [direction, state]", NULL)
		BitPos = (unsigned int)mxGetScalar(prhs[1]);
		
		NumBytesToSend = 0;
		
		if (nrhs > 2)
		{
			ERROR_IF((mxGetM(prhs[2]) > 1) || (mxGetN(prhs[2]) < 2) , "Third argument must be a line vector of 2 elements [direction, state]", NULL)
			DataArray = mxGetPr(prhs[2]);
			BitDir = (bool)DataArray[0];
			BitVal = (bool)DataArray[1];
			
			if (BitDir)
				GPIOL_DIR[DEVICE_INDEX] |= 1 << BitPos;
			else
				GPIOL_DIR[DEVICE_INDEX] &= ~(1 << BitPos);
			if (BitVal)
				GPIOL[DEVICE_INDEX] |= 1 << BitPos;
			else
				GPIOL[DEVICE_INDEX] &= ~(1 << BitPos);
				
			OutputBuffer[NumBytesToSend++] = 0x80; // Configure low byte of port
			OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX] ; // Initial state
			OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		}
		OutputBuffer[NumBytesToSend++] = 0x81; // Read low byte of port
		
		// Send commands
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, NumBytesToSend, InputBuffer, 1, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		// Mex output
		plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
		DataArray = mxGetPr(plhs[0]);
		DataArray[0] = (GPIOL_DIR[DEVICE_INDEX] & (1<<BitPos)) ? 1 : 0;
		DataArray[1] = (InputBuffer[0] & (1<<BitPos)) ? 1 : 0;
		return;
	}
	
	//------ set GPIO high byte ------//
	
	else if (strcmp(CommandName, "GPIOH") == 0)
	{
		ERROR_IF(nrhs < 2, "Missing bit position and/or [direction, state]", NULL)
		BitPos = (unsigned int)mxGetScalar(prhs[1]);
		
		NumBytesToSend = 0;
		
		if (nrhs > 2)
		{
			ERROR_IF((mxGetM(prhs[2]) > 1) || (mxGetN(prhs[2]) < 2) , "Third argument must be a line vector of 2 elements [direction, state]", NULL)
			DataArray = mxGetPr(prhs[2]);
			BitDir = (bool)DataArray[0];
			BitVal = (bool)DataArray[1];
			
			if (BitDir)
				GPIOH_DIR[DEVICE_INDEX] |= 1 << BitPos;
			else
				GPIOH_DIR[DEVICE_INDEX] &= ~(1 << BitPos);
			if (BitVal)
				GPIOH[DEVICE_INDEX] |= 1 << BitPos;
			else
				GPIOH[DEVICE_INDEX] &= ~(1 << BitPos);
				
			OutputBuffer[NumBytesToSend++] = 0x82; // Configure high byte of port
			OutputBuffer[NumBytesToSend++] = GPIOH[DEVICE_INDEX] ; // Initial state
			OutputBuffer[NumBytesToSend++] = GPIOH_DIR[DEVICE_INDEX]; // Direction
		}
		OutputBuffer[NumBytesToSend++] = 0x83; // Read high byte of port
		
		// Send commands
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, NumBytesToSend, InputBuffer, 1, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		// Mex output
		plhs[0] = mxCreateDoubleMatrix(1, 2, mxREAL);
		DataArray = mxGetPr(plhs[0]);
		DataArray[0] = (GPIOH_DIR[DEVICE_INDEX] & (1<<BitPos)) ? 1 : 0;
		DataArray[1] = (InputBuffer[0] & (1<<BitPos)) ? 1 : 0;
		return;
	}
	
	//------ Print GPIO ------//
	
	else if (strcmp(CommandName, "print_GPIO") == 0)
	{
		// Send commands
		OutputBuffer[0] = 0x81; // Read low byte of port
		OutputBuffer[1] = 0x83; // Read high byte of port
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, 2, InputBuffer, 2, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		mexPrintf("\n");
		for (i = 0; i < 8; i++)
			mexPrintf("GPIOL%d (ADBUS%d) %s %d\n", i, i, (GPIOL_DIR[DEVICE_INDEX] & (1<<i)) ? "OUT" : "IN ", (InputBuffer[0] & (1<<i)) ? 1 : 0);
		mexPrintf("\n");
		for (i = 0; i < 8; i++)
			mexPrintf("GPIOH%d (ACBUS%d) %s %d\n", i, i, (GPIOH_DIR[DEVICE_INDEX] & (1<<i)) ? "OUT" : "IN ", (InputBuffer[1] & (1<<i)) ? 1 : 0);
		RETURN_SUCCESS
	}
	
	//------ SPI transaction ------//
	
	else if (strcmp(CommandName, "SPI") == 0)
	{
		// Argument parsing: command, [bytes to write]
		ERROR_IF(nrhs < 2, "Missing data to write", NULL)
		ERROR_IF(mxGetM(prhs[1]) > 1 , "Data to write must be a line vector", NULL)
		SpiNumBytesToSend = (unsigned int) mxGetN(prhs[1]);
		ERROR_IF(SpiNumBytesToSend > DATA_TRANSFER_SIZE_MPSSE, "Write size must not exceed %d bytes", DATA_TRANSFER_SIZE_MPSSE)
		DataArray = mxGetPr(prhs[1]);
		
		NumBytesToSend = 0;
		
		// Program clock polarity
		if (CLOCK_POLARITY[DEVICE_INDEX])
			GPIOL[DEVICE_INDEX] |= PIN_CLK;
		else
			GPIOL[DEVICE_INDEX] &= ~PIN_CLK;
		OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
		OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
		OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		
		// Program CSN = 0
		GPIOL[DEVICE_INDEX] &= ~PIN_CSN;
		OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
		OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
		OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		
		// Handle SPI modes
		if ((CLOCK_POLARITY[DEVICE_INDEX] == false) && (CLOCK_PHASE[DEVICE_INDEX] == true))
		{
			GPIOL[DEVICE_INDEX] |= PIN_CLK;
			OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
			OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
			OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		}
		else if ((CLOCK_POLARITY[DEVICE_INDEX] == true) && (CLOCK_PHASE[DEVICE_INDEX] == false))
		{
			GPIOL[DEVICE_INDEX] &= ~PIN_CLK;
			OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
			OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
			OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		}
		
		// Program data to send
		OutputBuffer[NumBytesToSend++] = (CLOCK_PHASE[DEVICE_INDEX]) ? 0x34 : 0x31; // 0x31: out fall, in rise, 0x34: out rise, in fall (MSB first)
		OutputBuffer[NumBytesToSend++] = (SpiNumBytesToSend-1) & 0x00FF; // Length L
		OutputBuffer[NumBytesToSend++] = ((SpiNumBytesToSend-1) & 0xFF00) >> 8; // Length H, Length = Length L + 2^8*Length H + 1
		for (i = 0; i < SpiNumBytesToSend; i++)
			OutputBuffer[NumBytesToSend++] = (BYTE)(DataArray[i]);
		
		// Program CSN = 1
		GPIOL[DEVICE_INDEX] |= PIN_CSN;
		OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
		OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
		OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		
		// Program clock polarity
		if (CLOCK_POLARITY[DEVICE_INDEX])
			GPIOL[DEVICE_INDEX] |= PIN_CLK;
		else
			GPIOL[DEVICE_INDEX] &= ~PIN_CLK;
		OutputBuffer[NumBytesToSend++] = 0x80; // Configure data bits low-byte of MPSSE port
		OutputBuffer[NumBytesToSend++] = GPIOL[DEVICE_INDEX]; // Initial state
		OutputBuffer[NumBytesToSend++] = GPIOL_DIR[DEVICE_INDEX]; // Direction
		
		// Send commnands
		Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, NumBytesToSend, InputBuffer, SpiNumBytesToSend, &NumBytesRead);
		ERROR_IF(Status < 0, "USB transaction failed", NULL)
		
		// Mex output
		plhs[0] = mxCreateDoubleMatrix(1, SpiNumBytesToSend, mxREAL);
		DataArray = mxGetPr(plhs[0]);
		for (i = 0; i < SpiNumBytesToSend; i++)
			DataArray[i] = (double)InputBuffer[i];
		return;
	}
	
	//------ SPI transaction for Synchronous Bit Bang ------//
	
	else if (strcmp(CommandName, "SPI2") == 0)
	{
		// Argument parsing: command, [bytes to write]
		ERROR_IF(nrhs < 2, "Missing data to write", NULL)
		ERROR_IF(mxGetM(prhs[1]) > 1 , "Data to write must be a line vector", NULL)
		SpiNumBytesToSend = (unsigned int) mxGetN(prhs[1]);
		ERROR_IF(SpiNumBytesToSend > 1025, "Write size must not exceed 1025 bytes", NULL)
		DataArray = mxGetPr(prhs[1]);
		
		// Parallel to serial
		NumBytesToSend1 = 0;
		OutputBuffer1[NumBytesToSend1++] = 0;
		for (i = 0; i < SpiNumBytesToSend; i++)
		{
			for (j = 7; j >= 0; j--)
			{
				DataBit = (((BYTE)(DataArray[i]) & (1<<j)) > 0) ? PIN_TXD_MOSI : 0;
				OutputBuffer1[NumBytesToSend1++] = DataBit;
				OutputBuffer1[NumBytesToSend1++] = DataBit | PIN_DTR_SCLK;
			}
		}
		OutputBuffer1[NumBytesToSend1++] = PIN_CTS_CSN;
		
		// Divide into burst of DATA_TRANSFER_SIZE_SERIAL bytes
		NumBytesToSend = 0;
		NumBytesRead1 = 0;
		for (i = 0; i < NumBytesToSend1; i++)
		{
			OutputBuffer[NumBytesToSend++] = OutputBuffer1[i];
			if ((i == (NumBytesToSend1-1)) || ((NumBytesToSend % DATA_TRANSFER_SIZE_SERIAL) == 0))
			{
				// USB transaction
				Status = USB_transaction(ftHandle[DEVICE_INDEX], OutputBuffer, NumBytesToSend, InputBuffer, NumBytesToSend, &NumBytesRead);
				ERROR_IF(Status < 0, "USB transaction failed", NULL)
				
				memcpy(InputBuffer1+NumBytesRead1, InputBuffer, NumBytesRead);
				
				NumBytesRead1 += NumBytesRead;
				NumBytesToSend = 0;
			}
		}
		
		// Mex output
		plhs[0] = mxCreateDoubleMatrix(1, SpiNumBytesToSend, mxREAL);
		DataArray = mxGetPr(plhs[0]);
		
		// Serial to parallel
		NumBytesRead = 0;
		NumBytesRead++; // 0
		for (i = 0; i < SpiNumBytesToSend; i++)
		{
			for (j = 7; j >= 0; j--)
			{
				NumBytesRead++; // DataBit
				NumBytesRead++; // DataBit | PIN_DTR_SCLK
				DataArray[i] = DataArray[i] + (((InputBuffer1[NumBytesRead] & PIN_RXD_MISO) > 0) ? (double)(1<<j) : 0);
			}
		}
		return;
	}
	
	//------ Pinout ------//
	
	else if (strcmp(CommandName, "pin") == 0)
	{
		mexPrintf("MPSSE:         | SK   | DO   | DI   | CS   | GPIOL4  | GPIOL5 | GPIOL6 | GPIOL7 | GPIOH0  | GPIOH1  | GPIOH2      | GPIOH3    | GPIOH4 | GPIOH5 | GPIOH6 | GPIOH7 |\n");
		mexPrintf("FT2232H A bus: | AD0  | AD1  | AD2  | AD3  | AD4     | AD5    | AD6    | AD7    | AC0     | AC1     | AC2         | AC3       | AC4    | AC5    | AC6    | AC7    |\n");
		mexPrintf("FT2232D A bus: | AD0  | AD1  | AD2  | AD3  | AD4     | AD5    | AD6    | AD7    | AC0     | AC1     | AC2         | AC3       |\n");
		mexPrintf("Semtech bridge:| PIN1 | PIN3 | PIN8 | PIN7 | PIN19   | PIN10  | PIN13  | PIN11  | PIN20   | PIN18   | PIN16       | PIN14     |\n");
		mexPrintf("EVK 1272:      | SCK  | MOSI | MISO | NSS  | FEM_CSD | RESET  | RXTX   | DIO4   | FEM_CTX | FEM_CPS | FEM_ANT_SEL | FEM_SPARE |\n");
	}
	
	//------ Command error ------//
	
	else
	{
		mexPrintf("Command not recognized\n");
		RETURN_ERROR
	}
}
