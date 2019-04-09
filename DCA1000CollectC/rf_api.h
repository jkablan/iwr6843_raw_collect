//*************************************************************************************************
//  @file  rf_api.h
//  @brief 
//        This file contains API declarations for DCA1000EVM system. 
//
//*      (C) Copyright 2016 Texas Instruments, Inc.
//*
//*  Redistribution and use in source and binary forms, with or without
//*  modification, are permitted provided that the following conditions
//*  are met:
//*
//*    Redistributions of source code must retain the above copyright
//*    notice, this list of conditions and the following disclaimer.
//*
//*    Redistributions in binary form must reproduce the above copyright
//*    notice, this list of conditions and the following disclaimer in the
//*    documentation and/or other materials provided with the
//*    distribution.
//*
//*    Neither the name of Texas Instruments Incorporated nor the names of
//*    its contributors may be used to endorse or promote products derived
//*    from this software without specific prior written permission.
//*
//*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*
//*************************************************************************************************


/**************************************************************************************************
// Sample code: Here is the sample C-code which user can refer to implement an application 
// to control DCA1000 using 'RF_API.dll' or 'libRF_API.a' which is available at 
// C:\ti\mmwave_studio_<ver>\mmWaveStudio\Clients\AR1xController path.
// Post capture from DCA1000, user needs to call 'Packet_Reorder_Zerofill.exe' to reorder zero 
// filling in the captured file.
// 
// Packet_Reorder_Zerofill.exe adc_data_raw.bin adc_data.bin traceFile.txt
//*************************************************************************************************

// Status to check if DCA1000 is connected over Ethernet, will be set in the EventhandlerCallback
uint8 gCaptureCardConnect = 0;

//RF data card capture event handler
void EthernetEventHandlercallback(UINT16 u16CmdCode, UINT16 u16Status)
{
    if (u16CmdCode == 0x0A)
    {
        //Syste Error cmd code
        switch (u16Status)
        {
           // handle Case STS_NO_LVDS_DATA to STS_PLAYBACK_OUT_OF_SEQUENCE
        }
    }
    else
    {
        //Event register status
        switch (u16CmdCode)
        {
            // case RESET_FPGA_CMD_CODE to PLAYBACK_IS_IN_PROGRESS_CODE
            
            case set SYSTEM_CONNECT_CMD_CODE:
                gCaptureCardConnect = 1;
            break;
        }
    }
}

main()
{
    
    SINT32 retVal;
    UINT8  waitCount = 0;
    strEthConfigMode ethernetInitConfigurationParam;
    strConfigMode RFDCCardModeConfigurationParam;
    UINT16 PacketDelayData[3];
    
    printf("=========== DCA1000 Capture/Playback CLI Application =========== \n");

    // Register Ethernet Event Handler
    retVal = StatusDCCard_EventRegister(EthernetEventHandlercallback);
    // TODO: Handle return error value of above API call
    
    // Disconnet Ethernet connection if Any
    retVal = DisconnectRFDCCard();
    // TODO: Handle return error value of above API call
    
    // Set Ethernet configuration parameters
    ethernetInitConfigurationParam.au8MacId[0] = 12;
    ethernetInitConfigurationParam.au8MacId[1] = 34;
    ethernetInitConfigurationParam.au8MacId[2] = 56;
    ethernetInitConfigurationParam.au8MacId[3] = 78;
    ethernetInitConfigurationParam.au8MacId[4] = 90;
    ethernetInitConfigurationParam.au8MacId[5] = 12;
    // System IP address
    ethernetInitConfigurationParam.au8SourceIpAddr[0] = 192;
    ethernetInitConfigurationParam.au8SourceIpAddr[1] = 168;
    ethernetInitConfigurationParam.au8SourceIpAddr[2] = 33;
    ethernetInitConfigurationParam.au8SourceIpAddr[3] = 30;
    // FPGA IP Address
    ethernetInitConfigurationParam.au8DestiIpAddr[0]  = 192;
    ethernetInitConfigurationParam.au8DestiIpAddr[1]  = 168;
    ethernetInitConfigurationParam.au8DestiIpAddr[2]  = 33;
    ethernetInitConfigurationParam.au8DestiIpAddr[3]  = 180;
    
    ethernetInitConfigurationParam.u32RecordPortNo=4098;
    ethernetInitConfigurationParam.u32ConfigPortNo=4096;
    // Ethernet Init 
    retVal = ConfigureRFDCCard_RecordEthInit(ethernetInitConfigurationParam);
    // TODO: Handle return error value of above API call
    
    // Try to connect multiple times
    while (1)
    {
        // User needs to set gCaptureCardConnect to '1' in EthernetEventHandlercallback under SYSTEM_CONNECT_CMD_CODE case
        if (gCaptureCardConnect == 0)
        {
            // Connect DCA1000 card over Ethernet
            retVal = ConnectRFDCCard();
            Sleep(2000);// wait for 2 sec
            waitCount++;

            if (waitCount > 10)
            {
                printf("Ethernet Cable is disconnected Please check.....!!!");
                //break;
                return retVal;
            }
        }
        else
        {
            break;
        }
    }
    
    // Set the parameters, important param is eDataFormatMode and eLvdsMode
    RFDCCardModeConfigurationParam.eLogMode = RAW_MODE;
    // Change to AR1642 in case capturing data with AWR1642 device
    RFDCCardModeConfigurationParam.eLvdsMode = AR1243;
    RFDCCardModeConfigurationParam.eDataXferMode = CAPTURE;
    RFDCCardModeConfigurationParam.eDataCaptureMode = ETH_STREAM;
    // Data Format mode BIT12/BIT14/BIT16
    RFDCCardModeConfigurationParam.eDataFormatMode = BIT16;
    RFDCCardModeConfigurationParam.u8Timer = DEFAULT_CONFIG_TIMER;
    
    // Configure DCA1000
    retVal = ConfigureRFDCCard_Mode(RFDCCardModeConfigurationParam);
    // TODO: Handle return error value of above API call
    
    // invoke GetRFCaptureCardViaEthernetErrorType(retVal) API to get the error status/type.
    
    // Packet delay config
    PacketDelayData[0] = 1472; // User can use this as default value.
    PacketDelayData[1] = 25;  // User can use this as default value.
    PacketDelayData[2] = 0;
    
    retVal = ConfigureRFDCCard_DataPacketConfig(&PacketDelayData[0], 6);
    // TODO: Handle return error value of above API call
    
    // Start the Recording , Provide the full capture file path. It'll trigger DCA1000 to start 
    // capturing raw data and store at given location.
    retVal = StartRecordData("d:\\adc_data_DCA.bin", 1);
    // TODO: Handle return error value of above API call
        
    
    // To stop the capture
    retVal = StopRecordData();
    // TODO: Handle return error value of above API call
    
    
    // Note - EthernetEventHandlercallback function will get the error/status value, which user 
    // needs to take care of while init/configure/capture process.
}

**************************************************************************************************/

#ifndef RF_API_H
#define RF_API_H

//****************
// Includes
//****************

/* Socket includes for Windows / Linux                                      */
#if defined _WIN32          //(_MSC_VER)
    #include <stdint.h>
    #include <stdio.h>
    
    #define EXPORT __declspec(dllexport)
    #define IMPORT __declspec(dllimport)

#else                       // GNUC
    #define EXPORT __attribute__((visibility("default")))
    #define IMPORT

#endif

#define SINT32    int32_t
#define SINT8     int8_t
#define bool      uint8_t
#define UINT16	  uint16_t
#define UINT8	  uint8_t
#define UINT32	  uint32_t

/* To avoid structure padding                                               */
#pragma pack (1)

//****************
// Defines
//****************

/* Enable debug prints in a file (for debugging)                            */
//#define ENABLE_DEBUG

/* Macro definition for command return value                                */
#define STATUS                              SINT32

/* Success status for command                                               */
#define    RFDCCARD_SUCCESS                    (0)

/* Invalid input parameters status for command                              */
#define RFDCCARD_INVALID_INPUT_PARAMS       (-1)

/* OS error status for command                                              */
#define RFDCCARD_OS_ERR                     (-2)

/* UDP write failure status for command                                     */
#define RFDCCARD_UDP_WRITE_ERR              (-3)

/* Etherent cable connection error                                          */
#define RFDCCARD_ETH_CABLE_CON_ERROR        (-4)

/* Reset FPGA command code                                                  */
#define RESET_FPGA_CMD_CODE                 0x01

/* Reset AR device command code                                             */
#define RESET_AR_DEV_CMD_CODE               0x02

/* Configure FPGA data modes command code                                   */
#define CONFIG_FPGA_GEN_CMD_CODE            0x03

/* Configure EEPROM command code                                            */
#define CONFIG_EEPROM_CMD_CODE              0x04

/* Start record command code                                                */
#define RECORD_START_CMD_CODE               0x05

/* Stop record command code                                                 */
#define RECORD_STOP_CMD_CODE                0x06

/* Playback start command code                                              */
#define PLAYBACK_START_CMD_CODE             0x07

/* Playback stop command code                                               */
#define PLAYBACK_STOP_CMD_CODE              0x08

/* System aliveness check command code                                      */
#define SYSTEM_CONNECT_CMD_CODE             0x09

/* System status command code                                               */
#define SYSTEM_STATUS_CMD_CODE              0x0A

/* Configure packet data delay and size command code                        */
#define CONFIG_PACKET_DATA_CMD_CODE         0x0B

/* Configure AR device data mode command code                               */
#define CONFIG_DATA_MODE_AR_DEV_CMD_CODE    0x0C

/* Initiate FPGA playback command code                                      */
#define INIT_FPGA_PLAYBACK_CMD_CODE            0x0D

/* Read FPGA version command code                                           */
#define READ_FPGA_VERSION_CMD_CODE            0x0E

/* Invalid response packet error code                                       */
#define INVALID_RESP_PKT_ERROR_CODE         0xC1

/* Record file creation error code                                          */
#define RECORD_FILE_CREATION_ERROR_CODE     0xC2

/* Record packet out of sequence error code                                 */
#define RECORD_PKT_OUT_OF_SEQ_ERROR_CODE    0xC3

/* Record progress status code                                              */
#define RECORD_IS_IN_PROGRESS_CODE          0xC4

/* Playback from GUI to FPGA completed status code                          */
#define GUI_PLAYBACK_COMPLETED_CODE            0xC5

/* Playback file open error code                                            */
#define PLAYBACK_FILE_OPEN_ERROR_CODE        0xC6

/* Playback UDP write failure error code                                    */
#define PLAYBACK_UDP_WRITE_ERR              0xC7

/* Playback progress status code                                            */
#define PLAYBACK_IS_IN_PROGRESS_CODE        0xC8

/* Command header start bytes                                               */
#define HEADER_START_BYTES                  0xA55A

/* Command footer start bytes                                               */
#define FOOTER_STOP_BYTES                   0xEEAA

/* Command packet size                                                      */
#define FIXED_PACKET_SIZE                   8

/* Command packet size excluding footer                                     */
#define PACKET_FIXED_SIZE_EXC_FOOTER        6

/* Command packet footer size                                               */
#define PACKET_FOOTER_DATA_SIZE             2

/* Maximum data bytes in command request                                    */
#define MAX_DATA_BYTES                      504

/* Callback typedef for event handling                                      */
typedef void (*EVENT_HANDLER)
(
    UINT16 u16CmdCode,
    UINT16 u16Status
);


//****************
// Enumerations
//****************

/* System status status from FPGA                                           */
enum SYS_STATUS
{
    /* No lvds data  status         */
    STS_NO_LVDS_DATA = 0,

    /* No header status             */
    STS_NO_HEADER,

    /* EEPROM failure status        */
    STS_EEPROM_FAILURE,

    /* SD card detected status      */
    STS_SD_CARD_DETECTED,

    /* SD card removed status       */
    STS_SD_CARD_REMOVED,

    /* SD card full status          */
    STS_SD_CARD_FULL,

    /* Mode config failure status   */
    STS_MODE_CONFIG_FAILURE,

    /* DDR full status              */
    STS_DDR_FULL,

    /* Record completed status      */
    STS_REC_COMPLETED,

    /* LVDS buffer full status      */
    STS_LVDS_BUFFER_FULL,

    /* Playback completed status    */
    STS_PLAYBACK_COMPLETED,

    /* Playback out of sequence     */
    STS_PLAYBACK_OUT_OF_SEQ
};

/* Data log mode                                                            */
typedef enum CONFIG_LOG_MODE
{
    /* Raw mode                     */
    RAW_MODE = 1,

    /* Multi mode                   */
    MULTI_MODE
}ConfigLogMode;


/* Data LVDS mode                                                           */
typedef enum CONFIG_LVDS_MODE
{
    /* AR1243                       */
    AR1243 = 1,

    /* AR1642                       */
    AR1642
}ConfigLvdsMode;

/* Data transfer mode                                                       */
typedef enum CONFIG_TRANSFER_MODE
{
    /* Capture mode                 */
    CAPTURE = 1,

    /* Playback mode                */
    PLAYBACK
}ConfigTransferMode;


/* Data format mode                                                         */
typedef enum CONFIG_FORMAT_MODE
{
    /* 12 bit mode                  */
    BIT12 = 1,

    /* 14 bit mode                  */
    BIT14,

    /* 16 bit mode                  */
    BIT16
}ConfigFormatMode;


/* Data capture mode                                                        */
typedef enum CONFIG_CAPTURE_MODE
{
    /* SD card storage              */
    SD_STORAGE =1,

    /* Ethernet stream              */
    ETH_STREAM
}ConfigCaptureMode;


/* AR device data mode                                                      */
typedef enum CONFIG_DATA_MODE_AR_DEVICE
{
    /* Internal config              */
    INTERNAL = 1,

    /* Radar EVM GUI config         */
    GUI
}ConfigDataModeArDevice;


//****************
// Stucture Declarations
//****************

/* Data mode config for FPGA                                                */
typedef struct CONFIG_MODE
{
    /* Data log mode                */
    ConfigLogMode           eLogMode;

    /* Data LVDS mode               */
    ConfigLvdsMode          eLvdsMode;

    /* Data transfer mode           */
    ConfigTransferMode         eDataXferMode;

    /* Data capture mode            */
    ConfigCaptureMode          eDataCaptureMode;

    /* Data format mode             */
    ConfigFormatMode           eDataFormatMode;

    /* Timeout value for LVDS data  */
    UINT8                   u8Timer;
} strConfigMode;


/* System ethernet configuration                                            */
typedef struct ETH_CONFIG_MODE
{
    /* FPGA MAC address             */
    UINT8              au8MacId[6];

    /* PC IP address                */
    UINT8           au8SourceIpAddr[4];

    /* FPGA IP address              */
    UINT8            au8DestiIpAddr[4];

    /* Record port number           */
    UINT32            u32RecordPortNo;

    /* Config port number           */
    UINT32            u32ConfigPortNo;
} strEthConfigMode;

/* Socket IDs                                                               */
typedef struct RFDCCARD_SOCKInfo
{
    /* Config socket                */
    SINT32 s32EthConfSock;

    /* Raw data socket              */
    SINT32 s32EthRawSock;

    /* Data type 1 socket           */
    SINT32 s32EthDataType1;

    /* Data type 2 socket           */
    SINT32 s32EthDataType2;

    /* Data type 3 socket           */
    SINT32 s32EthDataType3;

    /* Data type 4 socket           */
    SINT32 s32EthDataType4;
}strRFDCCard_SockInfo;

/* Command request protocol                                                 */
typedef struct
{
    /* Header                       */
    UINT16  u16Header;

    /* Command code                 */
    UINT16  u16CmdCode;

    /* Data size                    */
    UINT16  u16DataSize;

    /* Data                         */
    UINT8   strData[MAX_DATA_BYTES];

    /* Footer                       */
    UINT16  u16Footer;
}DATA_CAPTURE_REQ;

/* Command response protocol                                                */
typedef struct
{
    /* Header   */
    UINT16  u16Header;

    /* Command code                 */
    UINT16  u16CmdCode;

    /* Command status               */
    UINT16  u16Status;

    /* Footer                       */
    UINT16  u16Footer;
}DATA_CAPTURE_RESP;


//*****************
// API Declarations
//*****************

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// Function Description: This function is to write the error message in a
// file
// Input Parameters:
// s8Msg [SINT8 *] - Error message to display
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
STATUS DEBUG_FILE_WRITE(SINT8 *s8Msg);

//*****************************************************************************
// Function Description: This function is to display the error message in
// file or console
// Input Parameters:
// s8Msg [SINT8 *] - Error message to display
// Output Parameters: NA
// Return Value: None
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
void THROW_ERROR_STATUS(SINT8 *s8Msg);

//*****************************************************************************
// Function Description: This function is to open socket connection to RF
// data capture card with the following configuration
//         1.    FPGA MAC ID
//         2.    PC and FPGA IP Address
//         3.    Record/playback port number
//        4.    Configuration port number
// Input Parameters:
// sEthConfigMode [strEthConfigMode] - Structure filled with config data
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConfigureRFDCCard_RecordEthInit
(
    strEthConfigMode    sEthConfigMode
);

//*****************************************************************************
// Function Description: This function is to configure the DCA1000EVM system
// with the following mode configuration
//         1.    Logging Mode          RAW/MULTI
//         2.    LVDS Mode            - AR1243/AR1642
//         3.    Data Transfer Mode      LVDS Capture/Playback
//         4.    Data Capture Mode      Ethernet Streaming/SD Card Storage
//         5.    Data Format Mode      - 12/14/16 bit
//      6.  Timer               - Timeout for LVDS data
// Input Parameters:
// sConfigMode [strConfigMode] - Structure filled with config data
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConfigureRFDCCard_Mode
(
  strConfigMode      sConfigMode
);

//*****************************************************************************
// Function Description: This function is to configure the EEPROM in RF
// data capture card with the following configuration
//         1.    FPGA MAC ID
//         2.    PC and FPGA IP Address
//         3.    Record/playback port number
//        4.    Configuration port number
// Input Parameters:
// sEthConfigMode [strEthConfigMode] - Structure filled with config data
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConfigureRFDCCard_EEPROM
(
  strEthConfigMode      sEthConfigMode
);


//*****************************************************************************
// Function Description: This function is to check the DCA1000EVM system
// aliveness
// Input Parameters:NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConnectRFDCCard();


//*****************************************************************************
// Function Description: This function is to close all socket connection
// and disconnect from the DCA1000EVM system
// Input Parameters:NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS DisconnectRFDCCard (void);

//*****************************************************************************
// Function Description: This function is to start recording the data
// streamed over ethernet from the DCA1000EVM system
// Input Parameters:
// s8RecordFileName     [SINT8 *] - File name along with the path to record
// bRecordSeqNum        [bool   ] - Record sequence number in files
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS StartRecordData
(
    SINT8 *s8RecordFileName,
    bool bRecordSeqNum
);

//*****************************************************************************
// Function Description: This function is to stop recording the data
// streamed over ethernet from the DCA1000EVM system
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS StopRecordData (void);

//*****************************************************************************
// Function Description: This function is to start playbacking the data
// over ethernet to the DCA1000EVM system
// Input Parameters:
// s8PlaybackFileName   [SINT8 *] - File with the playback data
// u16Delay             [UINT16 ] - Delay between packets in microsec
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS PlaybackStart_RadarEVMData
(
  SINT8*    s8PlaybackFileName,
  UINT16    u16Delay
);

//*****************************************************************************
// Function Description: This function is to stop playbacking the data
// streamed over ethernet to the DCA1000EVM system
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS StopPlaybackData  (void);

//*****************************************************************************
// Function Description: This function is to reset FPGA
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ResetDCCard_FPGA (void);

//*****************************************************************************
// Function Description: This function is to reset Radar EVM
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ResetRadarEVM (void);

//*****************************************************************************
// Function Description: This function is to register user event callback
// Input Parameters:
// RFDCCard_EventCallback  [EVENT_HANDLER] - Callback function
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS StatusDCCard_EventRegister
(
    EVENT_HANDLER   RFDCCard_EventCallback
);

//*****************************************************************************
// Function Description: This function is to read FPGA version
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ReadRFDCCard_FpgaVersion(void);

//*****************************************************************************
// Function Description: This function is to read API DLL version
// Input Parameters:
// s8DllVersion    [SINT8 *] - Pointer to a char array
// Output Parameters:
// s8DllVersion    [SINT8 *] - DLL version is copied to the char array
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ReadRFDCCard_DllVersion(SINT8 *s8DllVersion);

//*****************************************************************************
// Function Description: This function is to configure AR device data mode
// as internal or through Radar EVM GUI
// Input Parameters:
// u8DataPkt    [UINT8 *] - Command packet data filled in this array
// u16Len       [UINT16 ] - Length of the command packet data
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConfigureDataMode_RadarEVMData
(
    UINT8 *u8DataPkt,
    UINT16 u16Len
);

//*****************************************************************************
// Function Description: This function is to configure record data packet
// delay and number of bytes in a data packet sent from FPGA
// Input Parameters:
// u8DataPkt    [UINT8 *] - Command packet data filled in this array
// u16Len       [UINT16 ] - Length of the command packet data
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS ConfigureRFDCCard_DataPacketConfig(UINT8 *u8DataPkt,
                                                 UINT16 u16Len);

//*****************************************************************************
// Function Description: This function is to initiate streaming of
// the playback data through FPGA to the DCA1000EVM system
// Input Parameters: NA
// Output Parameters: NA
// Return Value: SINT32 value
// Modifies Global variables: NA
// Algorithm Description: NA
//*****************************************************************************
EXPORT STATUS InitFpgaPlayback(void);

#ifdef __cplusplus
}
#endif

#endif // RF_API_H
