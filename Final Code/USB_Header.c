/***********************************************************************************************
 * Name:         USB_Header.c                                                                  *
 * Author:       Neel Desai, University of Colorado - Boulder                                  *
 * Date:         04/19/2014                                                                    *
 * Description:  -> This code initializes the driver and registers a UVC class driver          *
 *                  with the OS. The camInit() function is called to perform these             *
 *                  tasks. The UVC class is defined here as USB_CLASS_MISC. The                *
 *                  system can recognize dynamic attachment and removal of the device.         *
 *               -> Once the device is attached (or removed), the corresponding callback       *
 *                  function is called. On attachment, the device is in default state.         *
 *               -> We can read the device and configuration descriptors from the device to    *
 *                  learn more about the capabilities of the device by calling the             *
 *                  usbHstGetDescriptor() function calls. If the device supports multiple      *
 *                  configurations, we can select the one which we want. This information was  *
 *                  collected by probing the camera using an application called UVC Viewer.    *
 *                  Since this device (Logitech C200 camera) supports only 1 configuration,    *
 *                  I have used usbHstGetConfiguration() and usbHstSetConfiguration()          *
 *                  function calls to set the supported configuration. Now, the device is      *
 *                  in the configured state and all it's interfaces are activated.             *
 *               -> Then, the system starts negotiating parameters like the maxPayloadSize     *
 *                  with the camera using the VS_PROBE_CONTROL ans VS_COMMIT_CONTOL control    *
 *                  transfers. The maxPayloadSize was found to be 944 bytes(0x3B0). The data[] *
 *                  array is used to exchange data between the system and the camera.          *
 *               -> After the negotiation is complete, we can select the alternate setting     *
 *                  for interface 1(video streaming is controlled by interface 1). For the     *
 *                  maxPayloadSize of 944 bytes, the alternate setting value is 6. The         *
 *                  details regarding the alternate setting for maxPayloadSize and all the     *
 *                  configuration and device descriptors were obtained by probing the camera   *
 *                  with a Windows application called UVC viewer.                              *
 *               -> Once the interface alternate setting is successful, you are now ready to   *
 *                  ask the camera to transfer image data. But, before the camera can send the *
 *                  data, a pipe needs to be set up to the endpoint that supports isochronous  *
 *                  transfers. The endpoint value is 0x81. The pipe is set up using            *
 *                  usbHstPipePrepare()function call.                                          *
 *               -> Once the pipe is set up, you can submit a URB (USB Request Block) that     *
 *                  asks the device to send isochronous data in form of packets of size less   *
 *                  than or equal to the requested size.                                       *
 *               -> Once the host recieves the data from the camera, the callback function is  *
 *                  called which seperates image data from the stream header and makes function*
 *                  calls to process the data, convert it to RGB and same it as a ppm file.    *
 *                                                                                             *
 * Note:        There is an additional abstraction layer called USBD (USB Driver) which is     *
 *              basically used for USB 1.1 compatibility but, its API can be used to write a   *
 *              driver for this camera. The reason for not using the USBD API is that the      *
 *              usbHst API is more transparent. The usbHst layer has a better description of   *
 *              errors and so, the user can look at the source code to find out where exactly  *
 *              the problem is. This is difficult when using the USBD API because it returns   *
 *              (-1) for any error. So, debugging is difficult if the USBD API is used.        *
 *                                                                                             *
 * References:  -> Standalone Linux Driver for Logitech C200 camera:Jay Khandhar, Brijen Raval *
 *                 and John Pratt. (http://bit.ly/1nzxQ9D)                                     *
 *              -> UVC Specification 1.5 (http://bit.ly/PhfNJ5)                                *
 *              -> USB Complete 3rd Edition by Jan Alexson (http://bit.ly/1kNaLjS)             *
 *              -> VxWorks USB Speaker driver located at $WIND_DIR/vxworks-6.9/target/h/drv/usb*
 **********************************************************************************************/

#include <vxWorks.h>
#include <string.h>
#include <ioLib.h>
#include <seqIo.h>
#include <errno.h>
#include <stdio.h>
#include <logLib.h>
#include <usb/usb.h>
#include <usb/usbHst.h>
#include <usb/usbOsal.h>
#include <usb/usbd.h>
#include <usb/usbPlatform.h>
#include <usb/ossLib.h>
#include <usb/usbListLib.h>
#include <usb/usbdLib.h>
#include <usb/usbLib.h>
#include <timers.h>
#include <time.h>
#include <tickLib.h>

#include "USB_Header.h"
#include "drv/timer/timerDev.h"
#include "usb/usbHubInitialization.h"

/*#define DEBUG*/             /* To enable the logMsgs */

/************************************************************
 *                                                          *
 *                          GLOBALS                         *
 *                                                          *
 ***********************************************************/

UCHAR data[26] = {0};               /* Data buffer to be used in control transfers */
UCHAR *isotrans_buffer;             /* Buffer where the image data will be stored */
UCHAR image_buffer[(HRES*VRES*2)];  /* Buffer where the image data will be copied for further processing */
UINT32 offset;                      /* To help in copying the data in the proper location as there are multiple transfers for sending a single image data */
UINT16 frameCount;                  /* To maintain the count of total frames processed */
UINT8 mem_h;                        /* Used to maintain and check the value of FID bit in the stream header */
UINT8 end_of_image; 
UINT8 aborted;
UINT8 first;


char bigBuffer[(VRES*HRES*3)];     /* Buffer to store the data after YUV to RGB conversion is performed */
char new_header[22]={'P','6','\n','#','t','e','s','t','\n','1','6','0',' ','1','2','0','\n','2','5','5','\n','\0'};
char ppm_dumpname[]="/tgtsvr/test00000000.ppm";

long double last_ticks = 0, last_jiffies = 0;
long double current_ticks = 0, current_jiffies = 0;
long double jiffies_per_tick, jiffies_per_second, microseconds_per_tick, microseconds_per_jiffy;

SEM_ID synch_sem;                               /* Descriptor for a binary semaphore */

pUSBHST_DEVICE_DRIVER pDriverData;              /* Structure to store details about the driver */


/*********************************************************
 * Function:     VOID shutDown(void)                     *
 * Description:  Deregisters the driver from the OS.     *
 *               The user will have to call camInit()    *
 *               again in order to use the camera again. *
 ********************************************************/

VOID shutDown(void)
{
    int status = 0;
    
    status = usbHstDriverDeregister((pUSBHST_DEVICE_DRIVER)pDriverData);
    
    #ifdef DEBUG

    logMsg("%s: Status = %d\n", __FUNCTION__, status,3,4,5,6);

    #endif
}


/***************************************************************
 * Function:     VOID fill_global(void)                        *
 * Description:  Initializes all the global variables required *
 *               in the initial stage                          *
 **************************************************************/

VOID fill_global(void)
{
    data[0] = dwFrameInterval;              /* We want to keep the frame interval constant to achieve a 30 fps frame rate*/
    data[1] = 0x00;                         /* The dwFrameInterval field is actually 2 bytes long. These are the 8 MSBs, and are reserved */
    data[2] = UNCOMPRESSED_FRAMES;    
    data[3] = RESOLUTION;
    data[4] = FPS_15_DATA_4;                /* These 3 bytes determine the value of dwFrameInnterval */
    data[5] = FPS_15_DATA_5;                /* The values are in multiples of 100 ns */
    data[6] = FPS_15_DATA_6;                /* The current value is equal to 33.33ms which gives the rate of 30 fps */
        
    first = 1;                              /* Used for synchronization of memcpy() and processImage() */
    frameCount = FRAME_COUNT;               /* Will create FRAME_COUNT number of ppm images and then stop execution */
    aborted = 0;                            /* To stop the execution of code if FRAME_COUNT images have been created */
}

/**************************************************************************
 * Function:     VOID initialize_timer(void)                              *
 * Description:  Sets the system clock rate, and initializes the timer    *
 *               related variables.                                       *
 *************************************************************************/

VOID initialize_timer(void)
{
    sysClkRateSet(1000);
    sysTimestampEnable();
    
    jiffies_per_tick   = (long double)sysTimestampPeriod();
    jiffies_per_second = (long double)sysTimestampFreq();
    
    microseconds_per_tick  = ((jiffies_per_tick/jiffies_per_second)*1000000.0);
    microseconds_per_jiffy = (microseconds_per_tick / jiffies_per_tick);
}

/**************************************************************************
 * Function:     VOID start_timer(void)                                   *
 * Description:  Gets the current value of ticks and jiffies and stores   *
 *               them.                                                    *
 *************************************************************************/

VOID start_timer(void)
{
    last_jiffies = sysTimestampLock();
    last_ticks   = tickGet();
}

/**************************************************************************
 * Function:    VOID stop_timer(void)                                     *
 * Description: Gets the current values of ticks and jiffies and stores   *
 *              them.                                                     *
 *                                                                        *
 *              Calculates the difference between the values obtained in  *
 *              this function and those obtained in the start_timer()     *
 *              function and from those, calculates the elapsed time in ms*
 *************************************************************************/

VOID stop_timer(void)
{
    long double tick_difference = 0, jiffy_difference = 0, micro_difference = 0;
    
    current_jiffies = sysTimestampLock();
    current_ticks = tickGet();
    
    tick_difference  = ((current_ticks - last_ticks)*microseconds_per_tick);
    jiffy_difference = ((current_jiffies - last_jiffies)*microseconds_per_jiffy);
    micro_difference = tick_difference + jiffy_difference;
    
    #ifdef DEBUG
    
    logMsg("%s: Time in milliseconds between two frames = %d\n",__FUNCTION__, (int)(micro_difference/1000),3,4,5,6);
    
    #endif
}

/********************************************************************************************************************************
 * Function:     USBHST_STATUS Add_device_Callback(UINT32 hDevice, UINT8 uInterfaceNumber, UINT8 uSpeed, void **pDriverData)    *
 * Description:  The function is called when a device with matching device driver details is attached (basically any UVC camera)*
 *                                                                                                                              *
 *               It configures the device, negotiates bandwidth using control transfers, sets the alternate settings for the    *
 *               video streaming interface, creates a  pipe for isochronous transfers between the camera and the host and calls *
 *               the ISOCHRONOUS_TRANSFER() function which sends URBs for getting the data from the camera using isochronous    *
 *               transfers.                                                                                                     *
 *******************************************************************************************************************************/ 

USBHST_STATUS Add_Device_Callback(UINT32 hDevice, UINT8 uInterfaceNumber, UINT8 uSpeed, void **pDriverData)
{
    UINT32 size = 0;
    UINT8 i = 0;
    UINT8 numConfig = 0;
    UCHAR curr_config = 0;
    INT8 temp_status = 0;
    
    #ifdef DEBUG
    
    logMsg("%s: In add device callback function. decive handle = %d, interface = %d, speed = %d \n",__FUNCTION__, hDevice, uInterfaceNumber, uSpeed,5,6);

    #endif
    
    
    /* Before the device's interface or any of it's alternate settings are activated, the device needs to be in the
     * configured state. usbHstSetConfiguration() will change the state of the device from default (or addressed)
     * to configured */
    
    temp_status = usbHstGetConfiguration(hDevice, &curr_config);
    
    #ifdef DEBUG
    
    logMsg("%s: Get config temp_status = %d, calue = %d\n",__FUNCTION__,temp_status, curr_config,4,5,6);
    
    #endif
    
    if(temp_status != OK)
    {
        shutDown();
        
        return ERROR;
    }
    
    temp_status = usbHstSetConfiguration(hDevice, curr_config);
    
    #ifdef DEBUG
    
    logMsg("%s: Get config temp_status = %d, calue = %d\n",__FUNCTION__,temp_status, curr_config,4,5,6);
    
    #endif
    
    if(temp_status != OK)
    {
        shutDown();
        
        return ERROR;
    }
    
    /* The host probes the device for configuration data for configuring the device to send 160x126 uncompressed frame
     * data. */
    
    if(USBHST_SUCCESS != Control_Transfer(hDevice, USB_DIRECTION_OUT, USB_SET_CURRENT, UVC_VS_PROBE_CONTROL, USB_SETUP_PACKET_INDEX))
    {
        #ifdef DEBUG
        
        logMsg("%s: Control Transfer 1 failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        shutDown();
        
        OSS_FREE(pDriverData);
        
        return USBHST_FAILURE;
    }
    else
    {
        #ifdef DEBUG
        
        logMsg("%s: Control Transfer 1 Succeeded.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
    }
    
/*  memset(data, 0, sizeof(data));    Clear the array so as to store new information after control transfer 2 */
    
    /* The device sends the configuration data to the host when the following control transfer URB is submitted
     * Based on this configuration data, the host can then send a third control transfer with a VS_COMMIT flag
     * to actually configure the device. */
    
    if(USBHST_SUCCESS != Control_Transfer(hDevice, USB_DIRECTION_IN, USB_GET_CURRENT, UVC_VS_PROBE_CONTROL, USB_SETUP_PACKET_INDEX))
    {
        #ifdef DEBUG
            
        logMsg("%s: Control Transfer 2 failed.\n",__FUNCTION__,2,3,4,5,6);
            
        #endif
        
        shutDown();
        
        OSS_FREE(pDriverData);
    
        return USBHST_FAILURE;
    }
    else
    {
        #ifdef DEBUG
            
        logMsg("%s: Control Transfer 2 Succeeded.\n",__FUNCTION__,2,3,4,5,6);
            
        #endif
    }
    
    /* This is where the host actually configures the device to send 160x120 uncompressed frames */
    
    if(USBHST_SUCCESS != Control_Transfer(hDevice, USB_DIRECTION_OUT, USB_SET_CURRENT, UVC_VS_COMMIT_CONTROL, USB_SETUP_PACKET_INDEX))
    {
        #ifdef DEBUG
            
        logMsg("%s: Control Transfer 3 failed.\n",__FUNCTION__,2,3,4,5,6);
            
        #endif
        
        shutDown();
        
        OSS_FREE(pDriverData);
    
        return USBHST_FAILURE;
    }
    else
    {
        #ifdef DEBUG
            
        logMsg("%s: Control Transfer 3 Succeeded.\n",__FUNCTION__,2,3,4,5,6);
            
        #endif
    }

    /* Since the device is in configured state, the video streaming interface and its alternate setting can be selected */
    
    temp_status = usbHstSetInterface(hDevice, INTERFACE,ALTERNATE_INTERFACE);
    
    #ifdef DEBUG
    
    logMsg("%s: Set interface status = %d\n",__FUNCTION__,temp_status,3,4,5,6);

    #endif
    
    if(temp_status != OK)
    {
        shutDown();
        
        return ERROR;
    }
    
    /* Before requesting the image data from the camera, a pipe needs to be established with the isochronous tranfer
     * endpoint. You need to specify the max number of bytes that will be recieved  per transfer and the max number 
     * of transfers that are going to take place via that pipe. */
    
    pUSB_TRANSFER_SETUP_INFO pSetupInfo;
    
    pSetupInfo = (USB_TRANSFER_SETUP_INFO *)OSS_CALLOC(sizeof(USB_TRANSFER_SETUP_INFO));
    
    if(pSetupInfo == NULL)
    {
        #ifdef DEBUG
        
        logMsg("%s: OSS_CALLOC for pSetupInfo failed.\n",__FUNCTION__,2,3,4,5,6 );
        
        #endif
        
        shutDown();
        
        return ERROR;
    }
        
    pSetupInfo->uMaxNumReqests   = NO_OF_TRANSFERS;
    pSetupInfo->uMaxTransferSize = NUMBER_OF_ISOCHRONOUS_PACKETS*ISOCHRONOUS_BUFFER_SIZE;  
    pSetupInfo->uFlags           = 0;
        
    temp_status = usbHstPipePrepare(hDevice, ISOCHRONOUS_TRANSFER_ENDPOINT_INTERFACE_1, pSetupInfo);
    
    if(temp_status != USBHST_SUCCESS)
    {
        #ifdef DEBUG
        
        logMsg("%s: Pipe prepare failed. Status = %d\n",__FUNCTION__,temp_status,3,4,5,6);
        
        #endif
        
        shutDown();
        
        return ERROR;
    }
    else
    {
        #ifdef DEBUG
        
        logMsg("%s: Pipe prepared successfully. Status = %d",__FUNCTION__,temp_status,3,4,5,6);
        
        #endif
    }
    
    for(i = 0; i < NO_OF_TRANSFERS; i++)
    {
        if(USBHST_SUCCESS != Isochronous_Transfer(hDevice, ISOCHRONOUS_TRANSFER_ENDPOINT_INTERFACE_1, USBHST_START_ISOCHRONOUS_TRANSFER_ASAP | USB_FLAG_SHORT_OK, USBHST_SUCCESS))
        {
            #ifdef DEBUG
        
            logMsg("%s: Isochronous Transfer failed.\n",__FUNCTION__,2,3,4,5,6);
        
            #endif
        
            shutDown();
            
            OSS_FREE(pDriverData);
        
            return USBHST_FAILURE;
        }
    }
    return USBHST_SUCCESS;
}

/**************************************************************************************
 * Function:      VOID Remove_Device_Callback(UINT32 hDevice, void *pDriverData)      *
 * Description:   This function is called when the device with matching device driver *
 *                details is removed.                                                 *
 *************************************************************************************/

VOID Remove_Device_Callback(UINT32 hDevice, void *pDriverData)
{   
    
    #ifdef DEBUG
    
    logMsg("%s: In remove device callback function\n",__FUNCTION__,2,3,4,5,6);

    #endif

    shutDown();
    
    return;
}

/**************************************************************************************
 * Function:      VOID Suspend_Device_Callback(UINT32 hDevice, void *pDriverData)     *
 * Description:   This function is called when the device with matching device driver *
 *                details is suspended.                                               *
 *************************************************************************************/

VOID Suspend_Device_Callback(UINT32 hDevice, void *pDriverData)
{   
    #ifdef DEBUG
    
    logMsg("%s: In suspend device callback function\n",__FUNCTION__,2,3,4,5,6);
    
    #endif
    
    return;
}

/**************************************************************************************
 * Function:      VOID Resume_Device_Callback(UINT32 hDevice, void *pDriverData)      *
 * Description:   This function is called when the device with matching device driver *
 *                details is resumed.                                                 *
 *************************************************************************************/

VOID Resume_Device_Callback(UINT32 hDevice, void *pDriverData)
{   
    #ifdef DEBUG
    
    logMsg("%s: In resume device callback function\n",__FUNCTION__,2,3,4,5,6);

    #endif
    
    return;
}

/*********************************************************
 * Function:     VOID camInit(void)                      *
 * Description:  Initializes the timer, fills the driver *
 *               data structure and registers the driver *
 *               with the OS.                            *
 ********************************************************/

VOID camInit(void)
{
    USBHST_STATUS status;
    
    fill_global();                              /* Fill the data array */
    initialize_timer();
    start_timer();                              /* Only for the first frame */
    
    synch_sem = semBCreate(SEM_Q_FIFO, SEM_FULL);
    
    isotrans_buffer = (UCHAR *)malloc(ISOCHRONOUS_TRANSFER_LENGTH);
    if(isotrans_buffer == NULL)
    {
        #ifdef DEBUG
        
        logMsg("%s: malloc for isotrans_buffer failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        return;
    }
    
    memset(isotrans_buffer, 0, ISOCHRONOUS_TRANSFER_LENGTH);
        
    if((pDriverData = OSS_CALLOC(sizeof(USBHST_DEVICE_DRIVER))) == NULL)
    {
        #ifdef DEBUG    
        
        logMsg("%s: Calloc for pDriveData failed\n",__FUNCTION__,2,3,4,5,6);
        
        #endif

        return;
    }
    else
    {
        #ifdef DEBUG
    
        logMsg("%s: Calloc for pDriverData succeeded\n",__FUNCTION__,2,3,4,5,6);
    
        #endif
    }
    
    pDriverData->bFlagVendorSpecific             = NO;
    pDriverData->uVendorIDorClass                = USB_CLASS_MISC;
    pDriverData->uProductIDorSubClass            = USB_SUBCLASS_COMMON;
    pDriverData->uBCDUSBorProtocol               = USB_PROTOCOL_IAD;
    pDriverData->addDevice                       = Add_Device_Callback;
    pDriverData->removeDevice                    = Remove_Device_Callback;
    pDriverData->suspendDevice                   = Suspend_Device_Callback;
    pDriverData->resumeDevice                    = Resume_Device_Callback;
    
    status = usbHstDriverRegister(pDriverData, NULL, "Logitech C200 Camera");
    
    if(status != USBHST_SUCCESS)
    {
        OSS_FREE(pDriverData);
        
        #ifdef DEBUG
        
        logMsg("%s: usb host driver register failed\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        return;
    }
    else
    {
        #ifdef DEBUG
    
        logMsg("%s: status for usbHstDriverRegister = %d\n",__FUNCTION__, status,3,4,5,6);
    
        #endif  
    }
}

/**********************************************************************************
 * Function:     USBHST_STATUS Control_Completion_Callback(pUSBHST_URB pUrb)      *
 * Description:  Checks if the pointer pUrb is not a NULL pointer which indicates *
 *               an error.                                                        *
 *********************************************************************************/

USBHST_STATUS Control_Completion_Callback(pUSBHST_URB pUrb)
{
    #ifdef DEBUG
    
    logMsg("%s: In the control completion callback function.\n",__FUNCTION__,2,3,4,5,6);

    #endif
    
    if(pUrb == NULL)
    {
        #ifdef DEBUG
        
        logMsg("%s: pUrb = NULL \n",__FUNCTION__,2,3,4,5,6);
            
        #endif
        
        shutDown();
            
        return USBHST_FAILURE;
    }
        
    OS_RELEASE_EVENT((OS_EVENT_ID)pUrb->pContext);
        
    return USBHST_SUCCESS;
}

/*********************************************************************************************************************************
 * Function:    USBHST_STATUS Control_Transfer(UINT32 hDevice, UINT8 uRequestType, UINT8 uRequest, UINT16 uValue, UINT16 uIndex) *
 * Description: Creates and fills up a setup packet based on the arguments passed with the call. This setup packet determines the*
 *              type of request that the host is sending the device.                                                             *
 *                                                                                                                               *
 *              Creates a URB for control transfer and submits it.                                                               *
 ********************************************************************************************************************************/

USBHST_STATUS Control_Transfer(UINT32 hDevice, UINT8 uRequestType, UINT8 uRequest, UINT16 uValue, UINT16 uIndex)
{   
    #ifdef DEBUG
    
    UCHAR i = 0;
    
    for(i = 0; i < 26; i++)
    {
        logMsg("%s: data = %x \n",__FUNCTION__,data[i],3,4,5,6);
    }
    
    #endif
    
    pUSBHST_URB pUrb;
    
    USBHST_STATUS nStatus = USBHST_SUCCESS;
    
    OS_EVENT_ID EventId;
    
    pUrb = (USBHST_URB *)OSS_CALLOC(sizeof(USBHST_URB));
    
    if(NULL == pUrb)
    {
        #ifdef DEBUG
        
        logMsg("%s: OSS_CALLOC for pUrb failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        shutDown();
        
        return ERROR;
    }
    
    memset(pUrb, 0, sizeof(USBHST_URB));
    
    EventId = OS_CREATE_EVENT(OS_EVENT_NON_SIGNALED);                  /* Creates a context for the transfer */
    
    #ifdef DEBUG
    
    logMsg("%s: Event id is %d\n",__FUNCTION__,EventId,3,4,5,6);
    
    #endif
    
    pUSBHST_SETUP_PACKET pSetupPacket; 
        
    pSetupPacket = (USBHST_SETUP_PACKET *)OSS_CALLOC(sizeof(USBHST_SETUP_PACKET));
    
    if(pSetupPacket == NULL)
    {
        #ifdef DEBUG
        
        logMsg("%s: OSS_CALLOC for pSetupPacket failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        shutDown();
        
        return ERROR;
    }
    
    USBHST_FILL_SETUP_PACKET(pSetupPacket, uRequestType, uRequest, uValue, uIndex, sizeof(data));    
    
    #ifdef DEBUG
    
    logMsg("%s:Req type = %x, request = %x, value = %x, index = %x, size = %d\n",__FUNCTION__, pSetupPacket->bmRequestType, pSetupPacket->bRequest,OS_UINT16_LE_TO_CPU(pSetupPacket->wValue), OS_UINT16_LE_TO_CPU(pSetupPacket->wIndex), OS_UINT16_LE_TO_CPU(pSetupPacket->wLength));
    
    #endif
    
    USBHST_FILL_CONTROL_URB(pUrb, hDevice, CONTROL_TRANSFER_ENDPOINT, &data[0], sizeof(data), USBHST_SHORT_TRANSFER_OK /* Source: www.jungo.com/st/support/tech_docs/td107.html - Says that control transfers are always short transfers */, pSetupPacket, Control_Completion_Callback, EventId, USBHST_SUCCESS);
    
    #ifdef DEBUG
    
    logMsg("%s: After filling the control URB\n",__FUNCTION__,2,3,4,5,6);
    logMsg("hdevice = %d, Endpoint = %d, Transfer length = %d, Transfer flags = %d, context = %d, status = %d\n", pUrb->hDevice, pUrb->uEndPointAddress, pUrb->uTransferLength, pUrb->uTransferFlags, pUrb->pContext, pUrb->nStatus);
    logMsg("%s: pUrb = %x, Address = %x",__FUNCTION__,pUrb, &pUrb,4,5,6);
    
    #endif
    
    nStatus = usbHstURBSubmit(pUrb);
    
    if(nStatus == USBHST_SUCCESS)
    {
        #ifdef DEBUG
        
        logMsg("%s: usbHstUrbSubmit was successful\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        OS_WAIT_FOR_EVENT(EventId, OS_WAIT_INFINITE);
        
        nStatus = pUrb->nStatus;
    }   
    
    OSS_FREE(pSetupPacket);
    OSS_FREE(pUrb);
    OS_DESTROY_EVENT(EventId);
    
    #ifdef DEBUG
    
    logMsg("%s: Control transfer nStatus = %d\n",__FUNCTION__, nStatus,3,4,5,6);
    
    for(i = 0; i < 26; i++)
    {
        logMsg("%s: data = %x \n",__FUNCTION__,data[i],3,4,5,6);
    }
    
    #endif
    
    return nStatus;
}

/*****************************************************************************************
 * Function:     USBHST_STATUS Isochronous_Completion_Callback(pUSHBST_URB pUrb)         *
 * Description:  This callback function is called when the host revieves data from the   *
 *               camera.                                                                 *
 *                                                                                       *
 *               On recieving the data, the host checks the length field of all the      *
 *               isochronous packet descriptor structures. If the value of the field of  *
 *               any descriptor is 12, that means that the camera has sent only the      *
 *               stream header in the corresponding packet. There is not data. We ignore *
 *               the corresponding section of the transfer buffer (here - buffer pointed *
 *               by pUrb->pTransferBuffer). If the length is greater than 12 bytes, then *
 *               the value of the FID bit of the header is compared to its value in the  *
 *               previous transfer. If the value has changed, that means that the current*
 *               frame contains the data of a new image. So, before copying the current  *
 *               data in the image_buffer and thus overwriting the image data, the data  *
 *               in the image_buffer is processed (converted to RGB and saved as a PPM   *
 *               file) and after that, the current data is copied to the image_buffer.   *
 *                                                                                       *
 *               If the FID bit has not changed, then the data is simply copied to the   *
 *               image_buffer and the loop continues.                                    *
 *                                                                                       *
 *               Once the data corresponding to all the isochronous packet descriptors   *
 *               has been analyzed, the Urb is again filled and submitted to recieve new *
 *               data from the camera.                                                   *
 ****************************************************************************************/

USBHST_STATUS Isochronous_Completion_Callback(pUSBHST_URB pUrb)
{
    UINT16 i = 0;
    UINT32 taskId;
    
    pUSBHST_ISO_PACKET_DESC pIsochronous_Packet_Descriptor;
    
    pIsochronous_Packet_Descriptor = pUrb->pTransferSpecificData;

    for(i = 0; i < NUMBER_OF_ISOCHRONOUS_PACKETS; i++)
    {
        if(pIsochronous_Packet_Descriptor[i].uLength != HEADER_LENGTH)                    
        {
            if(mem_h != (pUrb->pTransferBuffer[(i*ISOCHRONOUS_BUFFER_SIZE) + 1] & 0x01))  /* Check for the FID bit */
            {
                mem_h = (pUrb->pTransferBuffer[(i*ISOCHRONOUS_BUFFER_SIZE) + 1] & 0x01);
                end_of_image  = 1;
                offset = 0;
                frameCount--;
                if(frameCount == 0)
                {
                    aborted = 1;
                }               
                
                if(taskSpawn("processImage", 51, 0, 6000, processImage, image_buffer, (UINT32)(HRES*VRES*2), 0, 0, 0, 0, 0, 0, 0, 0) == ERROR)
                {
                    logMsg("Process image task spawn failed\n",1,2,3,4,5,6);
                    
                    shutDown();
                    
                    return ERROR;
                }
                                
                /*processImage(image_buffer, (UINT32)(HRES*VRES*2));*/
                
                first = 1;
                
                memset(image_buffer, 0, sizeof(image_buffer));        /* Clear the image_buffer after a complete frame has been processed */
                
                void* buffer_ptr = (void *)pUrb->pTransferBuffer;
                memcpy((void *)(image_buffer + offset), (const void *)(buffer_ptr + ((i * ISOCHRONOUS_BUFFER_SIZE) + HEADER_LENGTH)), (pIsochronous_Packet_Descriptor[i].uLength) - HEADER_LENGTH);
                offset += pIsochronous_Packet_Descriptor[i].uLength - HEADER_LENGTH;
            }
            else
            {
                if(first == 1)
                {
                    semTake(synch_sem, WAIT_FOREVER);
                    first = 0;
                }
                void* buffer_ptr = (void *)pUrb->pTransferBuffer;
                memcpy((void *)(image_buffer + offset), (const void *)(buffer_ptr + ((i * ISOCHRONOUS_BUFFER_SIZE) + HEADER_LENGTH)), (pIsochronous_Packet_Descriptor[i].uLength) - HEADER_LENGTH);
                offset += pIsochronous_Packet_Descriptor[i].uLength - HEADER_LENGTH;
            }
        }
        if(pIsochronous_Packet_Descriptor[i].nStatus != USBHST_SUCCESS)
        {
            #ifdef DEBUG
            
            logMsg("%s: The packet %d has status %d\n",__FUNCTION__, i, pIsochronous_Packet_Descriptor[i].nStatus,4,5,6);
            
            #endif
            
            continue;
        }
    }
    
    /* Refill the URB and submit it */
    for(i = 0; i < NUMBER_OF_ISOCHRONOUS_PACKETS; i++)
    {
        pIsochronous_Packet_Descriptor[i].uLength = ISOCHRONOUS_BUFFER_SIZE;
        pIsochronous_Packet_Descriptor[i].uOffset = i*ISOCHRONOUS_BUFFER_SIZE;
    }
    if(!aborted)
    {
        usbHstURBSubmit(pUrb);
    }

    return USBHST_SUCCESS;
}

/******************************************************************************************************************************************
 * Function:     USBHST_STATUS Isochronous_Transfer(UINT32 hDevice, UINT8 uEndpointAddress, UINT32 uTransferFlags, USBHST_STATUS nStatus) *
 * Description:  This function allocates memory for the URB structure and the isochronous packet descriptors based on the number of       *
 *               packets specified.                                                                                                       *
 *                                                                                                                                        *
 *               Then, it fills the isochronous packet descriptors based on the buffer size and also fills the offset filed of the        *
 *               descriptors.                                                                                                             *
 *                                                                                                                                        *
 *               It creates a URB for isochronous transfers and submits it.                                                               *
 *****************************************************************************************************************************************/

USBHST_STATUS Isochronous_Transfer(UINT32 hDevice, UINT8 uEndpointAddress, UINT32 uTransferFlags, USBHST_STATUS nStatus)
{
    UINT8 i = 0;

    pUSBHST_URB pUrb;
    
    OS_EVENT_ID Event_Id;
        
    pUrb = (USBHST_URB *)OSS_CALLOC(sizeof(USBHST_URB));
    
    if(NULL == pUrb)
    {
        #ifdef DEBUG
        
        logMsg("%s: OSS_CALLOC for pUrb failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        shutDown();
        
        return ERROR;
    }
    
    memset(pUrb, 0, sizeof(USBHST_URB));
    
    Event_Id = OS_CREATE_EVENT(OS_EVENT_NON_SIGNALED);
    
    #ifdef DEBUG
    
    logMsg("%s: Event id is %d\n",__FUNCTION__,Event_Id,3,4,5,6);
    
    #endif

    pUSBHST_ISO_PACKET_DESC pIsochronous_Packet_Descriptor;
    pIsochronous_Packet_Descriptor = OSS_CALLOC((UINT32)NUMBER_OF_ISOCHRONOUS_PACKETS*(sizeof(USBHST_ISO_PACKET_DESC)));
    
    if(pIsochronous_Packet_Descriptor == NULL)
    {
        #ifdef DEBUG
        
        logMsg("%s: OSS_CALLOC for pIsochronous_Packet_Descriptor failed.\n",__FUNCTION__,2,3,4,5,6);
        
        #endif
        
        shutDown();
        
        return USBHST_FAILURE;
    }
    
    for(i = 0; i < NUMBER_OF_ISOCHRONOUS_PACKETS; i++)
    {
        pIsochronous_Packet_Descriptor[i].uLength = ISOCHRONOUS_BUFFER_SIZE;
        pIsochronous_Packet_Descriptor[i].uOffset = ISOCHRONOUS_BUFFER_SIZE*(UINT32)i;
        pIsochronous_Packet_Descriptor[i].nStatus = USBHST_SUCCESS;
    }
        
    USBHST_FILL_ISOCHRONOUS_URB(pUrb, hDevice, uEndpointAddress, isotrans_buffer, ISOCHRONOUS_TRANSFER_LENGTH, uTransferFlags, 1, NUMBER_OF_ISOCHRONOUS_PACKETS, pIsochronous_Packet_Descriptor, Isochronous_Completion_Callback, Event_Id, USBHST_SUCCESS);
    
    #ifdef DEBUG
    
    logMsg("%s:After filling the isochronous urb.\n Endpoint Address = %x\n no of packets = %d\n  Total length = %d\n ",__FUNCTION__, pUrb->uEndPointAddress, pUrb->uNumberOfPackets, pUrb->uTransferLength,5,6);
    
    #endif
    
    nStatus = usbHstURBSubmit(pUrb);
    if(nStatus == USBHST_SUCCESS)
    {
        #ifdef DEBUG
            
        logMsg("%s: usbHstUrbSubmit was successful\n",__FUNCTION__,2,3,4,5,6);
            
        #endif
            
        OS_WAIT_FOR_EVENT(Event_Id, OS_WAIT_INFINITE);
            
        nStatus = pUrb->nStatus;
    }   
    /*OSS_FREE(pUrb);
    OS_DESTROY_EVENT(Event_Id);*/
    
    return nStatus;
}

/********************************************************************
 * Function:      VOID processImage(const void *p, UINT32 size)     *
 * Description:   This function takes in a buffer and the size of   *
 *                data to be processed as an input.                 *
 *                                                                  *
 *                Then, it seperates the YUV 422 data into YUYV     *
 *                format and calls the YUV2RGB function to convert  *
 *                the data to RGB format and then calls the         *
 *                dump_ppm function to save the data as a PPM image.*
 *******************************************************************/

VOID processImage(const void *p, UINT32 size)
{
    UINT32 i = 0, newi = 0;
    INT16 y_temp = 0, y2_temp = 0, u_temp = 0, v_temp = 0;
    UCHAR *pptr = (UCHAR *)p;
    
    for(i = 0, newi = 0; i < size; i += 4, newi += 6)
    {
        y_temp  = (int)pptr[i];
        u_temp  = (int)pptr[i + 1];
        y2_temp = (int)pptr[i + 2];
        v_temp  = (int)pptr[i + 3];
        YUV2RGB(y_temp, u_temp, v_temp, &bigBuffer[newi], &bigBuffer[newi + 1], &bigBuffer[newi + 2]);
        YUV2RGB(y2_temp, u_temp, v_temp, &bigBuffer[newi + 3], &bigBuffer[newi + 4], &bigBuffer[newi + 5]);
    }   
    
    dump_ppm(bigBuffer, (UINT32)((size*6)/4), frameCount);
    
    stop_timer();
    start_timer();     /* For the next frame */
}

/*****************************************************************************
 * Function:    VOID YUV2RGB(int y, int u, int v, char *r, char *g, char *b) *
 * Description: This function takes YUV data as an input, converts it to     *
 *              RGB format and stores them at the appropriate location.      *
 ****************************************************************************/

VOID YUV2RGB(int y, int u, int v, char *r, char *g, char *b)
{
    int r1 = 0, g1 = 0, b1 = 0;
    int c = y - 16; 
    int d = u -128;
    int e = v - 128;
    int f = (298 * c);
    
    /*r1 = (298 * c           + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d           + 128) >> 8;*/

    r1 = (f           + 409 * e + 128) >> 8;
    g1 = (f - 100 * d - 208 * e + 128) >> 8;
    b1 = (f + 516 * d           + 128) >> 8;
    
    if(r1 > 255)
    {
        r1 = 255;
    }

    if(g1 > 255)
    {
        g1 = 255;
    }
    
    if(b1 > 255)
    {
        b1 = 255;
    }
    
    if(r1 < 0)
    {
        r1 = 0;
    }

    if(g1 < 0)
    {
        g1 = 0;
    }
    
    if(b1 < 0)
    {
        b1 = 0;
    }
    
    *r = r1;
    *g = g1;
    *b = b1;

    return;
}

/*************************************************************************
 * Function:    VOID dump_ppm(char *p, UINT32 size, UINT16 tag)          *
 * Description: This function takes the RGB data as an input and creates *
 *              a ppm image file using that data.                        *
 ************************************************************************/

VOID dump_ppm(char *p, UINT32 size, UINT16 tag)
{
    UINT32 written = 0, total = 0, dumpfd = 0;
    
    snprintf(&ppm_dumpname[12], 9, "%08d", tag);
    strncat(&ppm_dumpname[20], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_CREAT | O_RDWR, 0666);
    
    written = write(dumpfd, new_header, 21);
    
    total = 0;
    
    do
    {
        written = write(dumpfd, p, size);
        total += written;
    }while(total < size);
    
    semGive(synch_sem);
    
    close(dumpfd);

    return;
}
