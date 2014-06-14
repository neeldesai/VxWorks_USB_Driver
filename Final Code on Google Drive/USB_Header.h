/**********************************************************************************************************
 * Name:         USB_Header.h																			  *
 * Author:       Neel Desai, University of Colorado - Boulder											  *
 * Date:         04/19/2014																				  *
 * Description:  -> The file contains the function declarations and macros used in the USB_Header.c code. *
 *               -> The code in USB_Header.c file is written such that it configures the camera to send   *
 *                  data for 160x120 uncompressed frames in the YUV 422 format. 						  *
 *               																						  * 																			
 *********************************************************************************************************/



/************************************************************
 * 															*
 * 							MACROS							*
 * 															*
 ***********************************************************/

/************ Control Transfer related macros **************/

#define USB_DIRECTION_OUT         						0x21
#define USB_DIRECTION_IN          						0xA1
#define USB_SET_CURRENT           						0x01
#define USB_GET_CURRENT           						0x81
#define USB_SETUP_PACKET_INDEX    						0x01
#define CONTROL_TRANSFER_ENDPOINT 						0x00
#define UVC_VS_PROBE_CONTROL    		    			0x100   
#define UVC_VS_COMMIT_CONTROL							0x200
#define dwFrameInterval                                 0x01
#define UNCOMPRESSED_FRAMES                             0x00
#define RESOLUTION                                      0x02     			/* 0x02 - 160x120 resolution; 0x04 - 320x240 resolution */
#define NO												0x00

/************ Isochronous Transfer related macros ***********/

#define ISOCHRONOUS_TRANSFER_ENDPOINT_INTERFACE_1       0x81
#define NUMBER_OF_ISOCHRONOUS_PACKETS       			12
#define HEADER_LENGTH                                   12
#define ISOCHRONOUS_BUFFER_SIZE             			944
#define ISOCHRONOUS_TRANSFER_LENGTH         			11328      			/*944*12 */
#define NO_OF_TRANSFERS                                 5
#define HRES                                            160
#define VRES                                            120

/************************* Other Macros *********************/

#define INTERFACE                                       1
#define ALTERNATE_INTERFACE                             6
#define FRAME_COUNT                                     500
#define FPS_30_DATA_4                                   0b00010101         	/* LSB */
#define FPS_30_DATA_5                                   0b00010110 			/* The value of the 3 byte integer equals (1/frame rate) in multiples of 100 ns */
#define FPS_30_DATA_6                                   0b00000101 			/* MSB */
#define FPS_15_DATA_4                                   0b00101010
#define FPS_15_DATA_5                                   0b00101100
#define FPS_15_DATA_6                                   0b00001010
#define FPS_10_DATA_4                                   0b01000000
#define FPS_10_DATA_5                                   0b01000010
#define FPS_10_DATA_6                                   0b00001111
#define FPS_05_DATA_4                                   0b10000000
#define FPS_05_DATA_5                                   0b10000100
#define FPS_05_DATA_6                                   0b00011110

/************************************************************
 * 															*
 * 							FUNCTIONS						*
 * 															*
 ***********************************************************/

/***************** Camera related functions ****************/

VOID camInit(void);
VOID fill_global(void);
VOID Remove_Device_Callback(UINT32 hDevice, void *pDriverData);
VOID Suspend_Device_Callback(UINT32 hDevice, void *pDriverData);
VOID Resume_Device_Callback(UINT32 hDevice, void *pDriverData);
USBHST_STATUS Add_Device_Callback(UINT32 hDevice, UINT8 uInterfaceNumber, UINT8 uSpeed, void **pDriverData);
VOID shutDown(void);

/**************** Transfer related functions ***************/

USBHST_STATUS Control_Transfer(UINT32 hDevice, UINT8 uRequestType, UINT8 uRequest, UINT16 uValue, UINT16 uIndex);
USBHST_STATUS Isochronous_Transfer(UINT32 hDevice, UINT8 uEndpointAddress, UINT32 uTransferFlags, USBHST_STATUS nStatus);
USBHST_STATUS Control_Completion_Callback(pUSBHST_URB pUrb);
USBHST_STATUS Isochronous_Completion_Callback(pUSBHST_URB pUrb);

/*************** Image processing functions ****************/

VOID processImage(const void *p, UINT32 size);
VOID YUV2RGB(int y, int u, int v, char *r, char *g, char *b);
VOID dump_ppm(char *p, UINT32 size, UINT16 tag);

/************** Timer related functions ********************/

VOID initialize_timer(void);
VOID start_timer(void);
VOID stop_timer(void);
