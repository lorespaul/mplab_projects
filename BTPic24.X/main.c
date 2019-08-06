#include <stdlib.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "USB/usb.h"
#include "USB/usb_host_generic.h"
#include "user.h"
#include "timer.h"

//#define DEBUG_MODE
// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************

        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_NODIV & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_SOSC & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_SOSC & DSBOREN_OFF & DSWDTEN_OFF)
 
// *****************************************************************************

// Application States
typedef enum
{
    BT_INITIALIZE = 0,                // Initialize the app when a device is attached
    BT_STATE_IDLE,                    // Inactive State
    BT_STATE_PROCESS,
	BT_STATE_WRITE_CLASS,
	BT_STATE_READ_EP1,
	BT_STATE_READ_CLASS_WAITING,
	BT_STATE_READ_ACL,
	BT_STATE_WRITE_ACL,
	BT_STATE_READ_ACL_WAITING,
    BT_STATE_ERROR                    // An error has occured
} BT_STATE;

// Hci States
typedef enum
{
	HCI_CMD_RESET = 0,                // Initialize the hci when a device is attached
		HCI_CMD_RESET_WRITE_END,
		HCI_CMD_RESET_READ_END,
	HCI_CMD_READ_BD_ADDR,
		HCI_CMD_READ_BD_ADDR_WRITE_END,
		HCI_CMD_READ_BD_ADDR_READ_END,
	HCI_CMD_LOCAL_NAME,
		HCI_CMD_LOCAL_NAME_WRITE_END,
		HCI_CMD_LOCAL_NAME_READ_END,
	HCI_CMD_SCAN_ENABLE,
		HCI_CMD_SCAN_ENABLE_WRITE_END,
		HCI_CMD_SCAN_ENABLE_READ_END,
	HCI_CMD_WAIT_CONNECTION,
		HCI_CMD_WAIT_CONNECTION_READ_END,
	HCI_CMD_CONNECTION_ACCEPT,
		HCI_CMD_CONNECTION_ACCEPT_WRITE_END,
		HCI_CMD_CONNECTION_ACCEPT_READ_END,
		HCI_CMD_CONNECTION_ACCEPT_READ_END1,
	ACL_GET_DATA,
		ACL_GET_DATA_READ_END,
	ACL_PUT_DATA,		
		ACL_PUT_DATA_END
} HCI_STATE;


// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************

BYTE	deviceAddress;  // Address of the device on the USB
BT_STATE  DemoState;      // Current state of the demo application
HCI_STATE  HciState;      // Current state of the demo application
WORD data_size;
int data_num;
unsigned char	local_bd_addr[6];//this 
unsigned char	remote_bd_addr[6];//PC
unsigned char buf[64];
unsigned char handle[2];//a handle for ACL 

//******************************************************************************
//******************************************************************************
// Local Routines
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        InitializeSystem
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         TRUE if successful, FALSE if not.
 *
 * Side Effects:    See below
 *
 * Overview:        This routine initializes the processor and peripheral,
 *                  setting clock speeds and enabling any required
 *                  features.
 *************************************************************************/

BOOL InitializeSystem ( void )
{

	unsigned int pll_startup_counter = 600;
	CLKDIVbits.PLLEN = 1;
	while(pll_startup_counter--);

	// Configure U2RX - put on pin 19 (RP8)
	RPINR19bits.U2RXR = 8;
	// Configure U2TX - put on pin 5  (RP7)
	RPOR3bits.RP7R = 5;
    // Init UART
    UART2Init();

    // Set Default demo state
    DemoState = BT_INITIALIZE;
	HciState=HCI_CMD_RESET;
    return TRUE;
} // InitializeSystem


/*************************************************************************
 * Function:        CheckForNewAttach
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          deviceAddress (global)
 *                  Updates the device address when an attach is found.
 *
 * Returns:         TRUE if a new device has been attached.  FALSE,
 *                  otherwise.
 *
 * Side Effects:    Prints attach message
 *
 * Overview:        This routine checks to see if a new device has been
 *                  attached.  If it has, it records the address.
 *************************************************************************/

BOOL CheckForNewAttach ( void )
{
    // Try to get the device address, if we don't have one.
    if (deviceAddress == 0)
    {
        GENERIC_DEVICE_ID DevID;

        DevID.vid   = 0x0A12;
        DevID.pid   = 0x0001;
        #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
            DevID.serialNumberLength = 0;
            DevID.serialNumber = NULL;
        #endif

        if (USBHostGenericGetDeviceAddress(&DevID))
        {
            deviceAddress = DevID.deviceAddress;
			#ifdef DEBUG_MODE
            UART2PrintString( "Generic demo device attached - polled, deviceAddress=" );
            UART2PutDec( deviceAddress );
            UART2PrintString( "\r\n" );
			#endif
            return TRUE;
        }
    }

    return FALSE;

} // CheckForNewAttach

/*************************************************************************
 * Function:        ManageDemoState
 *
 * Preconditions:   The DemoState global variable must be initialized to
 *                  BT_STATE_IDLE (0).  (This occurs on reset.)
 *
 * Input:           DemoState (global)
 *                  Actions selected based value of DemoState on function
 *                  entry.
 *
 *                  deviceAddress (global)
 *                  May use device address to access device, depending on
 *                  state.
 *
 *                  DataPacket (global)
 *                  May read data from packet buffer, depending on state.
 *
 * Output:          DemoState (global)
 *                  Updates demo state as appropriate.
 *
 *                  DataPacket (global)
 *                  May cause data in the packet buffer to be updated,
 *                  depending on state.
 *
 * Returns:         None
 *
 * Side Effects:    Depend on state transition
 *
 * Overview:        This routine maintains the state of the application,
 *                  updateing global data and taking actions as necessary
 *                  to maintain the custom demo operations.
 *************************************************************************/
void ManageDemoState ( void )
{
    BYTE RetVal;

    // Watch for device detaching
    if (USBHostGenericDeviceDetached(deviceAddress) && deviceAddress != 0)
    {
		#ifdef DEBUG_MODE
        UART2PrintString( "Generic demo device detached - polled\r\n" );
		#endif
        DemoState = BT_INITIALIZE;
		HciState=HCI_CMD_RESET;
        deviceAddress   = 0;
    }

    switch (DemoState)
    {
    case BT_INITIALIZE:
        DemoState = BT_STATE_IDLE;
        break;

    /** Idle State:  Loops here until attach **/
    case BT_STATE_IDLE:
        if (CheckForNewAttach())
        {
			DemoState = BT_STATE_PROCESS;
			HciState = HCI_CMD_RESET;
        }
        break;

    case BT_STATE_PROCESS:
		switch (HciState)
		{
//********************************************************************************
		case HCI_CMD_RESET:
			buf[0]=0x03;
			buf[1]=0x0c;
			buf[2]=0;
			data_size=3;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_RESET_WRITE_END;
		    break;

		case HCI_CMD_RESET_WRITE_END:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_RESET_READ_END;
		    break;

		case HCI_CMD_RESET_READ_END:
			if(buf[0]!=0x0e){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_RESET \r\n" );
			for(data_num=0;data_num<6;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = HCI_CMD_READ_BD_ADDR;
		    break;
//********************************************************************************
		case HCI_CMD_READ_BD_ADDR:
			buf[0]=0x09;
			buf[1]=0x10;
			buf[2]=0;
			data_size=3;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_READ_BD_ADDR_WRITE_END;
			break;

		case HCI_CMD_READ_BD_ADDR_WRITE_END:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_READ_BD_ADDR_READ_END;
		    break;

		case HCI_CMD_READ_BD_ADDR_READ_END:
			if(buf[0]!=0x0e){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_READ_BD_ADDR \r\n" );
			for(data_num=0;data_num<12;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = HCI_CMD_LOCAL_NAME;
		    break;
//********************************************************************************
		case HCI_CMD_LOCAL_NAME:
			buf[0]=0x13;
			buf[1]=0x0c;
			buf[2]=0x04;
			buf[3]='y';
			buf[4]='t';
			buf[5]='s';
			buf[6]=0x00;
			data_size=7;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_LOCAL_NAME_WRITE_END;
			break;

		case HCI_CMD_LOCAL_NAME_WRITE_END:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_LOCAL_NAME_READ_END;
		    break;

		case HCI_CMD_LOCAL_NAME_READ_END:
			if(buf[0]!=0x0e){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_LOCAL_NAME \r\n" );
			for(data_num=0;data_num<6;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = HCI_CMD_SCAN_ENABLE;
		    break;
//********************************************************************************
		case HCI_CMD_SCAN_ENABLE:
			buf[0]=0x1a;
			buf[1]=0x0c;
			buf[2]=0x01;
			buf[3]=0x03;
			data_size=4;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_SCAN_ENABLE_WRITE_END;
			break;

		case HCI_CMD_SCAN_ENABLE_WRITE_END:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_SCAN_ENABLE_READ_END;
		    break;

		case HCI_CMD_SCAN_ENABLE_READ_END:
			if(buf[0]!=0x0e){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_SCAN_ENABLE \r\n" );
			for(data_num=0;data_num<6;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = HCI_CMD_WAIT_CONNECTION;
		    break;
//********************************************************************************
		case HCI_CMD_WAIT_CONNECTION:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_WAIT_CONNECTION_READ_END;
			break;

		case HCI_CMD_WAIT_CONNECTION_READ_END:
			if(buf[0]!=0x04){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_WAIT_CONNECTION \r\n" );
			for(data_num=0;data_num<12;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = HCI_CMD_CONNECTION_ACCEPT;
		    break;
//********************************************************************************
		case HCI_CMD_CONNECTION_ACCEPT:
			buf[8]=buf[7];//copy address
			buf[7]=buf[6];
			buf[6]=buf[5];
			buf[5]=buf[4];
			buf[4]=buf[3];
			buf[3]=buf[2];
			buf[0]=0x09;
			buf[1]=0x04;
			buf[2]=0x07;
			buf[9]=0x01;//role
			data_size=10;
			DemoState = BT_STATE_WRITE_CLASS;
			HciState = HCI_CMD_CONNECTION_ACCEPT_WRITE_END;
			break;

		case HCI_CMD_CONNECTION_ACCEPT_WRITE_END:
			DemoState = BT_STATE_READ_EP1;
			HciState = HCI_CMD_CONNECTION_ACCEPT_READ_END;
		    break;

		case HCI_CMD_CONNECTION_ACCEPT_READ_END:
			if(buf[0]!=0x03){ DemoState =BT_STATE_READ_EP1; break;}
			handle[0]=buf[3];handle[1]=buf[4]+0x20;

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_CONNECTION_ACCEPT \r\n" );
			for(data_num=0;data_num<13;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			DemoState =BT_STATE_READ_EP1;
			HciState = HCI_CMD_CONNECTION_ACCEPT_READ_END1;

		case HCI_CMD_CONNECTION_ACCEPT_READ_END1:
			if(buf[0]!=0x20){ DemoState =BT_STATE_READ_EP1; break;}

			#ifdef DEBUG_MODE
			UART2PrintString( "HCI_CMD_CONNECTED \r\n" );
			for(data_num=0;data_num<9;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = ACL_GET_DATA;
		    break;
//********************************************************************************
// GET ACL DATA 
//********************************************************************************
		case  ACL_GET_DATA:
			DemoState = BT_STATE_READ_ACL;
			HciState = ACL_GET_DATA_READ_END;
			break;

		case ACL_GET_DATA_READ_END:

			#ifdef DEBUG_MODE
			UART2PrintString( "ACL_GET_DATA \r\n" );
			for(data_num=0;data_num<60;data_num++)
      	          {UART2PutHex(buf[data_num]);UART2PutChar(' ');}
			UART2PrintString( "\r\n" );
			#endif

			HciState = ACL_PUT_DATA;
		    break;


//********************************************************************************
// PUT ACL DATA 
//********************************************************************************

		case  ACL_PUT_DATA:
			buf[4]+=1;
 			data_size=7;
			DemoState = BT_STATE_WRITE_ACL;
			HciState = ACL_GET_DATA;
			break;

		case  ACL_PUT_DATA_END:
			break;

		}
		break;

//********************************************************************************
//@READ-WRITE FUNCTIONS
//********************************************************************************
//WRITE ENDPOINT 0
    case BT_STATE_WRITE_CLASS:
        if (!USBHostGenericTxIsBusy(deviceAddress))
		{
            if ( (RetVal=USBHostGenericClassRequest( deviceAddress, buf, data_size )) == USB_SUCCESS )
            {
			//UART2PrintString( "HCI COMMAND SENT\r\n" );	
            DemoState = BT_STATE_PROCESS;
            }
            else
            {
			UART2PrintString( "Write Class Error !\r\n" );	
            }
        }
        break;

//READ ENDPOINT 1
    case BT_STATE_READ_EP1:
        if (!USBHostGenericRxIsBusy(deviceAddress))
        {
            if ( (RetVal=USBHostGenericRead(deviceAddress, buf, DATA_PACKET_LENGTH)) == USB_SUCCESS )
            {
                DemoState = BT_STATE_READ_CLASS_WAITING;
                //UART2PrintString( "READ EP1\r\n" );
            }
            else
            {
                UART2PrintString( "Device Read Error !\r\n" );
            }
        }
        break;

    case BT_STATE_READ_CLASS_WAITING:
        if (!USBHostGenericRxIsBusy(deviceAddress))
           DemoState = BT_STATE_PROCESS;
        break;


//WRITE ENDPOINT 2
    case BT_STATE_WRITE_ACL:
        if (!USBHostGenericTxIsBusy(deviceAddress))
		{
            if ( (RetVal=USBHostGenericAclWrite( deviceAddress, buf, data_size )) == USB_SUCCESS )
            {
			//UART2PrintString( "HCI COMMAND SENT\r\n" );	
            DemoState = BT_STATE_PROCESS;
            }
            else
            {
			UART2PrintString( "Write Acl Error !\r\n" );	
            }
        }
        break;

//READ ENDPOINT 2
    case BT_STATE_READ_ACL:
        if (!USBHostGenericRxIsBusy(deviceAddress))
        {
            if ( (RetVal=USBHostGenericAclRead(deviceAddress, buf, DATA_PACKET_LENGTH)) == USB_SUCCESS )
            {
                DemoState = BT_STATE_READ_ACL_WAITING;
				#ifdef DEBUG_MODE
                UART2PrintString( "READ EP2\r\n" );
				#endif
            }
            else
            {
                UART2PrintString( "Read Acl Error !\r\n" );
            }
        }
        break;

    case BT_STATE_READ_ACL_WAITING:
        if (!USBHostGenericRxIsBusy(deviceAddress))
           DemoState = BT_STATE_PROCESS;
        break;


    /** Error state:  Hold here until detached **/
    case BT_STATE_ERROR:                          // To Do: Flash LEDs
        break;

    default:
        DemoState = BT_INITIALIZE;
        break;
    }

    DelayMs(1); // 1ms delay

} // ManageDemoState


//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        USB_ApplicationEventHandler
 *
 * Preconditions:   The USB must be initialized.
 *
 * Input:           event       Identifies the bus event that occured
 *
 *                  data        Pointer to event-specific data
 *
 *                  size        Size of the event-specific data
 *
 * Output:          deviceAddress (global)
 *                  Updates device address when an attach or detach occurs.
 *
 *                  DemoState (global)
 *                  Updates the demo state as appropriate when events occur.
 *
 * Returns:         TRUE if the event was handled, FALSE if not
 *
 * Side Effects:    Event-specific actions have been taken.
 *
 * Overview:        This routine is called by the Host layer or client
 *                  driver to notify the application of events that occur.
 *                  If the event is recognized, it is handled and the
 *                  routine returns TRUE.  Otherwise, it is ignored (or
 *                  just "sniffed" and the routine returns FALSE.
 *************************************************************************/

BOOL USB_ApplicationEventHandler ( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
        BYTE i;
    #endif

    // Handle specific events.
    switch (event)
    {
        case EVENT_GENERIC_ATTACH:
            if (size == sizeof(GENERIC_DEVICE_ID))
            {
                deviceAddress   = ((GENERIC_DEVICE_ID *)data)->deviceAddress;
                DemoState = BT_STATE_PROCESS; HciState=HCI_CMD_RESET;//YTS !!!!!!!!!!!!!!!!!
                UART2PrintString( "Generic demo device attached - event, deviceAddress=" );
                UART2PutDec( deviceAddress );
                UART2PrintString( "\r\n" );
                #ifdef USB_GENERIC_SUPPORT_SERIAL_NUMBERS
                    for (i=1; i<((GENERIC_DEVICE_ID *)data)->serialNumberLength; i++)
                    {
                        UART2PutChar( ((GENERIC_DEVICE_ID *)data)->serialNumber[i] );
                    }
                #endif
                UART2PrintString( "\r\n" );
                return TRUE;
            }
            break;

        case EVENT_GENERIC_DETACH:
            deviceAddress   = 0;
            DemoState = BT_INITIALIZE;
            UART2PrintString( "Generic demo device detached - event\r\n" );
            return TRUE;

        case EVENT_GENERIC_TX_DONE:           // The main state machine will poll the driver.
        case EVENT_GENERIC_RX_DONE:
            return TRUE;

        case EVENT_VBUS_REQUEST_POWER:
            // We'll let anything attach.
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            // We aren't keeping track of power.
            return TRUE;

        case EVENT_HUB_ATTACH:
            UART2PrintString( "\r\n***** USB Error - hubs are not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            UART2PrintString( "\r\n***** USB Error - device is not supported *****\r\n" );
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
            UART2PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            UART2PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            UART2PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
            UART2PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
            return TRUE;
            break;

        case EVENT_SUSPEND:
        case EVENT_DETACH:
        case EVENT_RESUME:
        case EVENT_BUS_ERROR:
            return TRUE;
            break;

        default:
            break;
    }

    return FALSE;

} // USB_ApplicationEventHandler


//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************

/*************************************************************************
 * Function:        main
 *
 * Preconditions:   None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Returns:         Never exits
 *
 * Side Effects:    Runs the application
 *
 * Overview:        This is the USB Custom Demo Application's main entry
 *                  point.
 *************************************************************************/

int main ( void )
{

  // Initialize the processor and peripherals.
    if ( InitializeSystem() != TRUE )
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - system.  Halting.\r\n\r\n" );
        while (1);
    }
    if ( USBHostInit(0) == TRUE )
    {
        UART2PrintString( "\r\n\r\n***** USB Custom Demo App Initialized *****\r\n\r\n" );
    }
    else
    {
        UART2PrintString( "\r\n\r\nCould not initialize USB Custom Demo App - USB.  Halting.\r\n\r\n" );
        while (1);
    }

    // Main Processing Loop
    while (1)
    {
        // This demo does not check for overcurrent conditions.  See the
        // USB Host - Data Logger for an example of overcurrent detection
        // with the PIC24F and the USB PICtail Plus.

        // Maintain USB Host State
        USBHostTasks();
        // Maintain Demo Application State
        ManageDemoState();
    }

    return 0;

} // main


/*************************************************************************
 * EOF main.c
 */

