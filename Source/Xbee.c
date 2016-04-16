/****************************************************************************
 Module
  Xbee.c

 Revision     Revised by: 
  1.0.0       Eric
  1.1.0       Denny

 Description
  Xbee communications over UART. This module runs the Xbee paired/unpaired
  state machine, responds to incoming xbee packets (pair/data requests), 
  and forms the data packet to send to the Xbee in order to transmit the 
  Gridlinq's measurement readings.

  Incomming data bytes are sent to the XbeeComms for decoding first before
  an xbeePacketReceived event is sent to this service.
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include "Xbee.h"
#include "XbeeComms.h"
#include "Gridlinq.h"
#include "Wind.h"
#include "GPS.h"
#include "LidarCom.h"
#include "CompassCom.h"
#include "PWM.h"
#include "Clamp.h"

/*----------------------------- Module Defines ----------------------------*/
// Usual framework defines
#define BitsPerNibble       4
#define TicksPerMS          40000       // 40Mhz
#define ALL_BITS            (0xff<<2)

// Xbee specific defines
#define MY_ADDRESS	        0x0C        // Team 12
#define NO_OPTIONS          0x00
#define BROADCAST           0x06

// Communication timeout for automatically unpairing
#define COMM_TIMEOUT		2000

// Receive byte index defines:
#define API_ID              0   // 0x89 when it's from ourselves, 0x81 when it's from another xbee
#define SOURCE_ADDR_MSB     1   // MSB of the source address
#define SOURCE_ADDR_LSB     2   // LSB of the source address
#define RSSI                3   // Receive signal strength
#define RX_OPTIONS          4   // Options, usually 0x00
#define RX_HEADER           5   // Needs to be 0x01 indicating an event
#define RX_EVENT_TYPE       6   // Indicates what the event type is (pair, unpair, disable magnets etc.)

// Transmit byte index defines:
#define PAIR_REQ_CHKSUM     9

#define DEST_ADDR_MSB       5
#define DEST_ADDR_LSB       6
#define TX_OPTIONS          7
#define TX_HEADER           8
#define WIND_SPEED1		    9
#define WIND_SPEED2		    10
#define WIND_SPEED_UNIT	    11
#define WIND_ANGLE1		    12
#define WIND_ANGLE2			13
#define TEMP1			    14
#define TEMP2			    15
#define TEMP_UNIT			16
#define GPS_LAT_DEG		    17
#define GPS_LAT_MIN1	    18
#define GPS_LAT_MIN2	    19
#define GPS_LAT_MIN3		20
#define LAT_NS				21
#define GPS_LONG_DEG1		22
#define GPS_LONG_DEG2		23
#define GPS_LONG_MIN1		24
#define GPS_LONG_MIN2		25
#define GPS_LONG_MIN3		26
#define LONG_EW				27
#define DIST1				28
#define DIST2				29
#define HEADING1			30
#define HEADING2			31
#define PITCH1              32
#define PITCH2              33
#define ROLL1               34
#define ROLL2               35
#define DATA_CHKSUM		    36

// Header masks
#define PAIR_REQ			0x01
#define PAIR_ACK            0x02
#define UNPAIR_REQ          0x03
#define UNPAIR_ACK          0x04
#define DATA_REQ            0x04
#define DATA_PACKET         0x05
#define EVENT               0x06
#define COMMAND_ACK         0x07

// Events (maybe keep this somewhere else - in an event response module)
#define AWAKEN              0x01
#define SLEEP               0x02
#define ENABLE_MAGNETS      0x03
#define DISABLE_MAGNETS     0x04


/*---------------------------- Module Functions ---------------------------*/
/* Prototypes for private functions for this service.*/
uint8_t checkSum(uint8_t chkArray[], uint8_t size);
bool isValidPacket(uint8_t chkArray[], uint8_t size);
void printPacket(uint8_t chkArray[], uint8_t size);

static void importData(Sensor s);
static void importXbeePacket(void);
static bool packetFromMaster(void);
static void LoadUpResponse(uint8_t header);
static void LoadUpData(void);
static void SendXbeeData(void);
static bool isValidPairRequest(void);

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static XbeeState CurrentState;
static uint8_t MasterAddrMSB = 0x21;    // Xbee receiving MSB
static uint8_t MasterAddrLSB = 0x8D;    // Xbee receiving LSB
static uint8_t rxData[12];              // incoming data array
static uint8_t txData[37];              // array for transmitting
static uint8_t* rx;
static uint8_t txIndex = 0;
static uint8_t txSize = 0;

// Structs to hold all measurement information locally
static Wind_Data CV7;
static GPS_Data GPSdata;
static LidarData_t lidarData;
static CompassReading_t compassData;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
    InitXbee

 Parameters
    uint8_t : the priorty of this service

 Returns
    bool, false if error in initialization, true otherwise

 Description
    Saves away the priority, sets the current state and posts initial event
****************************************************************************/
bool InitXbee ( uint8_t Priority ) {
    // Set priority and state
    MyPriority = Priority;
    CurrentState = xUnpairedState;
    printf("Xbee state machine initialized\r\n");
  
    // Send the initial transition event
    ES_Event ThisEvent = {ES_INIT, 0};
    return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    PostXbee

 Description
    Posts an event to this state machine's queue, returns bool successful
****************************************************************************/
bool PostXbee( ES_Event ThisEvent ) {
    return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunXbee

 Parameters
    ES_Event : the event to process

 Returns
    ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
    Runs the Xbee state machine, handling pairing/unpairing and sending data
****************************************************************************/
ES_Event RunXbee ( ES_Event CurrentEvent ) {
    ES_Event ReturnEvent = {ES_NO_EVENT};
    XbeeState NextState = CurrentState;
    
    switch(CurrentState) {
        case xUnpairedState:
            if(CurrentEvent.EventType == XbeePacketReceived) { 
                importXbeePacket();
                //printPacket(rxData, CurrentEvent.EventParam);
                if(isValidPairRequest()) {
                    MasterAddrMSB = rxData[SOURCE_ADDR_MSB];
                    MasterAddrLSB = rxData[SOURCE_ADDR_LSB];
                    NextState = xPairedState;
                    LoadUpResponse(PAIR_ACK);
                    SendXbeeData();
                    printf("Paired!\r\n");
                    ES_Timer_InitTimer(CommTimer, COMM_TIMEOUT);
                }
            }
            break;
            
        case xPairedState :
            if(CurrentEvent.EventType == XbeePacketReceived) {
                importXbeePacket();
                //printPacket(rxData, CurrentEvent.EventParam);
                if(!packetFromMaster()) break;
                if(rxData[RX_HEADER] == UNPAIR_REQ) {
                    printf("Unpaired from request\r\n");
                    NextState = xUnpairedState;
                    LoadUpResponse(UNPAIR_ACK);
                    SendXbeeData();
                }
                else if(rxData[RX_HEADER] == DATA_REQ) {
                    LoadUpData();
                    if(isValidPacket(txData, sizeof(txData))) {
                        SendXbeeData();
                    }
                    else {
                        printf("Failed to build packet");
                    }
                }
                else if(rxData[RX_HEADER] == EVENT) {
                    // Call the event handler... keep in a separate module 
                    if(rxData[RX_EVENT_TYPE] == AWAKEN) {
                        printf("Awaken event\r\n");
                        SetPWMDuty(0,BEACON_HACK);
                    }
                    else if(rxData[RX_EVENT_TYPE] == SLEEP) {
                        printf("Sleep\r\n");
                        SetPWMDuty(50,BEACON_HACK);
                        ES_Event nextEvent = {Disengage,0};
                        PostClamp(nextEvent);
                    }
                    if(rxData[RX_EVENT_TYPE] == ENABLE_MAGNETS) {
                        printf("Enable Magnets\r\n");
                    }
                    if(rxData[RX_EVENT_TYPE] == DISABLE_MAGNETS) {
                        printf("Dropping the gridlinq on its' ass\r\n");
                    }
                }
                
                ES_Timer_InitTimer(CommTimer, COMM_TIMEOUT);
            }
			else if(CurrentEvent.EventType == ES_TIMEOUT && CurrentEvent.EventParam == CommTimer) {
                printf("Unpaired from timeout.\r\n");
				NextState = xUnpairedState;
			}
			break;
    }
    
    CurrentState = NextState;
    return ReturnEvent;
}


/****************************************************************************
 Function
   importData

 Description
   moves the sensors current measurement information locally for loading
****************************************************************************/
static void importData(Sensor s) {
	switch(s) {
		case Wind :
			CV7 = GetWindData();
			break;
		case Distance :
            lidarData = getLidarData();
			break;
		case GPS :
			GPSdata = GetGPSData();
			break;
		case Compass :
            compassData = getCompassData();
			break;
	}
}

/****************************************************************************
 Function
    isValidPacket

 Parameters
    chkArray (array to check), size (size of array)

 Returns
    bool  validPacket

 Description
    Returns if the data we want to send is a valid Xbee packet
****************************************************************************/
bool isValidPacket(uint8_t chkArray[], uint8_t size) {
    if(size < 4) {
        printf("Size too small to be valid packet\r\n");
        return false;
    }
    if(chkArray[0] != 0x7E) {
        printf("Start delimiter invalid\r\n");
        return false;
    }
    uint16_t lengthBytes = (chkArray[1]<<8) + chkArray[2];
    if(lengthBytes != size - 4) {
        printf("Incorrect length\r\n");
        return false;
    }
    uint8_t chkSum = checkSum(chkArray, size);
    if(chkArray[size-1] != chkSum) {
        printf("Invalid checksum: Should be 0x%X\r\n", chkSum);
        return false;
    }
    return true;
}

/****************************************************************************
 Function
    checkSum

 Parameters
    chkArray (array to check), size (size of array)

 Returns
    uint8_t checksum

 Description
    Calculates the checkSum for a packet
****************************************************************************/
uint8_t checkSum(uint8_t chkArray[], uint8_t size) {
  uint8_t sum = 0;
  for(int i = 3; i < size-1; i++) {
    sum += chkArray[i];
  }
  return (0xFF - sum);
}

/****************************************************************************
 Function
   importXbeePacket

 Description
   Moves the data into memory for easier usibility rather than a pointer
****************************************************************************/
static void importXbeePacket(void) {
  rx = GetRxData();
  for(uint8_t i = 0; i < sizeof(rxData); i++) {
    rxData[i] = *(rx + i);
  }
}

/****************************************************************************
 Function
   printPacket

 Description
   Prints the array specified
****************************************************************************/
void printPacket(uint8_t chkArray[], uint8_t size) {
    for(int i=0;i<size;i++) {
        printf("Packet%d: 0x%X\r\n",i, chkArray[i]);
    }
    printf("\r\n");
}

/****************************************************************************
 Function
   isValidPairRequest

 Description
   Checks if a packet is a valid pair request
****************************************************************************/
static bool isValidPairRequest(void) {
    //printf("Header: %X (Expected %X)\r\n",rxData[RX_HEADER],PAIR_REQ);
    //printf("Event: %X (Expected %X)\r\n",rxData[RX_EVENT_TYPE],MY_ADDRESS);
    if(rxData[RX_HEADER] != PAIR_REQ) return false;
    if(rxData[RX_EVENT_TYPE] != MY_ADDRESS) return false;
    return true;
}

/****************************************************************************
 Function
   packetFromMaster

 Description
   Checks that the packet is from the xbee we are paired with and valid
****************************************************************************/
static bool packetFromMaster(void) {
    if(rxData[API_ID] == 0x89) return false; // From us sending a Tx
    if(MasterAddrMSB != rxData[SOURCE_ADDR_MSB] || MasterAddrLSB != rxData[SOURCE_ADDR_LSB]) return false;
    if(rxData[RX_OPTIONS] == BROADCAST) return false;
    return true;
}

/****************************************************************************
 Function
   LoadUpConfirmation

 Description
   Gets the Tx packet ready for a confirmation message
****************************************************************************/
static void LoadUpResponse(uint8_t header) {
    txSize = 10;
    
    /*Status bytes*/
    txData[0] = 0x7E;                               // Start delimiter
    txData[1] = 0x00;                               // length MSB
    txData[2] = 0x06;                               // length LSB
    txData[3] = 0x01;                               // API ID
    txData[4] = 0x00;                               // Frame ID
    txData[DEST_ADDR_MSB] = MasterAddrMSB;          // set destination address msb
    txData[DEST_ADDR_LSB] = MasterAddrLSB;          // set destination address lsb
    txData[TX_OPTIONS] = NO_OPTIONS;                // set options to not be broadcast
    txData[TX_HEADER] = header;                     // set the header to be pair acknowledge
    txData[PAIR_REQ_CHKSUM] = checkSum(txData,10);  // calculate the checksum
}

/****************************************************************************
 Function
    LoadUpStatus

 Description
    Creates a status packet to send over the xbee (tx)
****************************************************************************/
static void LoadUpData(void) {
    txSize = 37;
    
    /*Read in the new data*/
    importData(Wind);
	importData(GPS);
	importData(Distance);
	importData(Compass);
    
    /*Write data to message*/
    /*Status bytes*/
    txData[0] = 0x7E;                                   // Xbee Start delimiter
    txData[1] = 0x00;                                   // Message length MSB
    txData[2] = (sizeof(txData) - 4);                   // Message length LSB
    txData[3] = 0x01;                                   // API Identifier Frame Type (0x01 = Tx request with 16bit address)
    txData[4] = 0x00;                                   // Frame ID (was 0x52) (0x01 = Request response, 0x00 = No response)
    txData[DEST_ADDR_MSB] = MasterAddrMSB;              // Destination address MSB
    txData[DEST_ADDR_LSB] = MasterAddrLSB;              // Destination address LSB
    txData[TX_OPTIONS] = NO_OPTIONS;                    // Set options to none (try broadcast so we don't need dest addr)
    txData[TX_HEADER] = DATA_PACKET;                    // Set header to indicate a data packet        
    
	/*Data bytes*/
    /*Wind*/
	txData[WIND_SPEED1] = CV7.WindSpeed >> 8;           // High byte of wind speed
	txData[WIND_SPEED2] = CV7.WindSpeed;                // Low byte of wind speed
	txData[WIND_SPEED_UNIT] = CV7.WindUnit;             // Unit of wind speed
	txData[WIND_ANGLE1] = CV7.WindAngle >> 8;           // High byte of wind angle
	txData[WIND_ANGLE2] = CV7.WindAngle;                // Low byte of wind angle
    /*Temp*/
	txData[TEMP1] = CV7.Temp >> 8;                      // High byte of temperature
	txData[TEMP2] = CV7.Temp;                           // Low byte of temperature
	txData[TEMP_UNIT] = CV7.TempUnit;                   // Unit of temperature
    /*GPS*/
	txData[GPS_LAT_DEG] = GPSdata.Latitude[0];          // Latitude deg
	txData[GPS_LAT_MIN1] = GPSdata.Latitude[1];         // Latitude min1
	txData[GPS_LAT_MIN2] = GPSdata.Latitude[2] >> 8;    // Latitude min2
	txData[GPS_LAT_MIN3] = GPSdata.Latitude[2];         // Latitude min3
	txData[LAT_NS] = GPSdata.NSIndicator;               // North or South
	txData[GPS_LONG_DEG1] = GPSdata.Longitude[0] >> 8;  // Longitude deg1
	txData[GPS_LONG_DEG2] = GPSdata.Longitude[0];       // Longitude deg2
	txData[GPS_LONG_MIN1] = GPSdata.Longitude[1];       // Longitude min1
	txData[GPS_LONG_MIN2] = GPSdata.Longitude[2] >> 8;  // Longitude min2
	txData[GPS_LONG_MIN3] = GPSdata.Longitude[2];       // Longitude min3
	txData[LONG_EW] = GPSdata.EWIndicator;              // East or West
    /*Height*/
	txData[DIST1] = lidarData.distanceHIGH;             // High byte of Height
	txData[DIST2] = lidarData.distanceLOW;              // Low byte of Height
    /*Orientation*/
	txData[HEADING1] = compassData.heading >> 8;
	txData[HEADING2] = compassData.heading;
    txData[PITCH1] = compassData.pitch >> 8;
    txData[PITCH2] = compassData.pitch;
    txData[ROLL1] = compassData.roll >> 8;
    txData[ROLL2] = compassData.roll;
    
    /*Checksum*/
    txData[DATA_CHKSUM] = checkSum(txData,37);
}

/****************************************************************************
 Function
   SendXbeeData

 Description
   Sends the current txData out via UART
****************************************************************************/
void SendXbeeData(void) {
    txIndex = 0;
    //printf("Sent 0x%X\r\n",txData[txIndex]); 
    HWREG(UART1_BASE + UART_O_DR) = txData[txIndex++];
}

/****************************************************************************
 Function
   UART1ISR

 Description
   Interrupt response for Xbee UART module
****************************************************************************/
void UART1ISR( void ) {
    // If the interrupt is from Tx (Sending data)
    if(((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_TXRIS) >0)) {
        // Clear the interrupt flag
        HWREG(UART1_BASE + UART_O_ICR) |=UART_ICR_TXIC;
        // Put data to be transfered on UART data line
        if(txIndex % txSize > 0) {
            //printf("Sent 0x%X\r\n",txData[txIndex]); 
            HWREG(UART1_BASE + UART_O_DR) = txData[txIndex++];
        }
    }
    // If the interrupt is from Rx (Receiving new data)
	if((HWREG(UART1_BASE+UART_O_RIS)& UART_RIS_RXRIS) >0) {
		// Clear the interrupt flag
		HWREG(UART1_BASE + UART_O_ICR) |= UART_ICR_RXIC;
		// Post byte received to the Xbee
        uint8_t byte = HWREG(UART1_BASE + UART_O_DR);
		ES_Event ThisEvent = {XbeeByteReceived, HWREG(UART1_BASE + UART_O_DR)};
		PostXbeeComms(ThisEvent);
	}
}

/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/
