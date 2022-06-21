/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

#include "protocol.h"
#include "media.h"

// ENUMS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef enum __tag_protocol_state
{
  IDLE, // Only when Idle a New request can be Received or Sent

  /* Phone Initiating Requests */
  PREQ_RECVD,        // REQ from Phone to Monocle Received.
  PREQ_RES_PEND,     // RES from Monocle to Phone Send is initiated.
  PREQ_RES_SENT,     // RES from Monocle to Phone Sent.

  /* Monocle Initiating Requests */
  MREQ_SENT,         // REQ from Monocle to Phone sent.
  MREQ_RES_PEND,     // RES from Phone to Monocle is initiated.
  MREQ_RES_RECVD     // RES from Phone to Monocle Received.

} enumPtlState;

// STRUCTURES >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef struct __tag_protocol_context
{
  enumPtlState eState;
} structPtlCtx;

// GLOBAL VARIABLES >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
structPtlCtx sPtlCtx;

// FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/* ptl_parseResponse: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    This function is called by the bluetooth packet reception module
          to parse the MajorID and call the appropriate Handler.
Input(1): Major ID
Input(2): Minor ID
Input(3): Protocol Packet
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static void priv_ptl_parseRequest (uint8_t ucMajorId, uint8_t ucMinorId, 
                                   void *pvPkt, uint32_t uiPktSize)
{
  uint8_t ucPCmd = ucMajorId;

  /*
    Requests can only be from INFO, SYSTEM and UPGRADE
    MEDIA requests are generated from Monocle itself
   */
  ucPCmd &= ~(1 << DIR_BIT_POS);  // Clear the DIR bit from the MAJOR ID
  switch (ucPCmd)
  {
  case PCMD_INFO:
    break;
  case PCMD_SYSTEM:
    break;
  case PCMD_UPGRADE:
    break;

  case PCMD_MEDIA:
  default:
    /* ERROR CASE */
    break;
  }
}

/* priv_ptl_parseResponse: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    This function is called by the bluetooth packet reception module
          to parse the MajorID and call the appropriate Handler.
Input(1): Major ID
Input(2): Minor ID
Input(3): Protocol Packet
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static void priv_ptl_parseResponse (uint8_t ucMajorId, uint8_t ucMinorId, 
                                    punionPkt puPkt)
{
  uint8_t ucPCmd = ucMajorId;

  /* 
    Responses are paresed only for MEDIA commands
   */
  ucPCmd &= ~(1 << DIR_BIT_POS);  // Clear the DIR bit from the MAJOR ID
  switch (ucPCmd)
  {
  case PCMD_MEDIA:
    mda_actOnMediaResponse (ucMinorId, (pstructMediaPkt) puPkt);
    break; 

  case PCMD_INFO:
  case PCMD_SYSTEM:
  case PCMD_UPGRADE:
  default:
    /* Error Case */
    break;
  }
}

/* priv_ptl_sendRequest: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    This function invokes sending Request Protocol Packet to the Phone 
          APP
Input(1): Major ID
Input(2): Minor ID
Input(3): Protocol Packet
Input(4): Protocol Packet Size
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static void priv_ptl_sendRequest (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt, 
                                  uint32_t uiPktSize)
{
  ucMajorId |= (1 << DIR_BIT_POS);

  // :TODO Call the Bluetooth Stack to Transfer
}

/* priv_ptl_sendResponse: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    This function invokes sending Response Protocol Packet to the Phone 
          APP
Input(1): Major ID
Input(2): Minor ID
Input(3): Protocol Packet
Input(4): Protocol Packet Size
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
static void priv_ptl_sendResponse (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt, 
                                   uint32_t uiPktSize)
{
  ucMajorId |= (1 << DIR_BIT_POS);

  // :TODO Call the Bluetooth Stack to Transfer
}

/* ptl_sendPkt: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    
Input(1): Major ID
Input(2): Minor ID
Input(3): Packet
Input(4): Packet Size
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void ptl_sendPkt (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt,
                  uint32_t uiPktSize)
{
  switch (sPtlCtx.eState)
  {

    case IDLE:
      /* This is a new Request initiated by Monocle
       */
      priv_ptl_sendRequest (ucMajorId, ucMinorId, pvPkt, uiPktSize);
      break;

    case PREQ_RECVD:
      /* This is Phone Initiating Request case
         Send Pkt was called to send the response back to the Phone.
      */
     priv_ptl_sendResponse (ucMajorId, ucMinorId, pvPkt, uiPktSize);
     break;


    case PREQ_RES_PEND:
    case PREQ_RES_SENT:
    case MREQ_SENT:
    case MREQ_RES_RECVD:
    case MREQ_RES_PEND:
    default:
      // Error Case, Probably discard the Packet?
      break;
  }
}

/* ptl_parsePkt: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    
Input(1): Major ID
Input(2): Minor ID
Input(3): Protocol Packet
Input(4): Protocol Packet Size
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void ptl_parsePkt (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt, 
                   uint32_t uiPktSize)
{
  switch (sPtlCtx.eState)
  {
    case IDLE:
      /* This is a new Request from the Phone App */
      priv_ptl_parseRequest (ucMajorId, ucMinorId, pvPkt, uiPktSize);
      break;
    
    case MREQ_RES_PEND:
      /* Response from Phone Received */
      priv_ptl_parseResponse (ucMajorId, ucMinorId, pvPkt);
      break;
  
    case PREQ_RECVD:
    case PREQ_RES_PEND:
    case PREQ_RES_SENT:
    case MREQ_SENT:
    case MREQ_RES_RECVD:
    default:
      // Error Case: Pobably discard the packet.
      break;
  }
}

void ptl_init (void)
{
  sPtlCtx.eState = IDLE;
}