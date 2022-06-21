/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

#include "media.h"

// Pre-Processor >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#define SEND_MEDIA_DATA_HEADER(__PKT__)  \
  ptl_sendPkt (PMCD_MEDIA, SCMD_MEDIA_DATA_HDR, (void*)__PKT__, sizeof(unionPkt))

#define SEND_MEDIA_METADATA_HEADER(__PKT__)  \
  ptl_sendPkt (PMCD_MEDIA, SCMD_MEDIA_METADATA_HDR, (void*)__PKT__, sizeof(unionPkt))

#define SEND_MEDIA_DATA(__PKT__, __PKT_SIZE__)  \
  ptl_sendPkt (PMCD_MEDIA, SCMD_MEDIA_DATA, (void*)__PKT__, __PKT_SIZE__)

#define SEND_MEDIA_METADATA(__PKT__, __PKT_SIZE__)  \
  ptl_sendPkt (PMCD_MEDIA, SCMD_MEDIA_METADATA, (void*)__PKT__, __PKT_SIZE__)


// Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

/* mda_actOnMediaResponse: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Desc:	    This function acts on Media response from the Phone APP 
Input(1): Minor ID
Input(2): Protocol Packet
Returns:  None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void mda_actOnMediaResponse (uint8_t ucMinorId, pstructMediaPkt psMediaPkt)
{
  //:TODO
  switch (ucMinorId)
  {
  case SCMD_MEDIA_DATA:
    break;
  case SCMD_MEDIA_DATA_HDR:
    break;
  case SCMD_MEDIA_METADATA:
    break;
  case SCMD_MEDIA_METADATA_HDR:
    break; 
  default:
    break;
  }
}
