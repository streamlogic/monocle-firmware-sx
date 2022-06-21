/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

#ifndef MEDIA_H
#define MEDIA_H

#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"

// Enumerations >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
typedef enum __tag_media_types
{
  PHOTO = 1,
  VIDEO_WITHOUT_AUDIO = 2,
  VIDEO_WITH_AUDIO = 3
} enumMdaType; 

// Function Prototypes >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void mda_actOnMediaResponse (uint8_t ucMinorId, pstructMediaPkt psMediaPkt);


#endif /* EO MEDIA_H */