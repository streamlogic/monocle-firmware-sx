/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

//Function Prototypes >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
uint32_t cal_checksum     (uint8_t *buffer, uint32_t size);
uint32_t calnstr_checksum (uint8_t *buffer, uint32_t size);
bool     verify_checksum  (uint8_t *buffer, uint32_t size);