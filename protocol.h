/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define PROTOCOL_VERSION  1u

#define PROTOCOL_PACKET_SIZE  48u /* Bytes */

/**
 * @brief CMD
 */
typedef enum __tag_command
{
  PCMD_INFO = 0x00u,
  /* INFO SECONDARY CMD */
  SCMD_INFO_ALL        = 0x00u,
  SCMD_INFO_PTCL_VER   = 0x01u, /* PROTOCOL VERSION */
  SCMD_INFO_FMWR_STATE = 0x02u, /* FIRMWARE VERSION */
  SCMD_INFO_FPGA_VER   = 0x03u, /* FPGA VERSION */
  SCMD_INFO_MCU_VER    = 0x04u, 
  SCMD_INFO_HW_VER     = 0x05u, /* HARDWARE VERSION */
  SCMD_INFO_BAT_STATUS = 0x06u, /* BATTERY STATUS */

  PCMD_SYSTEM = 0x01u,
  /* SYSTEM SECONDARY CMD */
  SCMD_SYSTEM_RESET = 0x01u,

  PCMD_UPGRADE = 0x02u,
  /* UPGRADE SECONDARY CMD */
  SCMD_UPGRADE_IMG_HDR  = 0x01u,  /* IMAGE HEADER */
  SCMD_UPGRADE_SCT_HDR  = 0x02u,  /* SECTOR HEADER */
  SCMD_UPGRADE_SCT_DATA = 0x03u,  /* SECTOR DATA */

  PCMD_MEDIA = 0x03u,
  /* MEDIA SECONDARY CMD */
  SCMD_MEDIA_DATA_HDR     = 0x01u,  /* DATA HEADER */
  SCMD_MEDIA_METADATA_HDR = 0x02u,  /* METADATA HEADER */
  SCMD_MEDIA_DATA         = 0x03u,   /* DATA */
  SCMD_MEDIA_METADATA     = 0x04u  /* METADATA */

} enumCMD;

/**
 * @brief Direction of Packet
 */
typedef enum __tag_direction
{
  APP_TO_MONOCLE = 0x0u,
  MONOCLE_TO_APP = 0x80u
} enumDIR;

#define DIR_BIT_POS 7u

/**
 * @brief Status Code
 */
typedef enum __tag_status_code
{
  ALL_OK                   = 0x00,
  FPGA_VERSION_MISMATCH    = 0x01, /* Checked in the begining of the bootup to check if 
                                      the FPGA version is same as indicated in the firmware */
  SECTOR_CHECKSUM_MISMATCH = 0x02, /* Check of the SECTOR doesnt match, in this case the Phone APP
                                      can retry sending the same sector again. */
  IMAGE_CHECKSUM_MISMATCH  = 0x03  /* Mismatch in the checksum of the whole image what was 
                                      transferred to Monocle  */
} enumSTATUS;

/**
 * @brief Info Packet Format
 */
typedef struct __tag_info_packet
{
  uint32_t uiProtocolVer;
  uint32_t uiFirmwareState;
  uint32_t uiFPGAVersion;
  uint32_t MCUVersion;  /* Probably not needed */
  uint32_t HWVersion;
  uint32_t BatteryStatus;

} structInfoPkt, *pstructInfoPkt;

/**
 * @brief System Packet Format
 */
typedef struct __tag_system_packet
{
  uint32_t uiDummy[PROTOCOL_PACKET_SIZE / 4];

} structSysPkt, *pstructSysPkt;

/**
 * @brief Upgrade Packet Format
 */
typedef struct __tag_upgrade_packet
{
  uint32_t uiStatusCode;
  uint32_t uiImageType;
  uint32_t uiImageSize;
  uint32_t uiImageChecksum;
  uint32_t uiSectorSize;
  uint32_t uiSectorChecksum;
  uint32_t uiRemainderSize;
  uint32_t uiFlashSectorAddr;

} structUpgradePkt, *pstructUpgradePkt;

/**
 * @brief Media Packet Format
 */
typedef struct __tag_media_packet
{
  uint32_t uiStatusCode;
  uint32_t uiMediaType;
  uint32_t uiMediaLength;
  uint32_t uiUUID;
  uint32_t uiNumOfFrames;
  uint32_t uiFrameSize;
  uint32_t uiMetaDataLength;
  uint32_t uiRemainderSize;

} structMediaPkt, *pstructMediaPkt;

/**
 * @brief Info Packet Format
 */
typedef union __tag_packet
{
  uint8_t ucOctet[PROTOCOL_PACKET_SIZE];
  structInfoPkt    sInfo;
  structSysPkt     sSys;
  structUpgradePkt sUpgrade;
  structMediaPkt   sMedia;
  
} unionPkt, *punionPkt;

// Function Prototypes >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void ptl_sendPkt (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt,
                  uint32_t uiPktSize);
void ptl_parsePkt (uint8_t ucMajorId, uint8_t ucMinorId, void *pvPkt, 
                   uint32_t uiPktSize);

#endif /* End of PROTOCOL_H */