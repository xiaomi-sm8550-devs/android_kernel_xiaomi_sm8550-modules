#ifndef _CAM_BU24532_H_
#define _CAM_BU24532_H_

#define EEPROM_ADDR 0x50

#define DL_BYTES 4
#define DL_6200_SIZE 64
#define DL_2800_SIZE 64
#define DL_3000_SIZE 64
#define DL_BF00_SIZE 64

#define DL_DELAY_MS 10
#define AUTO_BOOT_RETRY_TIME 3
#define FIRMWARE_UPDATE_RETRY_TIMES 3

#define FIRMWARE_READ_EEPROM_DISABLE 0
#define FIRMWARE_READ_EEPROM_DEBUG 1

#define CALI_ADDR 0x1DC0
#define CALI_SIZE_BYTE 40

#define AUTO_BOOT_RETRY_TIME 3

int bu24532_ois_pkt_download(struct cam_ois_ctrl_t *o_ctrl);

#endif
