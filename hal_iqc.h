#ifndef HAL_IQC_H
#define HAL_IQC_H

#include "hal_board.h"

#ifdef __cplusplus
extern "C"
{
#endif
  
#define ISQC_CHANNEL 1
#define ISQC_CYCLE 10000  //ms
#define ISQC_INT_CYCLE 5000 //ms
#define ISQC_CLOCK 10  //us, 100kHZ
#define ISQC_DHT12_TEMP_INT 0x02
#define ISQC_DHT12_TEMP_PNT 0x03
#define ISQC_DHT12_HUMID_INT 0x00
#define ISQC_DHT12_HUMID_PNT 0x01
#define ISQC_DHT12_CHKSUM 0x04
#define ISQC_DHT12_BYTES 5
#define ISQC_DHT12_ADR 0xB8
#define ISQC_WRITE_CMD 0x00
#define ISQC_READ_CMD 0x01
#define ISQC_HIGH_LEVEL 0x01
#define ISQC_LOW_LEVEL 0x00
  
extern void HalIsqcInit(void);
extern int8 HalIsqcOpen(void);
extern int8 HalIsqcReadFrom(int8 id, int8 offset);
extern void HalDHT12ISR(int8 id);

#ifdef __cplusplus
}
#endif

#endif