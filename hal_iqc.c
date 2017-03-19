#include "hal_iqc.h"

/***************************************************************************************************
 *                                             CONSTANTS
 ***************************************************************************************************/
#define ISQC_SCL P1_2
#define ISQC_SDA P1_3
#define ISQC_FORLOOP_PERCYCLE 40
#define ISQC_FORLOOP_HALFCYCLE ISQC_FORLOOP_PERCYCLE/2
#define ISQC_FORLOOP_QUARTCYCLE ISQC_FORLOOP_PERCYCLE/4

/***************************************************************************************************
 *                                              MACROS
 ***************************************************************************************************/
#define ISQC_SET_INPUT_PIN() do {P1DIR = 0x00;} while (0)  //p1_2 + p1_3
#define ISQC_SET_OUTPUT_PIN() do {P1DIR = 0x0C;} while (0)  //p1_2 + p1_3
#define ISQC_SET_SCL_OUT() do {P1DIR |= 0x04;} while (0)  //p1_2
#define ISQC_SET_SDA_OUT() do {P1DIR |= 0x08;} while (0)  //p1_3
#define ISQC_SET_SCL_IN() do {P1DIR &= ~(0x04);} while (0)  //p1_2
#define ISQC_SET_SDA_IN() do {P1DIR &= ~(0x08);} while (0)  //p1_3
#define ISQC_BIT0(x) (x & 0x1)
#define ISQC_BIT1(x) ((x>>1) & 0x1)
#define ISQC_BIT2(x) ((x>>2) & 0x1)
#define ISQC_BIT3(x) ((x>>3) & 0x1)
#define ISQC_BIT4(x) ((x>>4) & 0x1)
#define ISQC_BIT5(x) ((x>>5) & 0x1)
#define ISQC_BIT6(x) ((x>>6) & 0x1)
#define ISQC_BIT7(x) ((x>>7) & 0x1)
/*
 * one for loop takes about 24 cycles
 * when system clock is 32MHZ
 * one for loop takes about 0.715255us
 */
#define ISQC_DELAY(x) do {for(volatile int i = 0; i < x; ++i) asm ("NOP");} while (0)

/*
 * a complete cycle to write a bit to device
 * start from the time allowed to write value
 * end with the time allowed to write value, either
 */
#define ISQC_WRITE_BIT(n, x) ISQC_SCL = ISQC_LOW_LEVEL; \
                             ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE); \
                             ISQC_SDA = ISQC_BIT##n(x); \
                             ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE); \
                             ISQC_SCL = ISQC_HIGH_LEVEL; \
                             ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
/*
 * write a byte to device
 * start from msb
 */                               
#define ISQC_WRITE_BYTE(x) ISQC_SET_OUTPUT_PIN(); \
                           ISQC_WRITE_BIT(7, x); \
                           ISQC_WRITE_BIT(6, x); \
                           ISQC_WRITE_BIT(5, x); \
                           ISQC_WRITE_BIT(4, x); \
                           ISQC_WRITE_BIT(3, x); \
                           ISQC_WRITE_BIT(2, x); \
                           ISQC_WRITE_BIT(1, x); \
                           ISQC_WRITE_BIT(0, x);

/*
 * a complete cycle to read a bit from device
 * start from the time allowed to read value
 * end with the time allowed to read value, either
 * SCL and SDA must be input pin first
 */
#define ISQC_READ_BIT(n, x) ISQC_SCL = ISQC_LOW_LEVEL; \
                            ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
                            ISQC_SCL = ISQC_HIGH_LEVEL; \
                            ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE); \
                            if (ISQC_SDA) *(x) |= (0x01 << n); \
                            else *(x) &= ~(0x01 << n); \
                            ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE);

/*
 * read a byte to device
 * start from msb
 */                               
#define ISQC_READ_BYTE(x) ISQC_SET_SDA_IN(); \
                          ISQC_READ_BIT(7, x); \
                          ISQC_READ_BIT(6, x); \
                          ISQC_READ_BIT(5, x); \
                          ISQC_READ_BIT(4, x); \
                          ISQC_READ_BIT(3, x); \
                          ISQC_READ_BIT(2, x); \
                          ISQC_READ_BIT(1, x); \
                          ISQC_READ_BIT(0, x);

/*
 * send a nack signal to device to end communication
 */  
#define ISQC_SEND_NACK() ISQC_SET_OUTPUT_PIN(); \
                         ISQC_SCL = ISQC_LOW_LEVEL; \
                         ISQC_SDA = ISQC_HIGH_LEVEL; \
                         ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
                         ISQC_SCL = ISQC_HIGH_LEVEL; \
                         ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE);

/*
 * wait a ack signal from device, or return a nack signal
 */
#define ISQC_WAIT_ACK() ISQC_SCL = ISQC_LOW_LEVEL; \
                        ISQC_SET_SDA_IN(); \
                        ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
                        ISQC_SCL = ISQC_HIGH_LEVEL; \
                        ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
                        if (ISQC_SDA != ISQC_LOW_LEVEL){ISQC_SEND_NACK(); \
                        ISQC_SET_INPUT_PIN(); return;}
                        
/*
 * send a ack signal to device
 */  
#define ISQC_SEND_ACK() ISQC_SET_OUTPUT_PIN(); \
                        ISQC_SCL = ISQC_LOW_LEVEL; \
                        ISQC_SDA = ISQC_LOW_LEVEL; \
                        ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE); \
                        ISQC_SCL = ISQC_HIGH_LEVEL; \
                        ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE);
                         
/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/
static int8 isqcOpenChnl = 0;
static int8 isqcData[5] = {0};
//static int8 state = 0;
static int8 humInt = 0;
static int8 humFlt = 0;
static int8 tmpInt = 0;
static int8 tmpFlt = 0;
static int8 chkSum = 0;
static halIntState_t intState;

/***************************************************************************************************
 * @fn      HalIsqcInit
 *
 * @brief   Initialize I2C channels
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void HalIsqcInit(void)
{
  ISQC_SET_INPUT_PIN(); //set input first to detect device
  P1SEL = 0x00;  //gpio
  P1INP = 0x00;  //pullup & down
  P1IEN = 0x00;  //disable int
}

/***************************************************************************************************
 * @fn      HalIsqcOpen
 *
 * @brief   Open a I2C channel
 *
 * @param   None
 *
 * @return  I2C id, id >= 0 means open ok, otherwise id < 0 means failure
 ***************************************************************************************************/
int8 HalIsqcOpen(void)
{
  //halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  if (isqcOpenChnl < ISQC_CHANNEL)
  {
    isqcOpenChnl++;
    HAL_EXIT_CRITICAL_SECTION(intState);
    return isqcOpenChnl;
  }
  HAL_EXIT_CRITICAL_SECTION(intState);
  
  return -1;
}

/***************************************************************************************************
 * @fn      HalIsqcReadFrom
 *
 * @brief   Read data from a I2C channel
 *
 * @param   id is the channel number, offset is the byte address
 *
 * @return  data, return 0 while errors
 ***************************************************************************************************/
int8 HalIsqcReadFrom(int8 id, int8 offset)
{
  int8 ret = -1;
  
  HAL_ENTER_CRITICAL_SECTION(intState);
  if (offset >= ISQC_DHT12_HUMID_INT && offset <= ISQC_DHT12_CHKSUM)
    ret = isqcData[offset];
  HAL_EXIT_CRITICAL_SECTION(intState);
  
  return ret;
}

/***************************************************************************************************
 * @fn      HalDHT12ISR
 *
 * @brief   Read temperature and humidity data from dht12
 *
 * @param   None
 *
 * @return  None
 ***************************************************************************************************/
void HalDHT12ISR(int8 id)
{
  uint8 dev = ISQC_DHT12_ADR | ISQC_WRITE_CMD;
  uint8 adr = 0x0;
  ISQC_SET_INPUT_PIN();

  if (ISQC_SCL == ISQC_LOW_LEVEL || ISQC_SDA == ISQC_LOW_LEVEL) //device is working
    return;
  
  /*
   * start flag
   */
  ISQC_SET_SDA_OUT();
  ISQC_SDA = ISQC_LOW_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE);
  
  /*
   * send slave address to write
   */
  ISQC_WRITE_BYTE(dev);
  
  /*
   * wait ack
   */
  ISQC_WAIT_ACK();
  
  /*
   * send register address to read
   */
  ISQC_WRITE_BYTE(adr);
  
  /*
   * wait ack
   */
  ISQC_WAIT_ACK();
  
  /*
   * read device
   */
  dev = ISQC_DHT12_ADR | ISQC_READ_CMD;  
  
  /*
   * start again
   */
  ISQC_SCL = ISQC_LOW_LEVEL;
  ISQC_SET_SDA_OUT();
  ISQC_SDA = ISQC_HIGH_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE);
  ISQC_SCL = ISQC_HIGH_LEVEL;   //avoid dead code elimination optimization
  ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE);
  ISQC_SDA = ISQC_LOW_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE);
  
  /*
   * send slave address to read
   */
  ISQC_WRITE_BYTE(dev);
  
  /*
   * wait ack
   */
  ISQC_WAIT_ACK();
  
  /*
   * recieve humidity integer value
   */
  ISQC_READ_BYTE(&humInt);
  
  /*
   * send ack
   */
  ISQC_SEND_ACK();
  
  /*
   * recieve humidity float value
   */
  ISQC_READ_BYTE(&humFlt);
  
  /*
   * send ack
   */
  ISQC_SEND_ACK();
  
  /*
   * recieve temperature integer value
   */
  ISQC_READ_BYTE(&tmpInt);
  
  /*
   * send ack
   */
  ISQC_SEND_ACK();
  
  /*
   * recieve temperature float value
   */
  ISQC_READ_BYTE(&tmpFlt);
  
  /*
   * send ack
   */
  ISQC_SEND_ACK();
  
  /*
   * recieve checksum value
   */
  ISQC_READ_BYTE(&chkSum);
  
  /*
   * send nack
   */
  ISQC_SEND_NACK();
  
  /*
   * send stop
   */
  ISQC_SCL = ISQC_LOW_LEVEL;
  ISQC_SDA = ISQC_LOW_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_HALFCYCLE);
  ISQC_SCL = ISQC_HIGH_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE);
  ISQC_SDA = ISQC_HIGH_LEVEL;
  ISQC_DELAY(ISQC_FORLOOP_QUARTCYCLE);
  
  ISQC_SET_INPUT_PIN();
  
  HAL_ENTER_CRITICAL_SECTION(intState);
  if (chkSum = humInt + humFlt + tmpInt + tmpFlt) //data is correct
  {
    isqcData[ISQC_DHT12_HUMID_INT] = humInt;
    isqcData[ISQC_DHT12_HUMID_PNT] = humFlt;
    isqcData[ISQC_DHT12_TEMP_INT] = tmpInt;
    isqcData[ISQC_DHT12_TEMP_PNT] = tmpFlt;
    isqcData[ISQC_DHT12_CHKSUM] = chkSum;
  }
  HAL_EXIT_CRITICAL_SECTION(intState);
}