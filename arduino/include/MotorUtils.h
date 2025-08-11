#include <stdint.h>

long getEncoderValue(uint8_t slaveAddr);
long positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
long positionMode2Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pulses);
long positionMode3Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position);
long positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position);
long speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc);
uint8_t getCheckSum(uint8_t *buffer, uint8_t size);
long waitingForACK();

void blinkLight(uint8_t times);
