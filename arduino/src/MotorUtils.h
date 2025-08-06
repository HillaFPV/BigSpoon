#include <stdint.h>

int64_t getEncoderValue(uint8_t slaveAddr);
int64_t positionMode1Run(uint8_t slaveAddr,uint8_t dir,uint16_t speed,uint8_t acc,uint32_t pulses);
int64_t positionMode2Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t pulses);
int64_t positionMode3Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position);
int64_t positionMode4Run(uint8_t slaveAddr, uint16_t speed, uint8_t acc, int32_t position);
int64_t speedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc);
uint8_t getCheckSum(uint8_t *buffer, uint8_t size);
int64_t waitingForACK();

void blinkLight(uint8_t times);

