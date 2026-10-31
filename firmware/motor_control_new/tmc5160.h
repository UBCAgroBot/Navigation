/*
 * Minimal TMC5160 SPI driver - move test only.
 * Requires CubeMX: hspi1 (Mode 3, 8-bit, MSB first, soft NSS, <= 4 MHz),
 * GPIO outputs CSN and ENN, both initial level High.
 */
#ifndef TMC5160_H
#define TMC5160_H

#include <stdint.h>

void tmc5160_init(void);
void tmc5160_move_to(int32_t position);

#endif /* TMC5160_H */
