#include "main.h"
#include "tmc5160.h"

extern SPI_HandleTypeDef hspi1;

/* Register addresses (datasheet ch. 6) */
#define GSTAT         0x01
#define IOIN          0x04
#define GLOBALSCALER  0x0B
#define IHOLD_IRUN    0x10
#define RAMPMODE      0x20
#define XACTUAL       0x21
#define AMAX          0x26
#define VMAX          0x27
#define DMAX          0x28
#define D1            0x2A
#define VSTOP         0x2B
#define XTARGET       0x2D
#define CHOPCONF      0x6C
#define V1            0x25
#define A1            0x24

#define MOVE_TIMEOUT_MS  20000
#define LOG_PERIOD_MS    200

static uint8_t last_status;   /* SPI_STATUS byte from the most recent read */

/* Send one 40-bit write datagram: address | 0x80, then 32-bit data MSB first (datasheet 4.1) */
static void write_reg(uint8_t reg, uint32_t value)
{
    uint8_t tx[5] = {
        reg | 0x80,
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value)
    };

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 5, 10);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

static uint32_t read_reg(uint8_t reg)
{
    //transmit
    uint8_t tx[5] = {
        reg
    };

    //receive
    static uint8_t rx[5];

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 10);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 5, 10);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    last_status = rx[0];

    return ((uint32_t)rx[1] << 24) | ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
}

void tmc5160_init(void)
{
    write_reg(GSTAT, 0x07);                 /* clear reset/error flags */
    write_reg(CHOPCONF, 0x000100C3);        /* SpreadCycle, TOFF=3, 256 microsteps */
    write_reg(GLOBALSCALER, 0xFF);           /* 255/256 of RSENSE max current */
    write_reg(IHOLD_IRUN, 0x00061008);      /* IRUN=16, IHOLD=8, IHOLDDELAY=6 */
    write_reg(AMAX, 500);                   /* acceleration */
    write_reg(VMAX, 100000);                 /* top speed (~0.3 rev/s) */
    write_reg(DMAX, 500);                   /* deceleration */
    write_reg(D1, 500);                     /* must be nonzero in position mode */
    write_reg(VSTOP, 10);                   /* must be nonzero in position mode */
    write_reg(XACTUAL, 0);
    write_reg(XTARGET, 0);
    write_reg(RAMPMODE, 0);                 /* position mode */
    write_reg(V1, 99999);                   /* velocity half of max */
    write_reg(A1, 1000);

    uint32_t ioin = read_reg(IOIN);
    Log("IOIN=0x%08lX (version 0x%02lX, SD_MODE=%lu) GSTAT=0x%02lX\r\n",
        ioin, ioin >> 24, (ioin >> 6) & 1, read_reg(GSTAT));
}

/* Moves to pos and busy waits until pos reached (or timeout), logging progress */
void tmc5160_move_to(int32_t position)
{
    write_reg(XTARGET, (uint32_t)position);
    Log("move_to %ld\r\n", position);

    uint32_t start = HAL_GetTick();
    uint32_t last_log = start;
    int32_t x;

    while ((x = (int32_t)read_reg(XACTUAL)) != position)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_log >= LOG_PERIOD_MS)
        {
            Log("  XACTUAL=%ld status=0x%02X\r\n", x, last_status);
            last_log = now;
        }
        if (now - start >= MOVE_TIMEOUT_MS)
        {
            Log("  TIMEOUT XACTUAL=%ld status=0x%02X GSTAT=0x%02lX\r\n",
                x, last_status, read_reg(GSTAT));
            return;
        }
    }
    Log("  reached %ld (%lu ms)\r\n", x, HAL_GetTick() - start);
}
