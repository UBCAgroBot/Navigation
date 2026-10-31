/**
  ******************************************************************************
  * @file           : cmd_link.h
  * @brief          : Command link between the Jetson and this board
  ******************************************************************************
  * ASCII lines over LPUART1 (ST-LINK virtual COM port), 115200 8N1.
  *
  * Jetson -> STM32, sent at ~20 Hz:
  *   $C,<seq>,<speed_mm_s>,<steer_mrad>*<CS>\n
  *
  * STM32 -> Jetson, one per valid command, echoing the values as applied:
  *   $A,<seq>,<speed_mm_s>,<steer_mrad>,<flags>*<CS>\n
  *
  * STM32 -> Jetson, on a rejected line:
  *   $E,<code>*<CS>\n          code: 1 = bad checksum, 2 = bad format,
  *                                   3 = line too long
  *
  * <CS>  two uppercase hex digits: XOR of every byte between '$' and '*'
  * seq   0..65535, wraps
  * speed signed mm/s, forward positive
  * steer signed milliradians, left positive (ROS convention)
  * flags bit 0 = motors enabled (CMD_LINK_DRIVE_MOTORS)
  *       bit 1 = values were clamped
  *
  * The Jetson side lives in the Navigation repo: src/motor_bridge.
  ******************************************************************************
  */
#ifndef CMD_LINK_H
#define CMD_LINK_H

#include <stddef.h>
#include <stdint.h>

#define CMD_LINK_MAX_LINE 48

#define CMD_LINK_NONE          0
#define CMD_LINK_OK            1
#define CMD_LINK_ERR_CHECKSUM -1
#define CMD_LINK_ERR_FORMAT   -2
#define CMD_LINK_ERR_OVERFLOW -3

typedef struct
{
  uint16_t seq;
  int32_t  speed_mm_s;
  int32_t  steer_mrad;
} cmd_link_cmd_t;

typedef struct
{
  char    buf[CMD_LINK_MAX_LINE];
  uint8_t len;
  uint8_t active;
} cmd_link_parser_t;

void cmd_link_parser_init(cmd_link_parser_t *p);

/* Feed one received byte. Returns CMD_LINK_OK and fills *out when a valid
   command line completes, a negative CMD_LINK_ERR_* code when a line is
   rejected, and CMD_LINK_NONE otherwise. */
int cmd_link_feed(cmd_link_parser_t *p, char c, cmd_link_cmd_t *out);

/* Format a reply into buf. Return the length written, or -1 if it didn't fit. */
int cmd_link_format_ack(char *buf, size_t size, const cmd_link_cmd_t *cmd, uint8_t flags);
int cmd_link_format_error(char *buf, size_t size, int code);

#endif /* CMD_LINK_H */
