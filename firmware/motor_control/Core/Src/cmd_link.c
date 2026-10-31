/**
  ******************************************************************************
  * @file           : cmd_link.c
  * @brief          : Parser and formatter for the Jetson command link.
  *                   No HAL dependencies, so it can be unit tested on a PC.
  ******************************************************************************
  */
#include "cmd_link.h"

#include <stdio.h>

#define MAX_FIELD_MAGNITUDE 1000000L

static uint8_t checksum(const char *s, size_t n)
{
  uint8_t cs = 0;
  for (size_t i = 0; i < n; i++)
  {
    cs ^= (uint8_t)s[i];
  }
  return cs;
}

static int hex_value(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

/* Parse a signed decimal integer at *pos, stopping at 'end'. Advances *pos. */
static int parse_int(const char **pos, const char *end, long *value)
{
  const char *s = *pos;
  int negative = 0;
  long v = 0;

  if (s < end && *s == '-')
  {
    negative = 1;
    s++;
  }
  if (s >= end || *s < '0' || *s > '9')
  {
    return 0;
  }
  while (s < end && *s >= '0' && *s <= '9')
  {
    v = v * 10 + (*s - '0');
    if (v > MAX_FIELD_MAGNITUDE)
    {
      return 0;
    }
    s++;
  }

  *value = negative ? -v : v;
  *pos = s;
  return 1;
}

static int expect(const char **pos, const char *end, char c)
{
  if (*pos < end && **pos == c)
  {
    (*pos)++;
    return 1;
  }
  return 0;
}

/* Validate and decode the text between '$' and the newline. */
static int parse_line(const char *line, size_t len, cmd_link_cmd_t *out)
{
  /* Need at least "*XX" after the payload */
  if (len < 4 || line[len - 3] != '*')
  {
    return CMD_LINK_ERR_FORMAT;
  }

  int hi = hex_value(line[len - 2]);
  int lo = hex_value(line[len - 1]);
  if (hi < 0 || lo < 0)
  {
    return CMD_LINK_ERR_FORMAT;
  }

  const char *pos = line;
  const char *end = line + len - 3;

  if (checksum(pos, (size_t)(end - pos)) != (uint8_t)((hi << 4) | lo))
  {
    return CMD_LINK_ERR_CHECKSUM;
  }

  long seq, speed, steer;
  if (!expect(&pos, end, 'C') || !expect(&pos, end, ',') ||
      !parse_int(&pos, end, &seq) || !expect(&pos, end, ',') ||
      !parse_int(&pos, end, &speed) || !expect(&pos, end, ',') ||
      !parse_int(&pos, end, &steer) || pos != end)
  {
    return CMD_LINK_ERR_FORMAT;
  }
  if (seq < 0 || seq > 65535)
  {
    return CMD_LINK_ERR_FORMAT;
  }

  out->seq = (uint16_t)seq;
  out->speed_mm_s = (int32_t)speed;
  out->steer_mrad = (int32_t)steer;
  return CMD_LINK_OK;
}

void cmd_link_parser_init(cmd_link_parser_t *p)
{
  p->len = 0;
  p->active = 0;
}

int cmd_link_feed(cmd_link_parser_t *p, char c, cmd_link_cmd_t *out)
{
  if (c == '$')
  {
    /* Start of a new line; drop any partial one */
    p->len = 0;
    p->active = 1;
    return CMD_LINK_NONE;
  }
  if (!p->active || c == '\r')
  {
    return CMD_LINK_NONE;
  }
  if (c == '\n')
  {
    p->active = 0;
    return parse_line(p->buf, p->len, out);
  }
  if (p->len >= sizeof(p->buf))
  {
    p->active = 0;
    return CMD_LINK_ERR_OVERFLOW;
  }
  p->buf[p->len++] = c;
  return CMD_LINK_NONE;
}

/* Wrap an already formatted "$<payload>" in buf with "*CS\n". */
static int finish_line(char *buf, size_t size, int payload_len)
{
  if (payload_len < 1 || (size_t)payload_len + 4 >= size)
  {
    return -1;
  }
  uint8_t cs = checksum(buf + 1, (size_t)payload_len - 1);
  int n = snprintf(buf + payload_len, size - (size_t)payload_len, "*%02X\n", cs);
  if (n != 4)
  {
    return -1;
  }
  return payload_len + n;
}

int cmd_link_format_ack(char *buf, size_t size, const cmd_link_cmd_t *cmd, uint8_t flags)
{
  int n = snprintf(buf, size, "$A,%u,%ld,%ld,%u",
                   (unsigned)cmd->seq, (long)cmd->speed_mm_s,
                   (long)cmd->steer_mrad, (unsigned)flags);
  if (n < 0 || (size_t)n >= size)
  {
    return -1;
  }
  return finish_line(buf, size, n);
}

int cmd_link_format_error(char *buf, size_t size, int code)
{
  int n = snprintf(buf, size, "$E,%d", code);
  if (n < 0 || (size_t)n >= size)
  {
    return -1;
  }
  return finish_line(buf, size, n);
}
