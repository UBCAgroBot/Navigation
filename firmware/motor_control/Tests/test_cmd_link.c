/*
 * Host-side tests for cmd_link.c. Not part of the firmware build.
 *
 *   gcc -Wall -Wextra -I Core/Inc Core/Src/cmd_link.c Tests/test_cmd_link.c -o /tmp/test_cmd_link
 *   /tmp/test_cmd_link
 *
 * With --echo, reads command lines on stdin and prints the replies the board
 * would send, for testing against the Jetson side (src/motor_bridge).
 */
#include "cmd_link.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int feed_str(cmd_link_parser_t *p, const char *s, cmd_link_cmd_t *out)
{
  int last = CMD_LINK_NONE;
  for (; *s; s++)
  {
    int r = cmd_link_feed(p, *s, out);
    if (r != CMD_LINK_NONE)
    {
      last = r;
    }
  }
  return last;
}

static void test_valid_command(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  cmd_link_parser_init(&p);

  /* XOR of "C,7,1000,-250" is 0x43 */
  assert(feed_str(&p, "$C,7,1000,-250*43\n", &cmd) == CMD_LINK_OK);
  assert(cmd.seq == 7);
  assert(cmd.speed_mm_s == 1000);
  assert(cmd.steer_mrad == -250);
}

static void test_noise_and_crlf(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  cmd_link_parser_init(&p);

  /* Garbage before '$' and a partial line are dropped; CR is ignored */
  assert(feed_str(&p, "xx$C,1,2$C,7,1000,-250*43\r\n", &cmd) == CMD_LINK_OK);
  assert(cmd.seq == 7);
}

static void test_bad_checksum(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  cmd_link_parser_init(&p);
  assert(feed_str(&p, "$C,7,1000,-250*44\n", &cmd) == CMD_LINK_ERR_CHECKSUM);
}

static void test_bad_format(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  cmd_link_parser_init(&p);

  /* Valid checksums, invalid contents */
  assert(feed_str(&p, "$C,7,1000*75\n", &cmd) == CMD_LINK_ERR_FORMAT);       /* missing field */
  assert(feed_str(&p, "$C,70000,0,0*58\n", &cmd) == CMD_LINK_ERR_FORMAT);    /* seq > 65535 */
  assert(feed_str(&p, "$X,1,0,0*45\n", &cmd) == CMD_LINK_ERR_FORMAT);        /* wrong type */
  assert(feed_str(&p, "$C,1,0,0\n", &cmd) == CMD_LINK_ERR_FORMAT);           /* no checksum */
}

static void test_overflow(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  cmd_link_parser_init(&p);

  char line[CMD_LINK_MAX_LINE + 8];
  line[0] = '$';
  memset(line + 1, '1', sizeof(line) - 2);
  line[sizeof(line) - 1] = '\0';
  assert(feed_str(&p, line, &cmd) == CMD_LINK_ERR_OVERFLOW);

  /* Parser recovers on the next line */
  assert(feed_str(&p, "\n$C,7,1000,-250*43\n", &cmd) == CMD_LINK_OK);
}

static void test_format_replies(void)
{
  char buf[64];
  cmd_link_cmd_t cmd = {7, 1000, -250};

  int n = cmd_link_format_ack(buf, sizeof(buf), &cmd, 1);
  assert(n == (int)strlen(buf));
  assert(strcmp(buf, "$A,7,1000,-250,1*5C\n") == 0);

  cmd_link_format_error(buf, sizeof(buf), 1);
  assert(strcmp(buf, "$E,1*58\n") == 0);

  /* Too small a buffer is reported, not overrun */
  assert(cmd_link_format_ack(buf, 8, &cmd, 0) == -1);
}

static int echo_mode(void)
{
  cmd_link_parser_t p;
  cmd_link_cmd_t cmd;
  char reply[64];
  int c;

  cmd_link_parser_init(&p);
  while ((c = getchar()) != EOF)
  {
    int r = cmd_link_feed(&p, (char)c, &cmd);
    if (r == CMD_LINK_OK)
    {
      cmd_link_format_ack(reply, sizeof(reply), &cmd, 0);
      fputs(reply, stdout);
      fflush(stdout);
    }
    else if (r < 0)
    {
      cmd_link_format_error(reply, sizeof(reply), -r);
      fputs(reply, stdout);
      fflush(stdout);
    }
  }
  return 0;
}

int main(int argc, char **argv)
{
  if (argc > 1 && strcmp(argv[1], "--echo") == 0)
  {
    return echo_mode();
  }

  test_valid_command();
  test_noise_and_crlf();
  test_bad_checksum();
  test_bad_format();
  test_overflow();
  test_format_replies();
  puts("cmd_link: all tests passed");
  return 0;
}
