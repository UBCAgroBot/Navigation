import unittest

from motor_bridge.protocol import Ack, Error, encode_command, parse_reply


class TestProtocol(unittest.TestCase):

    def test_encode_command(self):
        self.assertEqual(encode_command(7, 1.0, -0.25), b'$C,7,1000,-250*43\n')

    def test_encode_rounds_and_wraps_seq(self):
        self.assertEqual(encode_command(65537, 0.0004, 0.0006), b'$C,1,0,1*5F\n')

    def test_parse_ack(self):
        self.assertEqual(parse_reply(b'$A,7,1000,-250,1*5C\n'), Ack(7, 1000, -250, 1))

    def test_parse_error(self):
        reply = parse_reply(b'$E,1*58\r\n')
        self.assertEqual(reply, Error(1))
        self.assertEqual(reply.reason, 'bad checksum')

    def test_reject_bad_checksum(self):
        self.assertIsNone(parse_reply(b'$A,7,1000,-250,1*5D\n'))

    def test_reject_garbage(self):
        for line in [b'', b'\n', b'hello\n', b'$A,1,2*00\n', b'$A,x,0,0,0*00\n', b'\xff\xfe\n']:
            self.assertIsNone(parse_reply(line), line)


if __name__ == '__main__':
    unittest.main()
