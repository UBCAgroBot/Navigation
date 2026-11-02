import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from motor_bridge.protocol import Ack, Error, encode_command, parse_reply

try:
    import serial
except ImportError:  # only needed when not in dry_run
    serial = None


class MotorBridge(Node):
    """
    Forward drive commands to the STM32 over serial and check its acks.

    Inputs (command_source parameter):
      twist_steer  /cmd_vel (Twist, linear.x = speed) + /steer_fl, /steer_fr
                   (Float64, radians), as published by gap_follower_node
      ackermann    /drive (AckermannDriveStamped)
    """

    def __init__(self):
        super().__init__('motor_bridge')

        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.ack_timeout = self.declare_parameter('ack_timeout', 1.0).value
        source = self.declare_parameter('command_source', 'twist_steer').value
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.speed = 0.0
        self.steer = {}
        self.last_cmd_time = None
        self.seq = 0
        self.serial = None
        self.rx_buf = b''
        self.last_ack_time = None
        self.opened_time = 0.0
        self.next_open_attempt = 0.0

        if source == 'twist_steer':
            self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
            self.create_subscription(
                Float64, '/steer_fl', lambda m: self.steer_callback('fl', m), 10)
            self.create_subscription(
                Float64, '/steer_fr', lambda m: self.steer_callback('fr', m), 10)
        elif source == 'ackermann':
            self.create_subscription(
                AckermannDriveStamped, '/drive', self.ackermann_callback, 10)
        else:
            raise ValueError(
                f"command_source must be 'twist_steer' or 'ackermann', not '{source}'")

        if not self.dry_run and serial is None:
            raise RuntimeError('pyserial is not installed (apt install python3-serial)')

        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f"Motor bridge started: {source} -> {'dry run' if self.dry_run else self.port}")

    def twist_callback(self, msg: Twist):
        self.speed = msg.linear.x
        self.last_cmd_time = time.monotonic()

    def steer_callback(self, wheel: str, msg: Float64):
        self.steer[wheel] = msg.data

    def ackermann_callback(self, msg: AckermannDriveStamped):
        self.speed = msg.drive.speed
        self.steer = {'front': msg.drive.steering_angle}
        self.last_cmd_time = time.monotonic()

    def current_command(self):
        """Speed and steering to send; speed drops to 0 if the inputs go stale."""
        now = time.monotonic()
        fresh = self.last_cmd_time is not None and now - self.last_cmd_time < self.cmd_timeout
        steer = sum(self.steer.values()) / len(self.steer) if self.steer else 0.0
        return (self.speed if fresh else 0.0), steer

    def tick(self):
        speed, steer = self.current_command()
        line = encode_command(self.seq, speed, steer)
        self.seq = (self.seq + 1) & 0xFFFF

        if self.dry_run:
            self.get_logger().info(
                f'dry run: {line.decode().strip()}', throttle_duration_sec=1.0)
            return

        if not self.ensure_open():
            return
        try:
            self.serial.write(line)
            self.read_replies()
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(f'Serial error on {self.port}: {e}')
            self.close()
            return

        last_heard = self.last_ack_time or self.opened_time
        if time.monotonic() - last_heard > self.ack_timeout:
            self.get_logger().warn(
                f'No ack from STM32 on {self.port} for over {self.ack_timeout:.1f}s',
                throttle_duration_sec=2.0)

    def read_replies(self):
        self.rx_buf += self.serial.read(self.serial.in_waiting or 0)
        *lines, self.rx_buf = self.rx_buf.split(b'\n')
        for line in lines:
            reply = parse_reply(line)
            if isinstance(reply, Ack):
                if self.last_ack_time is None:
                    self.get_logger().info('STM32 link up')
                self.last_ack_time = time.monotonic()
                self.get_logger().info(
                    f'ack seq={reply.seq} speed={reply.speed_mm_s}mm/s '
                    f'steer={reply.steer_mrad}mrad flags={reply.flags:#x}',
                    throttle_duration_sec=1.0)
            elif isinstance(reply, Error):
                self.get_logger().warn(
                    f'STM32 rejected a command: {reply.reason}', throttle_duration_sec=1.0)
            elif line.strip():
                self.get_logger().warn(
                    f'Unrecognized line from STM32: {line[:60]!r}', throttle_duration_sec=1.0)

    def ensure_open(self):
        if self.serial is not None:
            return True
        now = time.monotonic()
        if now < self.next_open_attempt:
            return False
        self.next_open_attempt = now + 2.0
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0)
            self.rx_buf = b''
            self.last_ack_time = None
            self.opened_time = now
            self.get_logger().info(f'Opened {self.port} at {self.baud} baud')
            return True
        except (OSError, serial.SerialException) as e:
            self.get_logger().error(
                f'Cannot open {self.port}: {e} (retrying)', throttle_duration_sec=10.0)
            return False

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except (OSError, serial.SerialException):
                pass
        self.serial = None

    def stop(self):
        """Best-effort zero-speed command on shutdown."""
        if self.serial is None:
            return
        try:
            self.serial.write(encode_command(self.seq, 0.0, 0.0))
            self.serial.flush()
        except (OSError, serial.SerialException):
            pass
        self.close()


def main():
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
