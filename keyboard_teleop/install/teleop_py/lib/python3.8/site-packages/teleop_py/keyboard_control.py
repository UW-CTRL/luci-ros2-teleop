import rclpy
from rclpy.node import Node
from luci_messages.msg import LuciJoystick
import sys


def read_single_keypress():
    """ https://stackoverflow.com/questions/983354/how-do-i-wait-for-a-pressed-key
    Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns a tuple of characters of the key that was pressed - on Linux, 
    pressing keys like up arrow results in a sequence of characters. Returns 
    ('\x03',) on KeyboardInterrupt which can happen when a signal gets
    handled.

    """
    import termios
    import fcntl
    import os
    import sys
    fd = sys.stdin.fileno()
    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save)  # copy the stored version to update
    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                  | termios.ISTRIP | termios.INLCR | termios. IGNCR
                  | termios.ICRNL | termios.IXON)
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios. PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
    # read a single keystroke
    ret = []
    try:
        ret.append(sys.stdin.read(1))  # returns a single character
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save | os.O_NONBLOCK)
        c = sys.stdin.read(1)  # returns a single character
        while len(c) > 0:
            ret.append(c)
            c = sys.stdin.read(1)
    except KeyboardInterrupt:
        ret.append('\x03')
    finally:
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
    return tuple(ret)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_comands')
        self.publisher_ = self.create_publisher(LuciJoystick, 'joystick_topic', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        keyboard_data = str(read_single_keypress())
        print(keyboard_data)
        if keyboard_data == "('q',)" or keyboard_data == r"('\x03',)":
            sys.exit("\n I quit")
        msg = LuciJoystick()
        # UP
        if keyboard_data == r"('\x1b', '[', 'A')":
            msg.forward_back = 30
            msg.left_right = 0
        # DOWN
        elif keyboard_data == r"('\x1b', '[', 'B')":
            msg.forward_back = -30
            msg.left_right = 0
        # LEFT
        elif keyboard_data == r"('\x1b', '[', 'D')":
            msg.forward_back = 0
            msg.left_right = -30
        # RIGHT
        elif keyboard_data == r"('\x1b', '[', 'C')":
            msg.forward_back = 0
            msg.left_right = 30
        else:
            msg.forward_back = 0
            msg.left_right = 0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: {} {}'.format(msg.forward_back,
                                                          msg.left_right))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
