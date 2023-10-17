# https://stackoverflow.com/a/57387909
import sys
if sys.platform == 'win32':
    import msvcrt
    is_windows = True
else:
    import termios
    import tty
    is_windows = False

import atexit
import threading

SPECIAL_CODES = {
    '\x1b[A': 'up',
    '\x1b[B': 'down',
    '\x1b[C': 'right',
    '\x1b[D': 'left',
}


class KeyboardListener(threading.Thread):
    def __init__(self, callback):
        threading.Thread.__init__(self, name='keyboard_listener')
        self.callback = callback

        if not is_windows:
            self.saved_settings = termios.tcgetattr(sys.stdin)
            atexit.register(self.restore_settings)

            tty.setcbreak(sys.stdin.fileno())
        else:
            self.saved_settings = None

        self.running = True
        self.start()

    def restore_settings(self):
        if self.saved_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
                              self.saved_settings)
        self.running = False

    def run(self):
        buffer = ''
        while self.running:
            k = self.get_key()
            if k == '\x1b' or buffer:
                buffer += k

                if len(buffer) == 3:
                    if buffer in SPECIAL_CODES:
                        self.callback(SPECIAL_CODES[buffer])
                    else:
                        for k in buffer:
                            self.callback(k)
                    buffer = ''
            else:
                self.callback(k)

    def get_key(self):
        if is_windows:
            return msvcrt.getwch()
        else:
            return sys.stdin.read(1)
