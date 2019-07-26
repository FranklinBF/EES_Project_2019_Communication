"""
Communicate with NXT over Bluetooth using Python sockets (with Python 3.3 or above).
Handle outgoing single values and incoming values (e.g. for debug possiblities),
as well as controlling the NXT via keyboard actions (acting as remote control via shell).

NXT uses BT 2.0 and application protocol SPP (Serial-Port Profile),
which is based on top of a low-level protocol called RFCOMM (reliable transport protocol).
"""
from pynput import keyboard
from threading import Thread
from .bluetooth import Bluetooth
import logging
import throttle
import readline
import socket
import time
import struct


class Communication(Bluetooth):

    # The key combinations to check
    COMBINATION_UP_RIGHT = {keyboard.Key.up, keyboard.Key.right}
    COMBINATION_UP_LEFT = {keyboard.Key.up, keyboard.Key.left}
    COMBINATION_DOWN_LEFT = {keyboard.Key.down, keyboard.Key.left}
    COMBINATION_DOWN_RIGHT = {keyboard.Key.down, keyboard.Key.right}

    # Debug flag
    DEBUG = True

    def __init__(self):
        # Init bluetooth parent to check for enabled BT
        super().__init__()

        # Logging config
        logging.basicConfig(filename='debug_receive.log', level=logging.DEBUG)

        # The currently active keys
        self.current_keys = set()

        # Socket
        self.s = None

        # Received value
        self.recv_value = None

        # Running
        self.running = True

    # Establish socket connection
    def connect_with_nxt(self):
    	# Use the BT family, SOCK_STREAM as TCP type for reliable communication
    	# and the RFCOMM transport protocol which provides a simple reliable data stream (similar to TCP)
        self.s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        # Try connecting without raising exceptions
        while self.s.connect_ex((self.NXT_MAC_ADDRESS, self.PORT)) != 0:
            print('.', end='', flush=True)
            time.sleep(2)
        print('\n', end='', flush=True)
        
        return self.s

    # Extra thread handles incoming messages
    def socket_receive_message(self):
        print('Start receiving too...')
        while self.running:
            data = self.s.recv(1024)
            if len(data) > 0:
            	self.recv_value = data
            logging.debug(data)
            #print(data)

    # Return latest incoming message value
    def get_recv_value(self):
    	return self.recv_value

    # Try to sync after connecting with NXT and start second thread for incoming messages
    def wait_for_sync(self):
        while True:
            if self.s.recv(32):
                if self.DEBUG:
                    Thread(target=self.socket_receive_message).start()
                break

    # Origin of sending single float values to NXT
    def socket_send_value(self, v):
        packed_value = struct.pack('<f', v)
        hex_value = ''.join('\\0x{0:02x}'.format(v) for v in packed_value)
        bytes_sent = self.s.send(packed_value)
        print('Sent "{}" packed as "{}" (hex) with {} bytes'.format(v, hex_value, bytes_sent))
        
    # Send periodically current time (for testing)
    def send_periodically_time_value(self, period=3, sleeping=5):
        while True:
            value = time.time() % period
            self.socket_send_value(value)
            time.sleep(sleeping)
        self.s.close()

    """
    Values 1 to 8 represent directions clockwise,
    for example represents 1 forward direction,
    2 top-right direction, 3 right direction etc.
    While 0 represents to stop.
    Throttle calls at 10/s.
    """
    @throttle.throttle(seconds=0.1)
    def keyboard_key_pressed(self, key):
        self.current_keys.add(key)

        if all(k in self.current_keys for k in self.COMBINATION_UP_RIGHT):
            #print('Top right active!')
            self.socket_send_value(2)
        elif all(k in self.current_keys for k in self.COMBINATION_DOWN_RIGHT):
            #print('Down right active!')
            self.socket_send_value(4)
        elif all(k in self.current_keys for k in self.COMBINATION_DOWN_LEFT):
            #print('Down left active!')
            self.socket_send_value(6)
        elif all(k in self.current_keys for k in self.COMBINATION_UP_LEFT):
            #print('Top left active!')
            self.socket_send_value(8)
        elif key == keyboard.Key.up:
            #print('Up active!')
            self.socket_send_value(1)
        elif key == keyboard.Key.right:
            #print('Right active!')
            self.socket_send_value(3)
        elif key == keyboard.Key.down:
            #print('Down active!')
            self.socket_send_value(5)
        elif key == keyboard.Key.left:
            #print('Left active!')
            self.socket_send_value(7)

    # Handle keyboard navigation discards for diagonal movement handling
    # and check for ESC (exit) or P (pause) key released events
    def keyboard_key_released(self, key):
        self.current_keys.discard(key)
        isESCReleased = key == keyboard.Key.esc
        #print('{} released'.format(key))

        # Stop listener
        if key == keyboard.KeyCode(char = 'p') or isESCReleased:
            self.socket_send_value(0)
            if isESCReleased:
                self.running = False
                return False

    # Setup keyboard control
    def control_nxt_with_keyboard(self):
        # Collect events until released
        with keyboard.Listener(on_release=self.keyboard_key_released,on_press=self.keyboard_key_pressed,suppress=True) as listener:
            listener.join()
