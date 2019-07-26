"""
Python utility script for Bluetooth,
storing MAC address, port and checking
for enabled Bluetooth.
"""
import subprocess


class Bluetooth:

    # Constants
    NXT_MAC_ADDRESS = '00:16:53:11:0C:48'
    PORT = 1

    # Check for enabled BT, raise exception otherwise
    def __init__(self):
        if not self.is_enabled():
            raise ConnectionError('Bluetooth is turned off, please turn it on!')

    # Start hcitool tool and check output for listed local device
    def is_enabled(self):
        return 'hci0' in subprocess.check_output('hcitool dev', shell=True).decode('utf-8')
