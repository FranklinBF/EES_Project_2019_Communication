"""
Python script to communicate with NXT over Bluetooth using
Python sockets (with Python 3.3 or above).
"""

from communication.communication import Communication
import traceback
import socket

# Main
def start_connection_with_nxt():
    try:
        # New communication instance
        com = Communication()

        print('Trying to connect to NXT...', end='', flush=True)
        com.connect_with_nxt()
        
        print('Connected! Waiting for NXT to sync...')
        com.wait_for_sync()
        
        print('Waiting done!')
        #print('Hit the arrow keys to control the robot, P for pause and ESC to stop...')
        #com.control_nxt_with_keyboard()

        return com
        
    except (KeyboardInterrupt, SystemExit):
        pass
    except (socket.timeout, TimeoutError) as err:
        print('Socket timeout: ', err)
    except Exception as err:
        print(err)
        print(traceback.format_exc())
