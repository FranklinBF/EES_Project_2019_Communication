#!/usr/bin/env python3
"""
Python client wrapper to communicate with NXT over Bluetooth.
"""
from communication import Communication
from message import Message
import traceback
import socket


if __name__ == '__main__':
    try:
        # New communication instance
        com = Communication()

        print('Trying to connect to NXT...', end='', flush=True)
        com.connect_with_nxt()
        
        print('Connected! Waiting for NXT to sync...')
        com.wait_for_sync()

        print('Sent message...')
        msg = Message(flags=Message.HEADER_LEFT, payload=0.1458)
        msg.send(com.s)
        
        #print('Waiting done!')
        #com.send_periodically_time_value()
        
        #print('Hit the arrow keys to control the robot, P for pause and ESC to stop...')
        #com.control_nxt_with_keyboard()
        
    # Ignore keyboard interrupt and system exit
    except (KeyboardInterrupt, SystemExit):
        pass
    # Print timeouts
    except (socket.timeout, TimeoutError) as err:
        print('Socket timeout: ', err)
    # Print exceptions
    except Exception as err:
        print(err)
        print(traceback.format_exc())
