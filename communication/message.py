"""
Packet wrapper class
"""
import socket
import struct
import binascii
import traceback
import libscrc

class Message:

    # Protocol flags, 1 byte size in header
    HEADER_OBSTACLE = 1 << 0
    HEADER_TURN_AROUND = 1 << 1
    HEADER_INTERSECTION = 1 << 2
    HEADER_LEFT = 1 << 3
    HEADER_RIGHT = 1 << 4
    HEADER_STRAIGHT = 1 << 5
    HEADER_SLOWDOWN = 1 << 6

    def __init__(self, flags, payload):
        # If going straight, check for float data type
        if flags is self.HEADER_STRAIGHT and type(payload) is not float:
            raise TypeError('Payload needs to be float')

        # Use little endian and pack header as unsigned char (1 byte size), payload as float (4 byte)
        self.header = struct.pack('<B', flags)
        self.payload = struct.pack('<f', payload)
        # Buffer as binary string for transferring as sequence of bytes
        self._buffer = b''

    # Create buffer by appending each part of the packet
    def _generateBuffer(self):
        self._buffer = bytearray(self._createHeaderPrefix())
        for b in self.header:
          self._buffer.append(b)
        for b in self.payload:
          self._buffer.append(b)
        self._buffer.append(self._createCRC())
        # Create bytes object as immutable sequences of bytes for binary protocol
        self._buffer = bytes(self._buffer)

    # Header to signalize beginning of package
    def _createHeaderPrefix(self):
        return bytes.fromhex('FF FF')

    # Calculate CRC8 checksum to ensure data integrity (1 byte)
    def _createCRC(self):
        crc = bytearray([self._buffer[0]])
        for b in range(1, len(self._buffer)):
            crc.append(self._buffer[b])
        return libscrc.crc8(bytes(crc))

    # Set bit to 1 at offset
    @staticmethod
    def setBitByOffset(offset):
        mask = 1 << offset
        return(0 | mask)

    # Generate buffer and send it
    def send(self, socket):
        self._generateBuffer()
        if self._buffer:
            print('Sending', repr(self._buffer), 'with', len(self._buffer), 'bytes')
            try:
                sent = socket.send(self._buffer)
                self._buffer = self._buffer[sent:]
            except BlockingIOError:
                # Resource temporarily unavailable (errno EWOULDBLOCK)
                pass
