import sys
import glob
import serial
from typing import Optional

class ToolManager:

    def __init__(self):
        pass





    def serial_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def int_to_bytes(self,x: int) -> bytes:
        return x.to_bytes((x.bit_length() + 7) // 8, 'big')
    
    def int_from_bytes(xbytes: bytes) -> int:
        return int.from_bytes(xbytes, 'big')

    def int_to_bytes(self,number: int) -> bytes:
        return number.to_bytes(length=(8 + (number + (number < 0)).bit_length()) // 8, byteorder='big', signed=True)

    def int_from_bytes(binary_data: bytes) -> Optional[int]:
        return int.from_bytes(binary_data, byteorder='big', signed=True)

    def bitstring_to_bytes(self, s):
        return int(s, 2).to_bytes((len(s) + 7) // 8, byteorder='big')

    def bytes_to_bitstring(self, s: bytes):

        string = s.hex()
    
        return ''.join(format(ord(byte), '08b') for byte in string)