from tabnanny import check

import serial
import json

def crc(data_bytes):
    """
    Calculates an 8-bit checksum by summing the byte values and taking modulo 256.

    Args:
        data_bytes (bytes or bytearray): The data sequence to checksum.

    Returns:
        int: The 8-bit checksum (an integer between 0 and 255).
    """
    checksum = 0
    for byte_value in data_bytes:
        checksum = (checksum + byte_value) % 256  # Sum and keep within 8 bits
    return checksum

def parsePacket(message):
    if len(message) >= 3:
        try:
            payload = {
                'head': hex(ord(message[0])),
                'slave_addr': ord(message[1]),
                'function': hex(ord(message[2])),
                'data': [ord(x) for x in message[3:len(message)-1]],
                'checksum': ord(message[len(message)-1])
            }
        except Exception as e:
            print(e)

        print(json.dumps(payload, indent=2))



with serial.Serial('COM11', 38400, timeout=None) as ser:

    # Wait until the start of a conversation
    message = [b'\xfa']
    ser.read_until(b'\xfa', 1)

    while True:
        packet = ser.read()
        if packet == b'\xfa' or packet == b'\xfb':
            parsePacket(message)
            message = []
        message.append(packet)