#!/usr/bin/env python3.8

import struct, math

def fast_binary2float(data):
    float_value = struct.unpack('<f', data)[0]
    return float_value

def binary2float(data):
    binary = bytes(data)
    float_value = struct.unpack('<f', binary)[0]
    return float_value

def binary2float_array(data):
    binary_data = data.tobytes()
    float_value = struct.unpack('<f', binary_data)[0]
    return float_value    

def float2binary(value):
    binary = struct.pack("f", value)
    binary_value = list(struct.unpack('B' * len(binary), binary))
    return binary_value

def calring(x, y, z):
    r = (x ** 2 + y ** 2) ** 0.5
    ring = math.atan2(z, r)
    return math.degrees(ring)

def angle2LID(angle): 
    if angle <= -22.3 and angle >= -22.5:
        Laser_ID = 0
        return Laser_ID
    elif angle <= 13.2 and angle >= 13.0:
        Laser_ID = 1
        return Laser_ID
    elif angle <= -17.2 and angle >= -17.4:
        Laser_ID = 2
        return Laser_ID
    elif angle <= 17.5 and angle >= 17.3:
        Laser_ID = 3
        return Laser_ID   
    elif angle <= -12.4 and angle >= -12.6:
        Laser_ID = 4
        return Laser_ID
    elif angle <= 22.1 and angle >= 21.9:
        Laser_ID = 5
        return Laser_ID
    elif angle <= -7.3 and angle >= -7.5:
        Laser_ID = 6
        return Laser_ID
    elif angle <= 27.0 and angle >= 26.8:
        Laser_ID = 7
        return Laser_ID
    elif angle <= -2.6 and angle >= -2.8:
        Laser_ID = 8
        return Laser_ID
    elif angle <= 31.8 and angle >= 31.6:
        Laser_ID = 9
        return Laser_ID
    elif angle <= 0.1 and angle >= -0.3:
        Laser_ID = 10
        return Laser_ID
    elif angle <= 34.5 and angle >= 34.3:
        Laser_ID = 11
        return Laser_ID
    elif angle <= 2.1 and angle >= 1.9:
        Laser_ID = 12
        return Laser_ID
    elif angle <= 37.0 and angle >= 36.8:
        Laser_ID = 13
        return Laser_ID
    elif angle <= 6.9 and angle >= 6.7:
        Laser_ID = 14
        return Laser_ID
    elif angle <= 42.5 and angle >= 42.3:
        Laser_ID = 15
        return Laser_ID