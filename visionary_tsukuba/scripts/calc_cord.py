import math
from config import * 

def calc_x_center(x_min, x_max):
    return (x_min + x_max)/2

def calc_y_center(y_min, y_max):
    return (y_min + y_max)/2

def xy2angle(x_center, y_center, is_radian=True):
    CONF = calc_coordinates_config()

    theta = (CONF.horizontal_FOV/CONF.horizontal_resolution) * x_center - (CONF.horizontal_FOV / 2)
    phi = (CONF.vertical_FOV/CONF.vertical_resolution) * y_center - (CONF.vertical_FOV / 2)

    #theta = 0.117 * x_center - 60 # 方位角(azimuth), degree
    #phi = 0.182 * y_center - 52.5 #仰角(elevation angle), degree

    if is_radian == True:
        return math.radians(theta), math.radians(phi)
    else:
        return theta, phi
    
def calc_distance2d(distance, phi):
    distance_2d = distance * math.cos(phi)
    return abs(distance_2d)

def calc_coordinates(distance, theta, phi):
    distance_2d = calc_distance2d(distance, phi)
    x = distance_2d * math.sin(theta)
    y = distance_2d * math.cos(theta)
    z = distance * math.sin(phi)
    return x*2, y, -z