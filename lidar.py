import thread
import time
import sys
import traceback 
import math
import struct
import serial
from visual import *

sincos = [(math.sin(math.radians(a)), math.cos(math.radians(a))) for a in range(360)]
lidar_data = [(0, 0) for i in range(360)]

ZERO_I_OFFSET = 140

lidar_orientation = 0
wall_orientation = 90

class Visualization:
  point = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0))
  pointi = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0 , 1, 0))

  label_speed = label(pos = (200,-700,0), xoffset=1, box=False, opacity=0.1)
  label_angle = label(pos = (-700,700,0), xoffset=1, box=False, opacity=0.1)
  label_dist = label(pos = (-700,500,0), xoffset=1, box=False, opacity=0.1)

vis = Visualization()

def update_view(start_angle, packet_data):
  global lidar_orientation
  angle = start_angle + int(lidar_orientation)
  for data in packet_data:
    if angle >= 360:
      angle %= 360

    DIST_MASK = 0x3fff
    BAD_MASK = 0x4000 
    FAILED_MASK = 0x8000 

    dist_mm = data & DIST_MASK
    s, c = sincos[angle]
    dist_x = dist_mm * c
    dist_y = dist_mm * s
    lidar_data[angle] = (dist_x, dist_y)

    vis.point.pos[angle] = vector( dist_x, dist_y,0)
    vis.pointi.pos[angle] = vector( ZERO_I_OFFSET * c, ZERO_I_OFFSET * s, 0)
    
    if data & FAILED_MASK or dist_mm < ZERO_I_OFFSET: 
      vis.point.color[angle] = vector( 0, 0, 0)
      vis.pointi.color[angle] = vector( 0.4, 0, 0)

    elif data & BAD_MASK: # quality is not as good as expected
      vis.point.color[angle] = vector( 0, 0.4, 0)
      vis.pointi.color[angle] = vector( 0.8, 0.8, 0)

    else: # quality is OK
      vis.point.color[angle] = vector( 0, 1, 0)
      vis.pointi.color[angle] = vector( 0, 1, 0)

    angle += 1

def update_position():
  global lidar_orientation, wall_orientation
  rel_angle, d_normal = get_relative_pos_to_wall(wall_orientation)
  lidar_orientation += rel_angle
  vis.label_angle.text = str(lidar_orientation) + ' degrees'
  vis.label_dist.text = str(d_normal) + 'mm'
  
def get_relative_pos_to_wall(angle):

  x1, y1 = lidar_data[angle - 5]
  x2, y2 = lidar_data[angle + 5]
  rel_angle_5 = math.degrees(math.atan2(y1 - y2, x1 - x2))
  x1, y1 = lidar_data[angle - 10]
  x2, y2 = lidar_data[angle + 10]
  rel_angle_10 = math.degrees(math.atan2(y1 - y2, x1 - x2))
  x1, y1 = lidar_data[angle - 15]
  x2, y2 = lidar_data[angle + 15]
  rel_angle_15 = math.degrees(math.atan2(y1 - y2, x1 - x2))

  if abs(rel_angle_5 - rel_angle_10) > 5 or abs(rel_angle_10 - rel_angle_15) > 5:
    rel_angle = min(rel_angle_5, rel_angle_10, rel_angle_15, key=lambda x: abs(x))
  else:
    rel_angle = rel_angle_15
  x3, y3 = lidar_data[angle + int(rel_angle)]
  d_normal = (x3 ** 2 + y3 ** 2) ** 0.5
  
  return (-rel_angle, d_normal)

def read_Lidar():
  received = 0
  while True:
    try:
      ser.read_until('\nab\n', 1440)
      
      header = struct.unpack('<HHH', ser.read(6))
      angle = header[0]
      lidar_rpm = header[1] / 64.0
      vis.label_speed.text = "RPM : " + str(lidar_rpm)
      count = header[2]

      data = struct.unpack('<' + str(count / 2) + 'H', ser.read(count))
      
      update_view(angle, data)

      received += count >> 1 
      if received > 360:
        received %= 360
        update_position()
            
    except :
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
  com_port = "COM5"
  baudrate = 250000
  ser = serial.Serial(com_port, baudrate)
  th = thread.start_new_thread(read_Lidar, ())
  while True:
    rate(24)
