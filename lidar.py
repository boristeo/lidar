#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial


import thread
import time
import sys
import traceback 
import math
import struct
import serial
from visual import *

com_port = "COM5" # example: 5 == "COM6" == "/dev/tty5"
baudrate = 250000
visualization = True

lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality
lidar_rpm = 0

ZERO_I_OFFSET = 140


class Visualization:
  point = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0))
  pointb = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0, 0.4, 0))
  pointbb = points(pos=[(0,0,0) for i in range(360)], size=5, color=(1, 0, 0))
  pointi = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0 , 1, 0))
  pointib = points(pos=[(0,0,0) for i in range(360)], size=3, color=(1, 1, 0))
  pointibb = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0.4, 0, 0))

  zero_intensity_ring = ring(pos=(0,0,0), axis=(0,1,0), radius=ZERO_I_OFFSET - 1, thickness=1, color = color.yellow)

  label_speed = label(pos = (200,-700,0), xoffset=1, box=False, opacity=0.1)
  label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)

vis = Visualization()

def update_view(angle, data):
  angle_rad = angle * math.pi / 180.0
  c = math.cos(angle_rad)
  s = math.sin(angle_rad)

  dist_mm = data & 0x3fff # distance is coded on 14 bits
  quality = 0 # quality is on 16 bits
  #lidarData[angle] = dist_mm
  dist_x = dist_mm*c
  dist_y = dist_mm*s
  qual_x = (quality + ZERO_I_OFFSET) * c
  qual_y = (quality + ZERO_I_OFFSET) * s

  #reset the point display
  vis.point.pos[angle] = vector( 0, 0, 0 )
  vis.pointb.pos[angle] = vector( 0, 0, 0 )
  vis.pointbb.pos[angle] = vector( 0, 0, 0 )
  vis.pointi.pos[angle] = vector( 0, 0, 0 )
  vis.pointib.pos[angle] = vector( 0, 0, 0 )
  vis.pointibb.pos[angle] = vector( 0, 0, 0 )
  
  FAILED_MASK = 0x8000 
  BAD_MASK = 0x4000 
  if data & FAILED_MASK: 
    #vis.pointbb.pos[angle] = vector( dist_x, dist_y,0)
    vis.pointibb.pos[angle] = vector( qual_x, qual_y, 0)

  elif data & BAD_MASK: # quality is not as good as expected
    vis.pointb.pos[angle] = vector( dist_x, dist_y, 0)
    vis.pointib.pos[angle] = vector( qual_x, qual_y, 0)

  else: # quality is OK
    vis.point.pos[angle] = vector( dist_x, dist_y,0)
    vis.pointi.pos[angle] = vector( qual_x, qual_y, 0)


def read_Lidar():
  while True:
    try:
      ser.read_until('\nab\n', 1440)
      
      header = struct.unpack('<HHH', ser.read(6))
      angle = header[0]
      lidar_rpm = header[1] / 64.0
      vis.label_speed.text = "RPM : " + str(lidar_rpm)
      count = header[2]

      data_b = ser.read(count)
      data = struct.unpack('<' + str(count / 2) + 'H', data_b)
      
      for d in data:
        update_view(angle, d)
        angle = angle + 1 if angle + 1 < 360 else 0
            
    except :
        traceback.print_exc(file=sys.stdout)


if __name__ == '__main__':
  vis = Visualization
  ser = serial.Serial(com_port, baudrate)
  th = thread.start_new_thread(read_Lidar, ())
  while True:
    rate(60) # synchonous repaint at 60fps
