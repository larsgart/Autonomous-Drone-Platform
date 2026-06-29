import socket
import time
import sys
import json

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from zed_model import ZedModel

socc = None

def main():
   print('entering main')
   serverAddressPort = ('192.168.0.126', 44444)

   socc = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)

   zed = ZedModel(log=False)

   prev_pos = [0,0,0]

   print("Sending data")

   while True:
      rel_pos = zed.get_pos_relative()
      # rel_pos = zed.get_pos_global()
      pos_diff = [
         rel_pos[0] - prev_pos[0],
         rel_pos[2] - prev_pos[2]
      ]
      # print([rel_pos[0], rel_pos[2]])
      sock_data = json.dumps([rel_pos[0], rel_pos[2]]).encode()
      socc.sendto(sock_data, serverAddressPort)

      prev_pos = rel_pos

      time.sleep(0.05)

if __name__ == '__main__':
   try:
      main()
   except KeyboardInterrupt:
      print("Program killed")
      if socc:
         socc.close()
         print("Socket closed")