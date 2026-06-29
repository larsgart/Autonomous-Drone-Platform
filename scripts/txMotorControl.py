import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import serial
from Models.motor_model import Motors
from Models.rx_model import RX
   
rx = RX()
test_motors = Motors()
tests_pass = test_motors.test_motors()

def outputSpeeds(speeds: list):
   reordered_speeds = [
      speeds[0],
      speeds[3],
      speeds[1],
      speeds[2]
   ]
   test_motors.output_speeds(reordered_speeds)


def main():
   print('entering main')
   while 1:
      rx_data = rx.readRX()[2]
      speeds = [(rx_data - 1000) / 10] * 4
      outputSpeeds(speeds)


if __name__ == '__main__':
   try:
      if tests_pass:
         main()
      else:
         test_motors.disconnect()
         del test_motors
   except KeyboardInterrupt:
      test_motors.disconnect()