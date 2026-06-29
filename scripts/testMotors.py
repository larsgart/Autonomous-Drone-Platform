'''
If UART is in use, kill it with this command:
sudo systemctl stop serial-getty@ttyS0.service
Also use if you get this error: "device reports readiness to read but returned no data"
'''

import sys
sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")
from motor_model import Motors

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

# Main function to receive motor speeds and output them
def main():
   print('entering main')
   while True:
      spd = input("Enter motor speed (0-100): ")
      try:
         speedList = [min(15, int(x)) for x in spd.split(",")]
      except ValueError:
         print("Invalid Speeds!")
         speedList = [0] * 4
      outputSpeeds(speedList)

if __name__ == '__main__':
   try:
      if tests_pass:
         main()
      else:
         test_motors.disconnect()
         del test_motors
   except KeyboardInterrupt:
      test_motors.disconnect()