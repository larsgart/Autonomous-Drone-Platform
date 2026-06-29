'''
If UART is in use, kill it with:
  sudo systemctl stop serial-getty@ttyS0.service
'''
import sys
from models.motors import Motors

motors = Motors()
if not motors.test_motors():
    motors.close()
    sys.exit(1)

def main():
    print('entering main')
    while True:
        raw = input("Enter motor speed (0-100): ")
        try:
            speeds = [min(15, int(x)) for x in raw.split(",")]
        except ValueError:
            print("Invalid speeds")
            speeds = [0] * 4
        motors.output_speeds(speeds)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        motors.close()
