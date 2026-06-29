import sys
from models.motors import Motors
from models.rx import RX

rx = RX()
motors = Motors()
if not motors.test_motors():
    motors.close()
    sys.exit(1)

def main():
    print('entering main')
    while True:
        throttle = rx.read()[2]
        speed = (throttle - 1000) / 10
        motors.output_speeds([speed] * 4)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        motors.close()
