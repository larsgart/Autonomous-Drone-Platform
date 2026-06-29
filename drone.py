import sys

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from flight_controller_model import FlightController

if __name__ == '__main__':
    test_mode = sys.argv[1] if len(sys.argv) > 1 else False
    fc = FlightController(test_mode=test_mode)
    try:
        fc.run()
    except KeyboardInterrupt:
        fc.close()
