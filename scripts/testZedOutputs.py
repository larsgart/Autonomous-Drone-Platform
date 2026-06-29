import sys

sys.path.append("/home/drone/Autonomous-Drone-Platform/Models")

from zed_model import ZedModel

def main():
    zed = ZedModel()
    p0 = zed.get_pos_relative()
    print(p0)
    zed.closeCamera()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Program Closed')