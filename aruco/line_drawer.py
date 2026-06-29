""" 
MIT License
Copyright (c) 2019-2022 JetsonHacks

Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
NVIDIA Jetson Nano Developer Kit using OpenCV
Drivers for the camera and OpenCV are included in the base image

gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

#from turtle import width
import cv2
import imutils
import random
import time
global window_height
window_height = 540

global window_width
window_width = 960


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=window_width,
    display_height=window_height,
    framerate=120,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    window_title = "Autonomous Drone Platform"
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    if video_capture.isOpened():
        numFrames = 0
        coordList = []
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:

                numFrames += 1

                ret_val, frame = video_capture.read()

                #frame = imutils.resize(frame, width=960)

                if numFrames % 1 == 0:
                    coordPair0 = (random.randint(0,window_height),random.randint(0,window_height))
                    coordPair1 = (random.randint(0,window_width),random.randint(0,window_width))
                    coordList.append((coordPair0,coordPair1))
                
                if len(coordList) > 30:
                    coordList.pop(0)
                
                for coordPair in coordList:
                    # for seizures cv2.line(frame, coordPair[0], coordPair[1], (random.randint(0,255), random.randint(0,255), random.randint(0,255)), 2)
                    cv2.line(frame, coordPair[0], coordPair[1], (0, 255, 0), 2)

                cv2.imshow(window_title, frame)

                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    show_camera()
