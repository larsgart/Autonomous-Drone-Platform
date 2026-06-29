import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import socket


timeStampedData = []

UDP_IP = "0.0.0.0"
UDP_PORT = 44444
sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Drone Quaternion Orientation Visualizer")
    resizewin(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):

            # writing timeStampedData to text file in same directory
            textfile = open("timeStampedData.txt", "w")
            for element in timeStampedData:
                textfile.write(str(element) + "\n")
            textfile.close()

            break

        [w, nx, ny, nz] = read_data()
        draw(w, nx, ny, nz)
        pygame.display.flip()
        #frames += 1
    #print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))

def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def read_data():
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes

    timeStampedData.append([x for x in data])


    # Convert each byte to a float ranging from -1 to 1
    return [2/255 * b - 1 for b in data[:4]]

def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    [yaw, pitch , roll] = quat_to_ypr([w, nx, ny, nz])
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(2 * math.acos(w) * 180.00/math.pi, -1 * nx, nz, ny)

    # glBegin(GL_QUADS)
    # glColor3f(0.0, 1.0, 0.0)
    # glVertex3f(1.0, 0.2, -1.0)
    # glVertex3f(-1.0, 0.2, -1.0)
    # glVertex3f(-1.0, 0.2, 1.0)
    # glVertex3f(1.0, 0.2, 1.0)
    # glColor3f(1.0, 0.5, 0.0)
    # glVertex3f(1.0, -0.2, 1.0)
    # glVertex3f(-1.0, -0.2, 1.0)
    # glVertex3f(-1.0, -0.2, -1.0)
    # glVertex3f(1.0, -0.2, -1.0)
    # glColor3f(1.0, 0.0, 0.0)
    # glVertex3f(1.0, 0.2, 1.0)
    # glVertex3f(-1.0, 0.2, 1.0)
    # glVertex3f(-1.0, -0.2, 1.0)
    # glVertex3f(1.0, -0.2, 1.0)
    # glColor3f(1.0, 1.0, 0.0)
    # glVertex3f(1.0, -0.2, -1.0)
    # glVertex3f(-1.0, -0.2, -1.0)
    # glVertex3f(-1.0, 0.2, -1.0)
    # glVertex3f(1.0, 0.2, -1.0)
    # glColor3f(0.0, 0.0, 1.0)
    # glVertex3f(-1.0, 0.2, 1.0)
    # glVertex3f(-1.0, 0.2, -1.0)
    # glVertex3f(-1.0, -0.2, -1.0)
    # glVertex3f(-1.0, -0.2, 1.0)
    # glColor3f(1.0, 0.0, 1.0)
    # glVertex3f(1.0, 0.2, -1.0)
    # glVertex3f(1.0, 0.2, 1.0)
    # glVertex3f(1.0, -0.2, 1.0)
    # glVertex3f(1.0, -0.2, -1.0)
    # glEnd()

    vertices= ((1, -1, -1),(1, 1, -1),(-1, 1, -1),(-1, -1, -1),(1, -1, 1),(1, 1, 1),(-1, -1, 1),(-1, 1, 1))
    edges = ((0,1),(0,3),(0,4),(2,1),(2,3),(2,7),(6,3),(6,4),(6,7),(5,1),(5,4),(5,7))

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def quat_to_ypr(q):
    yaw   = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    val = 2.0 * (q[1] * q[3] - q[0] * q[2])
    pitch = -math.asin(1 if val > 1 else -1 if val < -1 else val)
    roll  = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]

if __name__ == '__main__':
    main()