import numpy as np
from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys

import numpy as np
from scipy.special import comb

def get_bezier_parameters(X, Y, degree=2):
    """ Least square qbezier fit using penrose pseudoinverse.

    Parameters:

    X: array of x data.
    Y: array of y data. Y[0] is the y point for X[0].
    degree: degree of the Bézier curve. 2 for quadratic, 3 for cubic.

    Based on https://stackoverflow.com/questions/12643079/b%C3%A9zier-curve-fitting-with-scipy
    and probably on the 1998 thesis by Tim Andrew Pastva, "Bézier Curve Fitting".
    """
    if degree < 1:
        raise ValueError('degree must be 1 or greater.')

    if len(X) != len(Y):
        raise ValueError('X and Y must be of the same length.')

    if len(X) < degree + 1:
        raise ValueError(f'There must be at least {degree + 1} points to '
                         f'determine the parameters of a degree {degree} curve. '
                         f'Got only {len(X)} points.')

    def bpoly(n, t, k):
        """ Bernstein polynomial when a = 0 and b = 1. """
        return t ** k * (1 - t) ** (n - k) * comb(n, k)
        #return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bmatrix(T):
        """ Bernstein matrix for Bézier curves. """
        return np.matrix([[bpoly(degree, t, k) for k in range(degree + 1)] for t in T])

    def least_square_fit(points, M):
        M_ = np.linalg.pinv(M)
        return M_ * points

    T = np.linspace(0, 1, len(X))
    M = bmatrix(T)
    points = np.array(list(zip(X, Y)))
    
    final = least_square_fit(points, M).tolist()
    final[0] = [X[0], Y[0]]
    final[len(final)-1] = [X[len(X)-1], Y[len(Y)-1]]
    return final

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=50):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals

vs = VideoStream(src=0).start()

frameList = []
numFrames = 0


while True:
    # grab the frame from the threaded video stream and resize it to have a maximum width of 1000 pixels
    frame = vs.read()
    frame = cv2.flip(frame, 1)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     
    lower_green = np.array([50,100,50])
    upper_green = np.array([70,255,255])
 
    # preparing the mask to overlay
    mask = cv2.inRange(hsv, lower_green, upper_green)
     
    # The black region in the mask has the value of 0,
    # so when multiplied with original image removes all non-blue regions
    result = cv2.bitwise_and(frame, frame, mask = mask)
    
    # convert image to grayscale image
    gray_image = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
     
    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,0)
    
    # calculate moments of binary image
    M = cv2.moments(thresh)

    cX = int(M["m10"] / (M["m00"]+1))
    cY = int(M["m01"] / (M["m00"]+1))
    
    frameList.append((cX,cY))
    
    cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    if len(frameList) >= 2 and len(frameList) % 1 == 0:
        for index, f in enumerate(frameList):
            #cv2.circle(frame, f, 4, (0, 255, 247), -1)
            #cv2.line(frame, frameList[index], frameList[index + 1], (0, 255, 0), 2)
            cv2.circle(frame, frameList[index], radius=2, color=(0, 255, 0), thickness=10)
        
    if len(frameList) >= 50:
        xpoints = [i[0] for i in frameList]
        ypoints = [i[1] for i in frameList]
        
        bezierParameters = get_bezier_parameters(xpoints, ypoints, degree=8)
        
        xvals, yvals = bezier_curve(bezierParameters, nTimes=100)
        
        for index, curvePiece in enumerate(xvals):
                #cv2.circle(frame,center=(int(xvals[index]),int(yvals[index])), radius=2, color=(255, 0, 0), thickness=4)
                try:
                    cv2.line(frame, (int(xvals[index]),int(yvals[index])), (int(xvals[index+1]),int(yvals[index+1])), color=(255,0,0), thickness=4)
                    cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                except:
                    pass
                
    if len(frameList) > 50:
        frameList.pop(0)
    # show the output frame
    cv2.imshow("Frame", frame)
    
    numFrames += 1
    cv2.imshow('frame', frame)
    #cv2.imshow('mask', mask)
    #cv2.imshow('result', result)
    #cv2.imshow('gray_image',gray_image)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
vs.stop()