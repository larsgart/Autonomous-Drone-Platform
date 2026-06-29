from bezierComputation import *


def main():

    frameList = []
    numFrames = 0

    while True:
        frame = vs.read()
        frame = cv2.flip(frame, 1)

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in Top-Left, TR BR, BL order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                
                frameList.append((cX,cY))
                numFrames += 1
                
        if len(frameList) >= 2 and len(frameList) % 1 == 0:
            for index, f in enumerate(frameList):
                cv2.circle(frame, frameList[index], radius=2, color=(0, 255, 0), thickness=10)
            
        if len(frameList) >= 50:
            xpoints = [i[0] for i in frameList]
            ypoints = [i[1] for i in frameList]
            
            bezierParameters = get_bezier_parameters(xpoints, ypoints, degree=8)
            xvals, yvals = bezier_curve(bezierParameters, nTimes=100)
            
            for index, curvePiece in enumerate(xvals):
                    try:
                        cv2.line(frame, (int(xvals[index]),int(yvals[index])), (int(xvals[index+1]),int(yvals[index+1])), color=(255,0,0), thickness=4)  
                    except:
                        pass

        # prune the frameList           
        if len(frameList) > 60:
            frameList.pop(0)
        
        # show the output frame
        cv2.imshow("Frame", frame)
        
        # if the `r` key is pressed, erase the frameList
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            frameList = []

        # if the `q` key is pressed, break from the loop  
        if key == ord("q"):
            break

    cv2.destroyAllWindows()
    vs.stop()


if __name__ == '__main__':
    main()
