import cv2
import numpy as np
import time

def nothing(x):
    pass

def turnLeft():
    print("left")

def goStraight():
    print("straight")

def turnRight():
    print("right")

def Stop():
    print("stop")

# Define the TCP URL from the Raspberry Pi
tcp_url = 'tcp://192.168.43.39:8554'

# Open a connection to the TCP stream
cam = cv2.VideoCapture(tcp_url)

cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,179,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

cv2.createTrackbar('morph_k','image',1,10,nothing)

#lower = np.array([0, 20, 160])
#upper = np.array([25, 255, 255])
#approxiamtely skin colour
# Set default value for MIN HSV trackbars.
cv2.setTrackbarPos('HMin', 'image', 0)
cv2.setTrackbarPos('SMin', 'image', 0)
cv2.setTrackbarPos('VMin', 'image', 0)
# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 241)

cv2.setTrackbarPos('morph_k', 'image', 10)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0
morph_k = 0

left = False
right = False
straight = False


while (True):
    ret, image = cam.read()
    if ret:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')

        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and max HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        mask = cv2.inRange(hsv, lower, upper)

        #make a binary image from mask
        ret2, bin = cv2.threshold(mask, 127, 255, 0)

        #apply morphological opening to eliminate inpurities in color detection
        morph_k = cv2.getTrackbarPos('morph_k', 'image')
        m_open = cv2.morphologyEx(bin, cv2.MORPH_OPEN, np.ones((morph_k, morph_k), np.uint8))


        #find the (second if counting the background) largest connected component
        connect = cv2.connectedComponentsWithStats(m_open, 4, cv2.CV_32S)
        stats = connect[2]
        centroids = connect[3]

        if (len(stats) > 1):
            sorted_i = np.argsort(-stats[:, -1])
            stat = stats[sorted_i[1]]
            (cx, cy) = centroids[sorted_i[1]]
            cv2.rectangle(image, (stat[0], stat[1]), (stat[0] + stat[2], stat[1] + stat[3]), (0, 255, 0), 3)
            cv2.circle(image, (int(cx), int(cy)), 1, (0,255, 0), 3)
            cv2.putText(image, "following", (stat[0], stat[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
            cv2.putText(image, "center", (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)

            #process where to go
            h, w, channels = image.shape

            cv2.line(image, (w//3, 0), (w//3, h), (0, 0, 255), 1)
            cv2.line(image, (2 * w // 3, 0), (2 * w // 3, h), (0, 0, 255), 1)

            if (int(cx) < w//3):
                turnLeft()
            elif (int(cx) > w//3 * 2):
                turnRight()
            else:
                if (right or left):
                    Stop()
                else:
                    goStraight()

        #show all images for debug
        cv2.imshow("image", image)
        cv2.imshow("hsv", hsv)
        cv2.imshow("mask", mask)
        cv2.imshow("open", m_open)

cam.release()
cv2.destroyAllWindows()


