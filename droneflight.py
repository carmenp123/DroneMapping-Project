#Code Section 1
#We're importing extensions also called libraries for less code
from djitellopy import Tello
import KeyPressModule as kp
import cv2
import numpy as np
import time
import math 
#End of section 1

#Section 2 Brief Explanation
#The first 3 lines set the variable parameter for forward speed

############ PARAMETERS ############
fSpeed = 112  # Forward/Backward cm/sec
aSpeed = 26  # Angular Velocity, Degrees/sec
uSpeed = 26  #up/down cm/sec
interval = 0.25

dInterval = fSpeed * interval  # Distance Interval
aInterval = aSpeed * interval  # Angular Interval
hInterval = uSpeed * interval  # Height Interval
#########################################
#End of section 2

#Section 3 brief explanation
#We set the angular velocity to 0 and initialize thee key press module
#It also explains what each key does when its pressed
#We also create variables lr,fb,ud,yv and set them to zero
x, y = 500, 500
a = 0
yaw = 0

kp.init()
tello = Tello()
# tello.connect()
# print(tello.get_battery())
# global img
# tello.streamon()
points = []

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50
    global x, y, yaw, a
    d = 0

    if kp.getKey("LEFT"):
        lr = -speed
        d = dInterval
        a = -180

    elif kp.getKey("RIGHT"):
        lr = speed
        d = -dInterval
        a = 180

    if kp.getKey("UP"):
        fb = speed
        d = dInterval
        a = 270

    elif kp.getKey("DOWN"):
        fb = -speed
        d = -dInterval
        a = -90

    if kp.getKey("w"):
        ud = speed
    elif kp.getKey("s"):
        ud = -speed

    if kp.getKey("a"):
        yv = -speed
        yaw += aInterval

    elif kp.getKey("d"):
        yv = speed
        yaw -= aInterval

    if kp.getKey("q"):
        tello.land()
        time.sleep(3)
    if kp.getKey("e"): tello.takeoff()
    #End of section 3
    #Section 4 explanation
    #This is the mapping thing its using red pointers to draw on the map

    if kp.getKey("z"):
      cv2.imwrite(f'Resources/Image/{time.time()}.jpg', img)
      time.sleep(0.3)
    
    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))

    return [lr, fb, ud, yv, x, y]


def drawPoints(img, points):
    for point in points:
        cv2.circle(img, (points[0], points[1]), 20, (0, 0, 255), cv2.FILLED)

        #ENd of section 4

        # Explanation for section 5
        #Without thid no point will be added


while True:
    vals = getKeyboardInput()
    tello.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    img = np.zeros((1000, 1000, 3), np.uint8)
    points.append((vals[4], vals[5]))
    drawPoints(img, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
    
# img = tello.get_frame_read().frame
# img = cv2.resize(img, (360, 240))
# cv2.imshow("image", img)
# cv2.waitKey(1)
