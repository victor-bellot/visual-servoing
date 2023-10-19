import sys
import time
from naoqi import ALProxy

import cv2
import numpy as np
from nao_driver import NaoDriver


def detect_ball(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV range for yellow
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Create a mask to isolate yellow regions
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Apply morphological operations to clean up the mask (optional)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        ball_contour = max(contours, key=lambda contour: cv2.contourArea(contour))
        return True, np.mean(ball_contour[:, :, 0]), np.mean(ball_contour[:, :, 1])
    else:
        return False, None, None


def sign(v):
    return +1 if v > 0 else -1


if __name__ == '__main__':
    robotIp = "localhost"
    robotPort = 11212

    fps = 6
    dt_loop = 1. / fps

    fractionMaxSpeed = 0.1
    deltaAngle = 1.0 * np.pi / 180  # 1 degree
    names = ["HeadYaw", "HeadPitch"]

    if len(sys.argv) == 3:
        robotIp = sys.argv[1]
        robotPort = int(sys.argv[2])

    print(robotIp, robotPort)

    try:
        motionProxy = ALProxy("ALMotion", robotIp, robotPort)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print("Error was: ", e)
        exit()

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIp, robotPort)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print("Error was: ", e)
        exit()

    # create NAO driver
    nao_drv = NaoDriver(nao_ip=robotIp, nao_port=robotPort)

    # Set your Nao path HERE
    nao_drv.set_virtual_camera_path("/home/victor/nao/UE52-VS-IK/imgs")

    # Send NAO to Pose Init : if it's not standing then standing up
    postureProxy.goToPosture("StandInit", 0.5)

    # allow to stop the motion when losing ground contact, NAO stops walking
    # when lifted  (True is default)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    # only activate head pitch and yaw servos
    stiffnesses = 1.0
    motionProxy.setStiffnesses(names, stiffnesses)

    # infinite test loop, stops with Ctrl-C
    while True:
        t0_loop = time.time()

        img_ok, img, nx, ny = nao_drv.get_image()
        ball_detected, bx, by = detect_ball(img)

        # Look for the yellow ball
        directions = [np.random.random(), np.random.random()]
        if ball_detected:
            ex = (nx / 2) - bx
            ey = by - (ny / 2)

            # Bang Bang
            directions = [sign(ex), sign(ey)]

        changes = [deltaAngle * direction for direction in directions]  # delta yaw & delta pitch
        motionProxy.changeAngles(names, changes, fractionMaxSpeed)

        dt = dt_loop - (time.time() - t0_loop)
        if dt > 0:
            time.sleep(dt)
        else:
            print("Out of time...")
