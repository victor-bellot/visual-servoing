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

    ball_contour = None
    ball_area = 0.
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)  # the curve is closed

        if (4. * np.pi * area) / (perimeter ** 2.) > 0.8:  # select circular contour only
            if area > ball_area:
                ball_contour = contour
                ball_area = area

    if ball_contour is not None:
        return True, np.mean(ball_contour[:, :, 0]), np.mean(ball_contour[:, :, 1])
    else:
        return False, None, None


if __name__ == '__main__':
    # robotIp = "localhost"
    robotIp = "172.17.0.1"
    robotPort = 11212

    fps = 10.
    dt_loop = 1. / fps

    fractionMaxSpeed = 0.1
    proportionalConstant = dt_loop * np.pi / 1000.  # such as deltaAngle = pixelError * proportionalConstant
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
    # nao_drv.set_virtual_camera_path("/home/victor/nao/UE52-VS-IK/imgs")
    nao_drv.set_virtual_camera_path("/home/dockeruser/shared/imgs")

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
        changes = [np.random.random() - 0.5 for _ in range(2)]
        if ball_detected:
            ex = (nx / 2) - bx
            ey = by - (ny / 2)
            changes = [proportionalConstant * ex, proportionalConstant * ey]

        motionProxy.changeAngles(names, changes, fractionMaxSpeed)

        dt = dt_loop - (time.time() - t0_loop)
        if dt > 0:
            time.sleep(dt)
        else:
            print("Out of time...")
