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
    lower_yellow = np.array([20, 100, 100])  # sim
    upper_yellow = np.array([30, 255, 255])  # sim

    # lower_yellow = np.array([15, 100, 32])  # real
    # upper_yellow = np.array([27, 255, 255])  # real

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
        perimeter = cv2.arcLength(ball_contour, True)
        
        # Ball distance estimate
        d_perimeter = r_to_d / (perimeter / (2 * np.pi))
        d_area = r_to_d / np.sqrt(cv2.contourArea(ball_contour) / np.pi)
        
        return d_area, np.mean(ball_contour[:, :, 0], dtype=int), np.mean(ball_contour[:, :, 1], dtype=int)
    else:
        return -1, None, None


if __name__ == '__main__':
    if len(sys.argv) > 1:
        user = sys.argv[1]
    else:
        user = 'nao'

    if user in 'etienne':
        robotIp = "172.17.0.1"
    elif user in 'victor':
        robotIp = "localhost"
    else:
        robotIp = ""

    robotPort = 11212

    fps = 10.
    dt_loop = 1. / fps

    head_fraction_max_speed = 0.1
    head_proportional_constant = dt_loop * np.pi / 256  # such as deltaAngle = pixelError * proportionalConstant

    body_proportional_constant = 1.0
    body_step_frequency = 1.0  # maximum step frequency
    body_head_min_error = np.pi / 20  # in radians

    names = ["HeadYaw", "HeadPitch"]
    r_to_d = 5.0

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
    if user in 'etienne':
        nao_drv.set_virtual_camera_path("/home/dockeruser/shared/imgs")
    elif user in 'victor':
        nao_drv.set_virtual_camera_path("/home/victor/nao/UE52-VS-IK/imgs")
    else:
        pass

    # Send NAO to Pose Init : if it's not standing then standing up
    postureProxy.goToPosture("StandInit", 0.5)

    # allow to stop the motion when losing ground contact, NAO stops walking
    # when lifted  (True is default)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    # only activate head pitch and yaw servos
    stiffnesses = 1.0
    motionProxy.setStiffnesses(names, stiffnesses)

    # Initialize yaw error (used to memorize where the ball is going)
    ex = 0

    # infinite test loop, stops with Ctrl-C
    while True:
        t0_loop = time.time()

        # tends to align body with head
        yaw_head, _ = motionProxy.getAngles(names, True)  # assuming between -pi and +pi

        dx_factor = 0.0
        dy_factor = 0.0
        d_yaw_factor = max(min(body_proportional_constant * yaw_head, 1), -1) if abs(yaw_head) > body_head_min_error else 0.0

        motionProxy.moveToward(dx_factor, dy_factor, d_yaw_factor, [["Frequency", body_step_frequency]])

        # get image and detect ball
        img_ok, img, nx, ny = nao_drv.get_image()
        ball_distance, bx, by = detect_ball(img)

        # Look for the yellow ball
        if ball_distance > 0:
            print("Estimated ball distance :", ball_distance)
            ex = (nx / 2) - bx
            ey = by - (ny / 2)
            changes = [head_proportional_constant * ex, head_proportional_constant * ey]
        else:
            print("No ball detected...")
            changes = [np.sign(ex) * 0.1, max(0, np.random.random() - 0.5)]
        
        # changes[0] -= d_yaw_factor  # need to be tuned
        motionProxy.changeAngles(names, changes, head_fraction_max_speed)

        dt = dt_loop - (time.time() - t0_loop)
        if dt > 0:
            time.sleep(dt)
        else:
            print("Out of time...")
