import sys
import time
from naoqi import ALProxy

import cv2
import numpy as np
from nao_driver import NaoDriver


def contour_center(contour):
    return np.mean(contour[:, :, 0]), np.mean(contour[:, :, 1])


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

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        ball_contour = max(contours, key=lambda contour: cv2.contourArea(contour))
        
        # Ball distance estimation via ball perimeter
        # perimeter = cv2.arcLength(ball_contour, True)
        # d_perimeter = r_to_d / (perimeter / (2 * np.pi))

        # Ball distance estimation via ball area
        area = cv2.contourArea(ball_contour)
        d_area = r_to_d / np.sqrt(area / np.pi)

        cx, cy = contour_center(ball_contour)
        
        return d_area, int(cx), int(cy)  # return distance_to_ball, x & y components of the ball's center in pixels
    else:
        return 0.0, None, None


def detect_goal(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV range for red
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Create a mask to isolate red regions
    mask = cv2.inRange(hsv, lower_red, upper_red)

    if np.any(mask > 0):
        cx, cy = np.mean(np.where(mask > 0), axis=1)
        return True, cx, cy
    else:
        return False, None, None


def bound(value, ceil=1.0):
    return max(-ceil, min(value, +ceil))


def changes_from_pixel(tx, ty, px, py):
    ex = tx - px
    ey = py - ty
    return [head_proportional_constant * ex, head_proportional_constant * ey]


if __name__ == '__main__':
    if len(sys.argv) > 1:
        user = sys.argv[1]
    else:
        user = 'nao'

    if user in 'etienne':
        robotIp = "localhost"
    elif user in 'victor':
        robotIp = "localhost"
    else:
        robotIp = ""

    robotPort = 11212

    fps = 10.
    dt_loop = 1. / fps

    # search for fov yaw : head_proportional_constant_yaw = dt_loop * fov_yaw / width(=320)
    # for fov pitch : head_proportional_constant_pitch = dt_loop * fov_pitch / height(=320)
    head_proportional_constant = dt_loop * np.pi / 256  # such as deltaAngle = pixelError * proportionalConstant
    head_fraction_max_speed = 0.1

    body_proportional_constant = 1.0
    body_step_frequency = 1.0  # maximum step frequency
    body_head_min_error = np.pi / 16  # in radians
    head_ball_goal_min_error = np.pi / 32  # in radians

    desired_dist_to_ball = 0.8
    dist_ball_min_error = 0.05
    walk_proportional_constant = 3.0

    walking_search_factor = 1.0  # maximum speed factor
    head_turning_search_factor = 0.1  # 10% of the maximum turning speed

    # Ensure to have the entire ball in the robot camera vision
    ball_target_security_factor = 1.5

    names = ["HeadYaw", "HeadPitch"]  # angles we want control on
    r_to_d = 16.0  # such as ball_distance = r_to_d * visual_ball_radius

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
        nao_drv.set_virtual_camera_path("/home/etrange/Documents/ensta/rob/Semestre5/visual_servoing/UE52-VS-IK/imgs")
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
    stiffness = 1.0
    motionProxy.setStiffnesses(names, stiffness)

    # Initialize yaw error (used to memorize where the ball is going)
    ball_yaw_error = 0.

    # infinite test loop, stops with Ctrl-C
    while True:
        t0_loop = time.time()

        # get image and detect the ball and the goal
        img_ok, img, width, height = nao_drv.get_image()
        ball_distance, bx, by = detect_ball(img)
        goal_detected, gx, gy = detect_goal(img)

        # Are the robot's head, its body, the yellow ball and the red goal align ?
        head_body_ball_goal_alignment = True

        # Look for the yellow ball
        if ball_distance > 0:
            # print("Ball distance estimation :", ball_distance)
            ball_diameter = 2 * (r_to_d / ball_distance)

            target_x = width / 2  # center the ball horizontally
            target_y = height - ball_target_security_factor * ball_diameter  # bottom the ball vertically

            head_ball_changes = changes_from_pixel(target_x, target_y, bx, by)
            ball_yaw_error = head_ball_changes[0]
        else:
            # print("No ball detected...")
            head_body_ball_goal_alignment = False  # no alignment without a ball
            head_ball_changes = [np.sign(ball_yaw_error) * head_turning_search_factor, max(0, np.random.random() - 0.5)]
        
        # Look for the red goal corners
        if goal_detected:
            target_x = width / 2  # center the goal horizontally
            target_y = 0.  # top the goal vertically
            head_goal_changes = changes_from_pixel(target_x, target_y, gx, gy)
        else:
            # print("No goal detected...")
            head_body_ball_goal_alignment = False  # no alignment without a goal
            head_goal_changes = [0.0, 0.0]

        # No alignment if ball and goal aren't aligned
        if abs(head_ball_changes[0] - head_goal_changes[0]) > head_ball_goal_min_error:
            head_body_ball_goal_alignment = False

        # Get head orientation to align the body accordingly
        yaw_head, _ = motionProxy.getAngles(names, True)  # assuming between -pi and +pi

        # No alignment if the body isn't align with the head
        if abs(yaw_head) > body_head_min_error:
            head_body_ball_goal_alignment = False

        # Compute how far the robot is from the desired ball distance
        d_dist = ball_distance - desired_dist_to_ball if ball_distance > 0 else dist_ball_min_error

        if head_body_ball_goal_alignment:
            print("Head-Body-Ball-Goal alignment!")

            # Move forward
            dx_factor = 1.0
            dy_factor = 0.0
            d_yaw_factor = 0.0
            changes = [0., 0.]
        else:
            # Maybe we need a Finite State Machine
            dx_factor = bound(walk_proportional_constant * d_dist) if abs(d_dist) > dist_ball_min_error else 0.0
            dy_factor = walking_search_factor if abs(d_dist) < dist_ball_min_error else 0.0
            d_yaw_factor = bound(body_proportional_constant * yaw_head) if abs(yaw_head) > body_head_min_error else 0.0

            # require some tuning...
            # changes[0] -= d_yaw_factor
            changes = [b_change + g_change for (b_change, g_change) in zip(head_ball_changes, head_goal_changes)]

        # Let's move!
        motionProxy.changeAngles(names, changes, head_fraction_max_speed)
        motionProxy.moveToward(dx_factor, dy_factor, d_yaw_factor, [["Frequency", body_step_frequency]])

        time_left = dt_loop - (time.time() - t0_loop)
        if time_left > 0:
            time.sleep(time_left)
        else:
            print("Out of time...")
