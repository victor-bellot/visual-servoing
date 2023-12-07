import time
import numpy as np
from naoqi import ALProxy
from nao_driver import NaoDriver

from tools import *


class NaoSoccer:
    def __init__(self, robot_ip, virtual_camera_path=None, robot_port=11212):
        if robot_ip is None:
            raise Exception

        print("Initializing a NaoSoccer...")

        fps = 10.
        self.dt = 1. / fps

        # search for fov yaw : head_proportional_constant_yaw = dt_loop * fov_yaw / width(=320)
        # for fov pitch : head_proportional_constant_pitch = dt_loop * fov_pitch / height(=320)
        # such as deltaAngle = pixelError * proportionalConstant
        self.head_proportional_constant = self.dt * np.pi / 256
        self.body_proportional_constant = 1.0
        self.walk_proportional_constant = 3.0

        self.head_fraction_max_speed = 0.1
        self.head_turning_search_factor = 0.1  # 10% of the maximum turning speed
        self.body_step_frequency = 1.0  # maximum step frequency
        self.walking_search_factor = 1.0  # maximum speed factor

        # Alignment maximum angles
        self.body_head_min_error = np.pi / 16  # in radians
        self.head_ball_goal_min_error = np.pi / 32  # in radians
        self.dist_ball_min_error = 0.05

        self.desired_dist_to_ball = 0.8
        self.r_to_d = 16.0  # such as ball_distance = r_to_d * visual_ball_radius
        self.ball_target_security_factor = 1.5  # ensure to have the entire ball in the robot camera vision

        self.angle_names = ["HeadYaw", "HeadPitch"]  # angles we want control on

        try:
            self.motion_proxy = ALProxy("ALMotion", robot_ip, robot_port)
        except Exception, e:
            print "Could not create proxy to ALMotion"
            print("Error was: ", e)
            exit()

        try:
            self.posture_proxy = ALProxy("ALRobotPosture", robot_ip, robot_port)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print("Error was: ", e)
            exit()

        # create NAO driver
        self.nao_drv = NaoDriver(nao_ip=robot_ip, nao_port=robot_port)

        # Set your Nao path HERE
        if virtual_camera_path is not None:
            self.nao_drv.set_virtual_camera_path(virtual_camera_path)

        # Send NAO to Pose Init : if it's not standing then standing up
        self.posture_proxy.goToPosture("StandInit", 0.5)

        # allow to stop the motion when losing ground contact, NAO stops walking
        # when lifted  (True is default)
        self.motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        # only activate head pitch and yaw servos
        stiffness = 1.0
        self.motion_proxy.setStiffnesses(self.angle_names, stiffness)

        # Initialize yaw error (used to memorize where the ball is going)
        self.ball_yaw_error = 0.

    def changes_from_pixel(self, tx, ty, px, py):
        ex = tx - px
        ey = py - ty
        return [self.head_proportional_constant * error for error in (ex, ey)]

    def run(self):
        # infinite test loop, stops with Ctrl-C
        while True:
            t0_loop = time.time()

            # get image and detect the ball and the goal
            img_ok, img, width, height = self.nao_drv.get_image()
            ball_radius, bx, by = detect_ball(img)
            goal_detected, gx, gy = detect_goal(img)

            # Convert visual ball radius into an distance estimate
            ball_distance = self.r_to_d / ball_radius if ball_radius else 0.0

            # Are the robot's head, its body, the yellow ball and the red goal align ?
            head_body_ball_goal_alignment = True

            # Look for the yellow ball
            if ball_distance > 0:
                # print("Ball distance estimation :", ball_distance)

                target_x = width / 2  # center the ball horizontally
                target_y = height - self.ball_target_security_factor * (2 * ball_radius)  # bottom the ball vertically

                head_ball_changes = self.changes_from_pixel(target_x, target_y, bx, by)
                self.ball_yaw_error = head_ball_changes[0]
            else:
                # print("No ball detected...")
                head_body_ball_goal_alignment = False  # no alignment without a ball
                head_ball_changes = [np.sign(self.ball_yaw_error) * self.head_turning_search_factor,
                                     max(0, np.random.random() - 0.5)]

            # Look for the red goal corners
            if goal_detected:
                target_x = width / 2  # center the goal horizontally
                target_y = height / 2  # center the goal vertically
                head_goal_changes = self.changes_from_pixel(target_x, target_y, gx, gy)
            else:
                # print("No goal detected...")
                head_body_ball_goal_alignment = False  # no alignment without a goal
                head_goal_changes = [0.0, 0.0]

            # No alignment if ball and goal aren't aligned
            if abs(head_ball_changes[0] - head_goal_changes[0]) > self.head_ball_goal_min_error:
                head_body_ball_goal_alignment = False

            # Get head orientation to align the body accordingly
            yaw_head, _ = self.motion_proxy.getAngles(self.angle_names, True)  # assuming between -pi and +pi

            # No alignment if the body isn't align with the head
            if abs(yaw_head) > self.body_head_min_error:
                head_body_ball_goal_alignment = False

            # Compute how far the robot is from the desired ball distance
            d_dist = ball_distance - self.desired_dist_to_ball if ball_distance > 0 else self.dist_ball_min_error

            if head_body_ball_goal_alignment:
                print("Head-Body-Ball-Goal alignment!")

                # Move forward
                dx_factor = 1.0
                dy_factor = 0.0
                d_yaw_factor = 0.0
                changes = [0., 0.]
            else:
                # Maybe we need a Finite State Machine
                dx_factor = bound(self.walk_proportional_constant * d_dist) \
                    if abs(d_dist) > self.dist_ball_min_error else 0.0

                dy_factor = self.walking_search_factor \
                    if abs(d_dist) < self.dist_ball_min_error else 0.0

                d_yaw_factor = bound(self.body_proportional_constant * yaw_head) \
                    if abs(yaw_head) > self.body_head_min_error else 0.0

                # require some tuning...
                # changes[0] -= d_yaw_factor
                changes = [b_change + g_change for (b_change, g_change) in zip(head_ball_changes, head_goal_changes)]

            # Let's move!
            self.motion_proxy.changeAngles(self.angle_names, changes, self.head_fraction_max_speed)
            self.motion_proxy.moveToward(dx_factor, dy_factor, d_yaw_factor, [["Frequency", self.body_step_frequency]])

            time_left = self.dt - (time.time() - t0_loop)
            if time_left > 0:
                time.sleep(time_left)
            else:
                print("Out of time...")
