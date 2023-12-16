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

        head_turning_speed = np.pi / 32.

        # search for fov yaw : head_proportional_constant_yaw = dt_loop * fov_yaw / width(=320)
        # for fov pitch : head_proportional_constant_pitch = dt_loop * fov_pitch / height(=320)
        # such as deltaAngle = pixelError * proportionalConstant
        self.head_proportional_constant = head_turning_speed / 320.
        self.body_proportional_constant = 1.0

        self.omega_search = 1.  # in radians per seconds
        self.yaw_search = np.pi / 2.  # in radians
        self.pitch_search = np.pi / 16.  # in radians
        self.body_to_head_factor = 1. / 10.

        self.head_fraction_max_speed = 0.1
        self.body_step_frequency = 1.0  # maximum step frequency
        self.walking_search_factor = 1.0  # maximum speed factor

        # Alignment maximum angles
        self.body_head_accuracy = np.pi / 16.  # in radians
        self.head_ball_goal_min_error = np.pi / 32.  # in radians
        self.ball_distance_accuracy = 0.05

        self.desired_dist_to_ball = 0.8
        self.r_to_d = 16.  # such as ball_distance = r_to_d * visual_ball_radius

        self.angle_names = ["HeadYaw", "HeadPitch"]  # angles we want control on
        self.pixel_distance_accuracy = 20.  # in pixels

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

        # Define camera image variables
        self.image_updated = False
        self.camera_image = None
        self.image_dimension = (0, 0)  # (width, height)

        # Define ball variables
        self.ball_updated = False
        self.ball_radius = None
        self.ball_position = (0, 0)  # pixel (x, y) on image
        self.ball_distance = 0

        # Define ball target variables
        self.target_updated = False
        self.target_position = (0, 0)  # pixel (x, y) on image

        # Define head yaw variables
        self.yaw_updated = False
        self.head_yaw = 0.  # in radians

        # Define goal variables
        self.goal_updated = False
        self.goal_detected = None
        self.goal_position = (0, 0)  # pixel (x, y) on image

        # Define motion variables
        self.motion = {
            'head_yaw': 0.,
            'head_pitch': 0.,
            'body_x': 0.,
            'body_y': 0.,
            'body_yaw': 0.,
        }

    def reset(self):
        self.image_updated = False
        self.ball_updated = False
        self.target_updated = False
        self.yaw_updated = False
        self.goal_updated = False

        # Zero motions
        for motion_name in self.motion.keys():
            self.motion[motion_name] = 0.
        self.move()

    def changes_from_pixel(self, tx, ty, px, py):
        return [self.head_proportional_constant * error for error in (tx - px, py - ty)]

    """ UPDATE METHODS """

    def update_image(self):
        if not self.image_updated:
            img_ok, img, width, height = self.nao_drv.get_image()
            if img_ok:
                self.image_updated = True
                self.camera_image = img
                self.image_dimension = (width, height)

        return self.image_updated

    def update_ball(self):
        if self.update_image() and not self.ball_updated:
            ball_radius, bx, by = detect_ball(self.camera_image)

            self.ball_updated = True
            self.ball_radius = ball_radius
            self.ball_position = (bx, by)

            # Convert visual ball radius into an distance estimate
            self.ball_distance = self.r_to_d / ball_radius if ball_radius else 0.0

        return self.ball_updated

    def update_target(self):
        if self.update_ball() and not self.target_updated:
            width, height = self.get_image_dimension()
            ball_radius = self.get_ball_radius()

            target_x = width / 2  # center the ball horizontally
            target_y = height - 3 * ball_radius  # bottom the ball

            self.target_updated = True
            self.target_position = (target_x, target_y)

        return self.target_updated

    def update_head_yaw(self):
        if not self.yaw_updated:
            # Get head orientation to align the body accordingly
            self.head_yaw, _ = self.motion_proxy.getAngles(self.angle_names, True)  # assuming between -pi and +pi
            self.yaw_updated = True

        return self.yaw_updated

    def update_goal(self):
        if self.update_image() and not self.goal_updated:
            goal_detected, gx, gy = detect_goal(self.camera_image)

            self.goal_updated = True
            self.goal_detected = goal_detected
            self.goal_position = (gx, gy)

        return self.goal_updated

    """ GETTER & CONDITION METHODS """

    def ball_found(self):
        if self.update_ball():
            return self.ball_distance > 0
        return False

    def get_image_dimension(self):
        if self.update_image():
            return self.image_dimension

    def get_ball_radius(self):
        if self.update_ball():
            return self.ball_radius

    def get_ball_distance(self):
        if self.update_ball():
            return self.ball_distance

    def get_target_position(self):
        if self.update_target():
            return self.target_position

    def ball_in_sight(self):
        if self.update_ball() and self.update_target() and self.update_head_yaw():
            bx, by = self.ball_position
            target_x, target_y = self.get_target_position()
            return (norm(bx - target_x, by - target_y) < self.pixel_distance_accuracy
                    and abs(self.head_yaw) < self.body_head_accuracy)
        return False

    def ball_reached(self):
        ball_distance = self.get_ball_distance()
        if ball_distance:
            return ball_distance - self.desired_dist_to_ball < self.ball_distance_accuracy
        return False

    def goal_found(self):
        if self.update_goal():
            return self.goal_detected
        return False

    """ MOTION METHODS """

    def add_motion(self, key, value):
        self.motion[key] += value

    def move(self):
        head_changes = [self.motion['head_yaw'], self.motion['head_pitch']]
        d_body_x = self.motion['body_x']
        d_body_y = self.motion['body_y']
        d_body_yaw = self.motion['body_yaw']

        self.motion_proxy.changeAngles(self.angle_names, head_changes, self.head_fraction_max_speed)
        self.motion_proxy.moveToward(d_body_x, d_body_y, d_body_yaw, [["Frequency", self.body_step_frequency]])

    def search_ball(self):
        t = time.time()
        head_angles = [self.yaw_search * np.sin(self.omega_search * t),
                       self.pitch_search * (1. - np.sin((1. / 3.) * self.omega_search * t))]
        self.motion_proxy.setAngles(self.angle_names, head_angles, self.head_fraction_max_speed)

    def track_ball(self):
        if self.update_ball() and self.update_target():
            target_x, target_y = self.get_target_position()
            d_head_yaw, d_head_pitch = self.changes_from_pixel(target_x, target_y, *self.ball_position)
            self.add_motion('head_yaw', d_head_yaw)
            self.add_motion('head_pitch', d_head_pitch)

    def head_align_body(self):
        if self.update_head_yaw():
            d_yaw_factor = bound(self.body_proportional_constant * self.head_yaw)
            self.add_motion('body_yaw', +d_yaw_factor)
            self.add_motion('head_yaw', -d_yaw_factor * self.body_to_head_factor)

    def reach_ball(self):
        ball_distance = self.get_ball_distance()
        if ball_distance:
            # Compute how far the robot is from the desired ball distance
            d_dist = ball_distance - self.desired_dist_to_ball
            dx_factor = np.sign(d_dist)
            self.add_motion('body_x', dx_factor)

    def search_goal(self):
        self.add_motion('body_y', self.walking_search_factor)
