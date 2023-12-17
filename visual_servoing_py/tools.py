import cv2
import numpy as np


def contour_center(contour):  # list of points (x, y)
    return np.mean(contour[:, :, 0]), np.mean(contour[:, :, 1])


def detect_ball(image, real=False):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV range for yellow
    if real:
        lower_yellow = np.array([15, 100, 32])  # real
        upper_yellow = np.array([27, 255, 255])  # real
    else:
        lower_yellow = np.array([20, 100, 100])  # sim
        upper_yellow = np.array([30, 255, 255])  # sim

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

        # Ball radius estimation via ball area
        area = cv2.contourArea(ball_contour)
        r_ball = np.sqrt(area / np.pi)

        cx, cy = contour_center(ball_contour)

        return r_ball, cx, cy  # return distance_to_ball, x & y components of the ball's center in pixels
    else:
        return None, None, None


def detect_goal(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV range for red
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Create a mask to isolate red regions
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    lst_cx, lst_cy = [], []
    if len(contours) >= 2:
        lst_centers = [contour_center(contour) for contour in contours]
        lst_cx, lst_cy = zip(*lst_centers)


        max_cx = max(lst_cx)
        min_cx = min(lst_cx)
        if max_cx - min_cx > 100:
            c1 = lst_cx.index(max_cx)
            c2 = lst_cx.index(min_cx)
            ptx = abs(lst_cx[c1] + lst_cx[c2]) / 2
            pty = abs(lst_cy[c1] + lst_cy[c2]) / 2
            return True, pty, ptx
    return False, None, None


def bound(value, ceil=1.0):
    return max(-ceil, min(value, +ceil))


def norm(x, y):
    return np.sqrt(x * x + y * y)
