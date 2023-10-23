import cv2
import numpy as np

def get_ball_center(img_path):
    img = cv2.imread(img_path)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definir la plage de couleurs pour la balle jaune (dans l'espace HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Creer un masque pour isoler la balle jaune
    mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

    # Trouver les contours dans le masque
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Trouver le plus grand contour (la balle jaune)
        largest_contour = max(contours, key=cv2.contourArea)

    # Trouver le centre de la balle jaune
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        print("Position de la balle jaune : x =", center_x, "y =", center_y)

    # Dessiner un cercle autour de la balle jaune
    cv2.drawContours(img, [largest_contour], -1, (0, 255, 0), 2)

    # Afficher l'image avec le cercle
    cv2.imshow('Image avec la balle jaune', img)
    while True:
        cv2.waitKey(1)


if __name__ == "__main__":
    get_ball_center("../../imgs/naorealimgs/naoreal_0000.png")
    