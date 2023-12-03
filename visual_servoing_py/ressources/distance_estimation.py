import cv2
from fusion import detect_ball


if __name__ == '__main__':
    # Open the default camera (usually the built-in laptop camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Check if the frame was read successfully
        if not ret:
            print("Error: Could not read frame.")
            break

        ball_detected, bx, by = detect_ball(frame)

        if ball_detected:
            cv2.circle(frame, (bx, by), 5, (0, 0, 255), -1)

        # Display the frame
        cv2.imshow('Camera Feed', frame)

        # Exit the loop when the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
