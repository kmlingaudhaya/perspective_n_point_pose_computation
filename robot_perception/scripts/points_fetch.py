import cv2
import matplotlib.pyplot as plt

def capture_video_and_take_picture():
    # Open a connection to the camera (0 is usually the default camera)
    cap = cv2.VideoCapture(0)

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Read a single frame from the video feed
        ret, frame = cap.read()

        # Display the video feed
        cv2.imshow("Press Enter to take a picture", frame)

        # Break the loop if the user presses Enter
        if cv2.waitKey(1) == 13:  # 13 is the Enter key
            break

    # Save the last captured frame as an image file
    cv2.imwrite("captured_picture.jpg", frame)

    # Release the camera
    cap.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

    # Open the saved picture using Matplotlib
    img = cv2.imread("captured_picture.jpg")
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Captured Picture")
    plt.axis("off")
    plt.show()

if __name__ == "__main__":
    capture_video_and_take_picture()
