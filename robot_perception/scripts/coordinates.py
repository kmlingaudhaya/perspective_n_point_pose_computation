import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

def onselect(eclick, erelease):
    # eclick and erelease are the press and release events
    x1, y1 = eclick.xdata, eclick.ydata
    x2, y2 = erelease.xdata, erelease.ydata

    print(f"Selected coordinates: ({x1:.2f}, {y1:.2f}) to ({x2:.2f}, {y2:.2f})")

def view_image_with_zoom(image_path):
    # Read the image using OpenCV
    img = cv2.imread(image_path)

    # Convert BGR to RGB (OpenCV uses BGR by default)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Display the image using Matplotlib with zooming capability
    fig, ax = plt.subplots()
    ax.imshow(img_rgb)

    # Enable zooming and panning
    rs = RectangleSelector(ax, onselect, drawtype='box', useblit=True, button=[1],
                           minspanx=5, minspany=5, spancoords='pixels', interactive=True)

    plt.show()

if __name__ == "__main__":
    # Specify the path to your image file
    image_path = "/home/pcboss/robot_perception_ws/captured_picture.jpg"  # Replace with your image file path
    view_image_with_zoom(image_path)
