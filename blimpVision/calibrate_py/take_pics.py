import cv2
import os

print("Hello World")

cap = cv2.VideoCapture(2)   # Left camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)   # Full image width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)  # Full image height

image_dir = "images"
if not os.path.exists(image_dir):
    os.makedirs(image_dir)

count = 0

while True:
    # Read the concatenated image
    ret, frame = cap.read()

    # Split the concatenated image into left and right frames
    height, width, channels = frame.shape
    left_frame = frame[:, :int(width/2), :]
    right_frame = frame[:, int(width/2):, :]

    # Display the left and right frames
    cv2.imshow("Left Frame", left_frame)
    cv2.imshow("Right Frame", right_frame)

    # Check for spacebar press
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        # Save the left and right frames as lossless PNG images
        cv2.imwrite(os.path.join(image_dir, f"left_{count}.png"), left_frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        cv2.imwrite(os.path.join(image_dir, f"right_{count}.png"), right_frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        print(f"Saved images {count}")
        count += 1
    elif key == ord('q'):
        # Break the loop if the 'q' key is pressed
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
