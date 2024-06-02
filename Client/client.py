import cv2
import requests
import time

# Define the URL of the server where the video will be streamed
server_url = "http://192.168.0.102/stream"

# Open a connection to the Raspberry Pi camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the camera resolution (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Function to send a frame to the server
def send_frame(frame):
    _, img_encoded = cv2.imencode('.jpg', frame)
    response = requests.post(server_url, data=img_encoded.tobytes(), headers={'Content-Type': 'image/jpeg'})
    return response

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Send the frame to the server
        response = send_frame(frame)
        if response.status_code != 200:
            print(f"Error: Failed to send frame. Status code: {response.status_code}")
        
        # Display the frame locally (optional)
        cv2.imshow('Video Stream', frame)
        
        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # Sleep for a short while to control frame rate (optional)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Streaming stopped by user.")

finally:
    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()
