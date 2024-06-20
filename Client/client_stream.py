import cv2
import requests
import time
from picamera2 import Picamera2

# Initialize the camera
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
camera.start()

server_url = 'http://10.191.71.116:5000/source_stream'  # Replace 'Server_IP' with the actual IP of the server

# Allow the camera to warm up
time.sleep(2)

try:
    while True:
        # Capture frame-by-frame
        frame = camera.capture_array()

        # Encode the image as JPEG
        _, img_encoded = cv2.imencode('.jpg', frame)

        # Send the frame to the server
        response = requests.post(server_url, data=img_encoded.tobytes(), headers={'Content-Type': 'application/octet-stream'})

        if response.status_code != 200:
            print(f"Error: {response.status_code}")

except KeyboardInterrupt:
    print("Stream stopped by user")

finally:
    camera.stop()
