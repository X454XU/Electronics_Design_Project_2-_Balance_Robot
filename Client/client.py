import cv2
import requests
import time

import websocket
import json
import threading

import RPi.GPIO as GPIO
import serial

"""
TX (Transmit): GPIO 14 (Physical Pin 8)
RX (Receive): GPIO 15 (Physical Pin 10)
"""

GPIO.setmode(GPIO.BCM)
ser = serial.Serial('/dev/ttyS0', 9600)

def on_message(ws, message):
    try:
        data = json.loads(message)
        if 'key' in data:
            key = data['key']
            send_uart(key)
            print(f"Key pressed: {data['key']}")
        else:
            print(f"Unexpected message: {data}")
    except json.JSONDecodeError as e:
        print(f"Failed to decode JSON: {e}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws):
    print("Connection closed")

def on_open(ws):
    print("Connection opened")
    
def send_uart(key):
    # Send the received key to UART
    if key in ['w', 'a', 's', 'd']:
        ser.write(key.encode())  # Send the key over UART
        print(f"Sent {key} over UART")
        


# Define the URL of the server where the video will be streamed
server_ip = "192.168.0.102"
server_url = "http://" + server_ip + "/stream"

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

def run_websocket():
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://"+server_ip+":5000/socket.io/?EIO=4&transport=websocket",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.run_forever()

if __name__ == "__main__":
    ws_thread = threading.Thread(target=run_websocket)
    ws_thread.start()

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
        GPIO.cleanup()  # Clean up GPIO settings
        ser.close()     # Close UART connection
