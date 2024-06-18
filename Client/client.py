import cv2
import requests
import threading
import time

import json

import RPi.GPIO as GPIO
import serial

"""
TX (Transmit): GPIO 14 (Physical Pin 8)
RX (Receive): GPIO 15 (Physical Pin 10)
"""

speed = 0.0

GPIO.setmode(GPIO.BCM)
ser = serial.Serial('/dev/ttyS0', 9600)

  
def get_key():
    response = requests.get(server_key_url)
    if response.status_code == 200:
        key = response.json()
        return key['key']
 
def send_key(key):
    # Send the received key to UART
    if key in ['w', 'a', 's', 'd']:
        ser.write(key.encode())  # Send the key over UART
        print(f"Sent {key} over UART")

def uart_com():
    global speed
    try:
        while True:
            send_key(get_key())
            if ser.in_waiting > 0:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                try:
                    # Convert the line to a float
                    number = float(line)
                    speed = number
                    print(f"Received speed: {number}")
                except ValueError:
                    print(f"Received non-float data: {line}")
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

  


# Define the URL of the server where the video will be streamed
server_ip = "10.191.71.116:5000"
server_url = "http://" + server_ip + "/stream"
server_key_url = "http://" + server_ip + "/get_key"
server_speed_url = "http://" + server_ip + "/post_speed"
# Open a connection to the Raspberry Pi camera
cap = cv2.VideoCapture(2)

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

def send_speed(speed):
    data = {'speed':speed}
    data_json = json.dumps(data)
    response = requests.post(server_speed_url, json = data_json)

if __name__ == "__main__":
    
    # Create a thread for the UART reading function
    uart_thread = threading.Thread(target=uart_com)
    uart_thread.daemon = True  # Set as a daemon so it will exit when the main program exits

    # Start the UART thread
    uart_thread.start()
    
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
            
            print(get_key())
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
