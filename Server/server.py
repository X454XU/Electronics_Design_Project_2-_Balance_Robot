from flask import Flask, request, Response, render_template, jsonify
import cv2
import numpy as np


app = Flask(__name__)


# Global variable to store the latest frame
latest_frame = None
keypress_log = []
speed = 0.0

@app.route('/post_speed', methods = ['POST'])
def store_speed():
    global  speed
    data = request.json.get('speed')
    if data:
        speed = data
        print(speed)
        # For simplicity, returning the last key pressed
        return jsonify({'speed': speed})
    return jsonify({'error': 'No key provided'}), 400

@app.route('/get_speed', methods=['GET'])
def get_speed():
    global speed
    data = {'speed': speed}
    return jsonify(data)

@app.route('/stream', methods=['POST'])
def stream():
    global latest_frame
    nparr = np.frombuffer(request.data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is not None:
        latest_frame = img
    return 'Frame received', 200

@app.route('/video_feed')
def video_feed():
    global latest_frame
    if latest_frame is not None:
        _, buffer = cv2.imencode('.jpg', latest_frame)
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    return 'No frame available', 404

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/log_keypress', methods=['POST'])
def log_keypress():
    key = request.json.get('key')
    if key:
        keypress_log.append(key)
        print(keypress_log)
        # For simplicity, returning the last key pressed
        return jsonify({'key': key})
    return jsonify({'error': 'No key provided'}), 400

@app.route('/get_key')
def send_key():
    global keypress_log
    if keypress_log is not None:
        return jsonify({'key':keypress_log[-1]})

@app.route('/command', methods=['POST'])
def receive_command():
    command = request.json.get('command')
    if command:
        # Process the command (e.g., moveForward, moveBackward, etc.)
        response = process_command(command)
        return jsonify({'response': response})
    return jsonify({'error': 'No command provided'}), 400

def process_command(command):
    # Implement command processing logic here
    if command == 'forward':
        # Move forward logic
        return 'Moving forward'
    elif command == 'backward':
        # Move backward logic
        return 'Moving backward'
    # Add other commands as needed
    return 'Unknown command'


if __name__ == '__main__':
    app.run(host = '10.191.71.116', port = 5000, debug=True) #host='127.0.0.1', port=5000

