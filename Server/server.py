from flask import Flask, request, Response, render_template, jsonify
import cv2
import numpy as np


app = Flask(__name__)


# Global variable to store the latest frame
latest_frame = None
keypress_log = ["w_up"]
auto_log = ["w_up"]
auto = 0
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

@app.route('/log_keyrelease', methods=['POST'])
def log_keyrelease():
    key = request.json.get('key')
    if key:
        keypress_log.append(key + "_up")
        print(keypress_log)
        # For simplicity, returning the last key pressed
        return jsonify({'key': key})
    return jsonify({'error': 'No key provided'}), 400

@app.route('/get_key')
def send_key():
    global keypress_log
    if keypress_log is not None:
        return jsonify({'key':keypress_log[-1]}), 200

@app.route('/auto', methods=['POST'])
def auto():
    auto_command = request.json.get('auto')
    if auto_command:
        auto_log.append(auto_command)
        print(auto_log)
        # For simplicity, returning the last key pressed
        return jsonify({'auto': auto_command})
    return jsonify({'error': 'No key provided'}), 400

@app.route('/command', methods=['GET'])
def send_command():
    global auto
    if auto:
        command = auto_log[-1]
    else:
        command = keypress_log[-1]
    if command:
        # Process the command (e.g., moveForward, moveBackward, etc.)
        command_o = process_command(command)
        return jsonify({'command': command_o}), 200
    return jsonify({'error': 'No command provided'}), 400

@app.route('/command_html', methods=['GET'])
def send_command_html():
    global auto
    if auto:
        command = auto_log[-1]
        if command:
            # Process the command (e.g., moveForward, moveBackward, etc.)
            command_o = process_command(command)
            return jsonify({'command': 'auto: '+ command_o}), 200
    else:
        command = keypress_log[-1]
        if command:
            # Process the command (e.g., moveForward, moveBackward, etc.)
            command_o = process_command(command)
            return jsonify({'command': 'manual: ' + command_o}), 200
    
    return jsonify({'error': 'No command provided'}), 400

def process_command(keypress):
    # Implement command processing logic here
    if '_up' in keypress or keypress == 'i':
        # Move forward logic
        return 'idle'
    elif keypress == 'w':
        # Move backward logic
        return 'forward'
    elif keypress == 's':
        # Move backward logic
        return 'backward'
    elif keypress == 'a':
        # Move backward logic
        return 'left'
    elif keypress == 'd':
        # Move backward logic
        return 'right'
    # Add other commands as needed
    return 'Unknown command'

@app.route('/mode', methods = ['POST'])
def set_mode():
    global auto
    mode = request.json.get('mode')
    if mode == 'auto':
        auto = 1
        print(auto)
        return jsonify({'mode': 'auto'})
    elif mode == 'manual':
        auto = 0
        print(auto)
        return jsonify({'mode': 'manual'})
        
    return jsonify({'error': 'No mode provided'}), 400
    
if __name__ == '__main__':
    app.run(host = '10.191.71.116', port = 5000, debug=True) #host='127.0.0.1', port=5000

