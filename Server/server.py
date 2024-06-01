from flask import Flask, request, Response, render_template
import cv2
import numpy as np
import os

app = Flask(__name__)

# Global variable to store the latest frame
latest_frame = None

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

if __name__ == '__main__':
    app.run(debug=True) #host='127.0.0.1', port=5000
