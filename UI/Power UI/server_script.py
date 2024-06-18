from flask import Flask, request, jsonify, render_template
import time

app = Flask(__name__)

# Store the latest battery data and the current ADC channel
battery_data = {
    'timestamp': None,
    'charge_subsystem1': None,
    'power_subsystem2': None,
    'power_subsystem3': None
}
adc_channel = 0  # Default ADC channel

@app.route('/update', methods=['POST'])
def update_data():
    global battery_data
    data = request.json
    battery_data['timestamp'] = data.get('timestamp')
    battery_data['charge_subsystem1'] = data.get('charge_subsystem1')
    battery_data['power_subsystem2'] = data.get('power_subsystem2')
    battery_data['power_subsystem3'] = data.get('power_subsystem3')
    return jsonify({'status': 'success'})

@app.route('/channel', methods=['POST'])
def update_channel():
    global adc_channel
    data = request.json
    adc_channel = data.get('channel', adc_channel)
    return jsonify({'status': 'success', 'channel': adc_channel})

@app.route('/channel', methods=['GET'])
def get_channel():
    return jsonify({'channel': adc_channel})

@app.route('/data', methods=['GET'])
def get_data():
    return jsonify(battery_data)

@app.route('/')
def display_data():
    return render_template('display.html', data=battery_data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)