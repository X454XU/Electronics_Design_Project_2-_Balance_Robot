import spidev
import time
import requests
import json
import RPi.GPIO as GPIO

def init_spi():
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 1350000
    return spi

def read_adc(channel, spi):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def adc_to_voltage(adc_value, v_ref=3.3, resolution=1023):
    return adc_value * (v_ref / resolution)

R1 = 4300000.0  
R2 = 25000.0  
voltage_divider_ratio = R2 / (R1 + R2)

sensing_resistance = 0.01
op_amp_gain = 171
power_supply_voltage = 5.0

spi = init_spi()

server_url = 'http://your_server_ip:5000/update'

with open('voltage_to_charge.json', 'r') as file:
    voltage_to_charge = json.load(file)

def get_current_channel():
    response = requests.get(f'{server_url}/channel')
    if response.status_code == 200:
        return response.json().get('channel', 0)
    return 0

def calculate_power(voltage):
    current = voltage / (sensing_resistance * op_amp_gain)
    power = current * power_supply_voltage
    return power

def find_closest_voltage(voltage, voltage_dict):
    closest_key = min(voltage_dict.keys(), key=lambda k: abs(float(k) - voltage))
    return voltage_dict[closest_key]

try:
    while True:
        current_channel = get_current_channel()

        adc_value_voltage_1 = read_adc(current_channel, spi)
        voltage_1 = adc_to_voltage(adc_value_voltage_1)
        battery_voltage_1 = voltage_1 / voltage_divider_ratio

        charge_subsystem1 = find_closest_voltage(battery_voltage_1, voltage_to_charge)

        adc_value_voltage_2 = read_adc(2, spi)
        voltage_2 = adc_to_voltage(adc_value_voltage_2)
        power_subsystem2 = calculate_power(voltage_2)

        adc_value_voltage_3 = read_adc(3, spi)
        voltage_3 = adc_to_voltage(adc_value_voltage_3)
        power_subsystem3 = calculate_power(voltage_3)
        current_time = time.time()

        data = {
            'timestamp': current_time,
            'charge_subsystem1': charge_subsystem1,
            'power_subsystem2': power_subsystem2,
            'power_subsystem3': power_subsystem3
        }

        response = requests.post(server_url, json=data)
        if response.status_code != 200:
            print('Failed to update data on the server')

        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
    GPIO.cleanup()