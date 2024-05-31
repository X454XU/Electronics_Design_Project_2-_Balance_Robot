import serial
import csv
import time

# Serial port settings
serial_port = '/dev/tty.usbserial-120'  # Replace with your port name, e.g., /dev/ttyUSB0 on Linux
baud_rate = 115200
timeout = 1

# Open serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=timeout)

# File names
csv_file_name = 'output.csv'

# Open CSV file
with open(csv_file_name, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)

    # Write CSV header
    csv_writer.writerow(['Timestamp', 'Tilt (mrad)', 'Tilt (deg)', 'Step1 Speed', 'Step2 Speed', 'Setpoint'])

    try:
        while True:
            # Read line from serial port
            line = ser.readline().decode('utf-8').strip()

            if line:
                # Parse the line (assuming CSV format sent from Arduino)
                data = line.split(',')

                # Add a timestamp
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                data.insert(0, timestamp)

                # Write data to CSV file
                csv_writer.writerow(data)

                # Print to console (optional)
                print(data)

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        ser.close()

print(f"Data written to {csv_file_name}")
