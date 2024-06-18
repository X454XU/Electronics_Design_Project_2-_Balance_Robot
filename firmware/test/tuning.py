import time
import re
import subprocess
import os

# Define the path to your main.cpp file relative to the location of the Python script
main_cpp_path = os.path.join(os.path.dirname(__file__), '../src/main.cpp')

# Define the initial Kd value
initial_kd = 10
kd_increment = 5

# Define the number of increments
num_increments = 10

# Function to modify the Kd value in main.cpp
def modify_kd(kd_value):
    with open(main_cpp_path, 'r') as file:
        lines = file.readlines()
    
    with open(main_cpp_path, 'w') as file:
        for line in lines:
            if 'double kd =' in line:
                line = re.sub(r'double kd = \d+\.?\d*;', f'double kd = {kd_value};', line)
            file.write(line)

# Function to upload the code to the robot using PlatformIO
def upload_code():
    project_root = os.path.join(os.path.dirname(__file__), '../')
    result = subprocess.run(['platformio', 'run', '--target', 'upload'], cwd=project_root, capture_output=True)
    if result.returncode == 0:
        print("Upload successful!")
    else:
        print(f"Upload failed: {result.stderr.decode()}")

# Main loop to increment Kd and upload the code
kd_value = initial_kd
for i in range(num_increments):
    kd_value += kd_increment
    print(f"Setting Kd to {kd_value}")
    modify_kd(kd_value)
    upload_code()
    time.sleep(10)
