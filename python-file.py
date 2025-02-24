import serial
import time

# Set up the serial connection (adjust the port as needed)
port = '/dev/cu.usbmodem1101'
ser = serial.Serial(port, baudrate=9600, timeout=1)

def getValues():
    # Ask the Arduino for any response data
    ser.write(b'g')  # This sends a 'get' command; adjust as needed
    arduinoData = ser.readline().decode('ascii').strip()
    return arduinoData

while True:
    userInput = input('Type in runtime (in minutes) and RPM, of the form MIN:RPM: ')
    try:
        # Split the input string into runtime and RPM
        runtime_min, rpm = userInput.split(':')
        runtime_sec = int(runtime_min) * 60  # Convert minutes to seconds
        rpm = int(rpm)
    except Exception as e:
        print("Invalid input. Please use the format MIN:RPM (e.g., 10:1500).")
        continue

    # Build the command string expected by the Arduino
    command = f"RPM:{rpm};TIME:{runtime_sec}\n"
    ser.write(command.encode('ascii'))
    print("Sent command:", command.strip())
    
    # Give the Arduino a moment to process the command
    time.sleep(0.1)