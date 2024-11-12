import serial
import time

# Set up the serial port (adjust the port name to match your system)
ser = serial.Serial('COM4', 9600)  # Windows, replace 'COM3' with your port name
# For Linux or macOS, the port might be '/dev/ttyUSB0' or '/dev/ttyACM0'

time.sleep(2)  # Wait for Arduino to reset

def send_command(command):
    ser.write(command.encode())  # Send the command to Arduino
    print(f"Sent command: {command}")

# Example usage
while True:
    command = input("Enter command (a for clockwise, b for counterclockwise, q to quit): ").strip().lower()
    
    if command == 'a' or command == 'b':
        send_command(command)
    elif command == 'q':
        print("Exiting...")
        break
    else:
        print("Invalid command. Please enter 'a', 'b', or 'q'.")
    
    time.sleep(1)  # Optional delay to prevent sending commands too quickly

ser.close()  # Close the serial connection
