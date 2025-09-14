import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

# Unlock configuration
ser.write(bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5]))
time.sleep(0.1)

# Calibrate accelerometer (keep sensor still)
print("Calibrating accelerometer - keep sensor STILL for 3 seconds...")
ser.write(bytes([0xFF, 0xAA, 0x01, 0x01, 0x00]))
time.sleep(3)

# Calibrate gyroscope (keep sensor still)  
print("Calibrating gyroscope - keep sensor STILL for 3 seconds...")
ser.write(bytes([0xFF, 0xAA, 0x01, 0x02, 0x00]))
time.sleep(3)

print("Calibration complete! Now test your movements.")
ser.close()