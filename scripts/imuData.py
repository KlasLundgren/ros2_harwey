import serial
import struct
import time

def parse_jy901b_stream(ser):
    buffer = b''
    
    while True:
        # Read available data and add to buffer
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)
        
        # Look for complete packets (starting with 0x55)
        while len(buffer) >= 11:
            # Find start of packet
            start_idx = buffer.find(0x55)
            if start_idx == -1:
                buffer = b''
                break
            
            # Remove data before packet start
            if start_idx > 0:
                buffer = buffer[start_idx:]
            
            # Check if we have a complete packet
            if len(buffer) >= 11:
                packet = buffer[:11]
                buffer = buffer[11:]  # Remove processed packet
                
                # Parse the packet
                if packet[0] == 0x55:
                    data_type = packet[1]
                    values = struct.unpack('<hhh', packet[2:8])
                    
                    if data_type == 0x51:  # Acceleration
                        ax, ay, az = [v/32768.0*16 for v in values]
                        print(f"Accel: X={ax:.3f}g, Y={ay:.3f}g, Z={az:.3f}g")
                        
                    elif data_type == 0x52:  # Angular velocity
                        gx, gy, gz = [v/32768.0*2000 for v in values]
                        print(f"Gyro: X={gx:.1f}°/s, Y={gy:.1f}°/s, Z={gz:.1f}°/s")
                        
                    elif data_type == 0x53:  # Angle (this is what you want!)
                        roll, pitch, yaw = [v/32768.0*180 for v in values]
                        print(f"ANGLE: Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°")
                        
                    elif data_type == 0x54:  # Magnetic field
                        mx, my, mz = values
                        print(f"Mag: X={mx}, Y={my}, Z={mz}")
            else:
                break
        
        time.sleep(0.1)

# Connect with correct port
ser = serial.Serial('/dev/ttyUSB2', 9600, timeout=1)  # Note: ttyUSB2!
print("Parsing JY901B data stream... Move the sensor to test!")

try:
    parse_jy901b_stream(ser)
except KeyboardInterrupt:
    print("\nStopping...")
    ser.close()