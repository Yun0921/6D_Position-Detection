import serial
import time
import numpy as np
import datetime
import logging

# set GY-25 serial port
gy25Serial = serial.Serial('/dev/ttyS0', 115200)  # port = /dev/ttyS0

Roll = 0.0 
Pitch = 0.0
Yaw = 0.0
buffer = [0] * 8
counter = 0
imu_updated = False

current_time = datetime.datetime.now().strftime("%m-%d %H:%M")
log_filename = f"{current_time}.log"

# set logging, used for output to screen and write to file
logging.basicConfig(
    level=logging.INFO, 
    format="%(message)s",  
    handlers=[
        logging.StreamHandler(),  # show on screen
        logging.FileHandler("log_filename")  # write into file
    ]
)

# define GY-25 commands
CMD_QUERY_MODE = [0xA5, 0x51]
CMD_AUTO_MODE = [0xA5, 0x52]
CMD_CORRECT_MODE = [0xA5, 0x55]

# send initial commands to GY-25
def setup():
    global gy25Serial
    print("--- Start setting GY-25 ---")
    
    # send correct mode command to initialize GY-25
    gy25Serial.write(bytearray(CMD_CORRECT_MODE))
    print("--- Calibrating GY-25, need to stay level ---")
    time.sleep(4)
    
    # send auto mode command to get continuous angle data
    gy25Serial.write(bytearray(CMD_AUTO_MODE))
    print("--- End setting GY-25 ---")

#
def toInt16(raw):
    # check the sign bit
    if raw & 0x8000:
        # if it is negative, convert to negative number using two's complement
        raw = (raw - 0x10000) / 100.0
    else:
        raw = raw / 100.0

    return raw

# read and deal with sensor data
def read_sensor_data():
    global buffer, counter, imu_updated, Roll, Pitch, Yaw
    
    while gy25Serial.in_waiting > 0:  
        byte = gy25Serial.read(1)  
        buffer[counter] = ord(byte) 
        
        if counter == 0 and buffer[0] != 0xAA:
            return  
        
        counter += 1
        
        if counter == 8: 
            counter = 0
            if buffer[0] == 0xAA and buffer[7] == 0x55:  
                Yaw = toInt16(buffer[1] << 8 | buffer[2])
                Pitch = toInt16(buffer[3] << 8 | buffer[4])
                Roll = toInt16(buffer[5] << 8 | buffer[6])
                imu_updated = True

def get_current_time_milliseconds_no_date():
    # get current time to milliseconds
    current_time = datetime.datetime.now()
    seconds = current_time.strftime("%Y-%m-%d %H:%M:%S")
    milliseconds = current_time.microsecond / 10000.0
    return seconds, round(milliseconds, 0)

# main loop
def loop():
    global imu_updated, Roll, Pitch, Yaw
    
    while True:
        current_time, current_time_ms = get_current_time_milliseconds_no_date()
        if (current_time_ms + 1) % 5 == 0:
            read_sensor_data()
            if imu_updated:
                imu_updated = False
                log_message = f"{current_time}.{current_time_ms:02.0f}: roll= {Roll:3.0f},   pitch= {Pitch:3.0f},   yaw= {Yaw:3.0f}"
                logging.info(log_message)  # both show on screen and write into file
                
            time.sleep(0.01)  # delay 10ms

if __name__ == "__main__":
    setup()
    loop()
