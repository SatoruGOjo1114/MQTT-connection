#!/usr/bin/python3
from capture_usb import Capture
from time import sleep
import cv2
import os
import serial
import Jetson.GPIO as GPIO
from datetime import datetime
import traceback
import sys
import psutil
import json
import time
import shutil
import socket

def permissions():
    try:
        sudoPassword = 'nvidia'
        command = 'chmod 666 /dev/ttyS0'
        os.system('echo %s|sudo -S %s' % (sudoPassword, command))
        sleep(.25)
    except:
        pass

def permissions1(path):
    sudoPassword = 'nvidia'
    command = 'chmod 666 ' + path
    os.system('echo %s|sudo -S %s' % (sudoPassword, command))
    sleep(.25)

def acquire_name(filePath2):
    names = os.listdir(filePath2)
    return len(names) + 1 if names else 1

def get_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(int(f.read()) / 1000.0, 2)
    except:
        return None

def get_cpu_usage():
    return psutil.cpu_percent(interval=0.5)

def check_camera_status(t1):
    return "connected" if t1.cam1_status else "disconnected"

def main():
    permissions()
    permissions1('/dev/ttyUSB0')
    permissions1('/dev/ttyS0')

    ser = serial.Serial("/dev/ttyS0", 115200, timeout=0.01)  # Original UART
    usb_serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)  # To ESP32-C3

    # Send boot message once
    boot_message = {
        "device": "avermedia_jetson",
        "status": "booted",
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    usb_serial.write((json.dumps(boot_message) + "\n").encode())

    filePath2 = "/home/nvidia/Documents/images/"
    t1 = Capture()
    t1.thread()

    pin = 13
    GPIO.cleanup()
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.IN)

    name = 0
    prevCheck = 0
    last_wroom_time = time.time()
    esp32_wroom_status = {"status": "offline"}

    while True:
        try:
            stat = t1.status
            img = t1.img
            ser = serial.Serial("/dev/ttyS0", 115200, timeout=0.01)
            val = GPIO.input(pin)
            val = 1
            print(stat, val, "status  valueeeee")

            if t1.cam1_status:
                ser.write(b"l2\n")
            else:
                ser.write(b"l3\n")

            if stat == 1:
                if val == 1:
                    ser.write(b"r1\n")
                    sleep(.10)
                    ser.write(b"l1\n")
                    sleep(3)

                    if prevCheck == 0 and stat == 1:
                        time_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + ".png"
                        name += 1
                        sleep(1)

            if stat == 0 or val == 0:
                prevCheck = stat
                ser.write(b"r0\n")
                sleep(.10)
                ser.write(b"l0\n")

            prevCheck = stat

            # Read ESP32-WROOM status from UART
            if ser.in_waiting:
                wroom_line = ser.readline().decode().strip()
                if wroom_line:
                    try:
                        esp32_wroom_status = json.loads(wroom_line)
                        esp32_wroom_status["status"] = "alive"
                        last_wroom_time = time.time()
                    except:
                        pass

            # Set WROOM offline if timeout
            if time.time() - last_wroom_time > 10:
                esp32_wroom_status = {"status": "offline"}

            # NEW: Additional system metrics
            try:
                disk = shutil.disk_usage("/")
                disk_percent = round(disk.used / disk.total * 100, 2)
            except:
                disk_percent = None

            try:
                ram_percent = psutil.virtual_memory().percent
            except:
                ram_percent = None

            try:
                uptime_sec = int(time.time() - psutil.boot_time())
            except:
                uptime_sec = None

            try:
                ip_addr = socket.gethostbyname(socket.gethostname())
            except:
                ip_addr = "unknown"

            # FINAL HEARTBEAT MESSAGE
            heartbeat = {
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "device": "avermedia_jetson",
                "camera": check_camera_status(t1),
                "cpu": get_cpu_usage(),
                "temperature": get_cpu_temp(),
                "memory": ram_percent,
                "disk": disk_percent,
                "ip": ip_addr,
                "uptime": uptime_sec,
                "esp32_wroom": esp32_wroom_status
            }

            usb_serial.write((json.dumps(heartbeat) + "\n").encode())
            print("Sent to ESP32-C3:", heartbeat)

            val = 0
            ser.flushOutput()
            ser.flushInput()

            sleep(5)

        except KeyboardInterrupt:
            print("error")
            t1.stop()
            ser.write(b"r0\n")
            sleep(.10)
            ser.write(b"l0\n")
            ser.close()
            usb_serial.close()
            break

        except Exception:
            traceback.print_exception(*sys.exc_info())
            t1.stop()
            ser.write(b"r0\n")
            sleep(.10)
            ser.write(b"l0\n")
            ser.close()
            usb_serial.close()
            break

if __name__ == '__main__':
    main()
