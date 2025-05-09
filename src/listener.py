#!/home/pi/bin/env python3
import RPi.GPIO as GPIO
import time
import subprocess
import psutil

def check_if_process_running(process_name):
    for process in psutil.process_iter(['name']):
        if process.info['name'] == process_name:
            return True
    return False

GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.IN)
PROCESS_NAME = "/home/pi/camera/arduino.py"

try:
    while True:
        if GPIO.input(16) and (not check_if_process_running(PROCESS_NAME)):
            try:
                print("in try")
                script = subprocess.run(('/home/pi/wake_script.sh'), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                script.check_returncode()
            except subprocess.CalledProcessError as e:
                print("in except")
                print ( "Error:\nreturn code: ", e.returncode, "\nOutput: ", e.stderr.decode("utf-8") )
                print(e.output)
                break
        time.sleep(0.1)
        
except KeyboardInterrupt:
    GPIO.cleanup()
