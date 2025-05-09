from picamera2 import Picamera2, Preview
import cv2
import numpy as np
import time
import pigpio
import threading
import serial
from queue import Queue
from datetime import datetime
from picamera2.encoders import H264Encoder
import os

# =============================================
# Shared Data Queue (Thread-Safe)
# =============================================
data_queue = Queue()

# =============================================
# Serial Reader Thread
# =============================================
def serial_reader(shutdown_event, thread_returned):
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    print("Serial reader thread started")
    try:
        while not shutdown_event.is_set():
            if ser.in_waiting > 0:
                line = ser.readline().decode().strip()
                print(f"Received from Arduino serial reader: {line}")
                data_queue.put(line)  # Send to main thread
                if(line == "CH"):
                    thread_returned.set()
                    return
    except Exception as e:
        print(f"Serial reader error {e}")
    finally:
        print("closing serial port ttyUSB0")
        ser.close()

# =============================================
# Servo Controller Class
# =============================================
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500
SERVO_RANGE = 180

class ServoController:
    def __init__(self, pin_x, pin_y):
        self.pin_x = pin_x
        self.pin_y = pin_y
        self.current_angle_x = -1
        self.current_angle_y = -1
        self.DEADZONE = 5.0  # Degrees

        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")

        self.set_angle(90, 90)  # Center position

    def map_to_pulse(self, angle):
        return SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) // SERVO_RANGE)

    def set_angle(self, angle_x, angle_y):
        # Only move if outside deadzone
        if (abs(angle_x - self.current_angle_x) > self.DEADZONE or 
            abs(angle_y - self.current_angle_y) > self.DEADZONE):

            # Apply constrained angles
            angle_x = max(0.0, min(float(SERVO_RANGE), angle_x))
            angle_y = max(0.0, min(float(SERVO_RANGE), angle_y))

            # Update servos
            if angle_x != self.current_angle_x:
                self.pi.set_servo_pulsewidth(self.pin_x, self.map_to_pulse(angle_x))
                self.current_angle_x = angle_x
            if angle_y != self.current_angle_y:
                self.pi.set_servo_pulsewidth(self.pin_y, self.map_to_pulse(angle_y))
                self.current_angle_y = angle_y

    def update_from_coordinates(self, x, y):
        # Convert screen coordinates to servo angles
        angle_x =self.current_angle_x - ((x - SCREEN_WIDTH/2) * 20) / SCREEN_WIDTH
        angle_y =self.current_angle_y + ((y - SCREEN_HEIGHT/2) * 20) / SCREEN_HEIGHT
        self.set_angle(angle_x, angle_y)

    def move_to(self, x, y):
        angle_x = self.current_angle_x + x
        angle_y = self.current_angle_y + y
        self.set_angle(angle_x, angle_y)

    def cleanup(self):
        self.pi.stop()

# =============================================
# Main Tracking Thread
# =============================================
def object_tracker(shutdown_event, thread_returned):
   # Initialize camera with optimized settings
    picam2 = Picamera2()

    # Camera configuration
    config = picam2.create_preview_configuration(
        main={"size": (640, 480)},
        controls={"FrameRate": 60}
    )
    picam2.configure(config)
    encoder = H264Encoder(10000000)

    # To control video recording
    IS_RECORDING = False
    # Initialize servo controller (GPIO pins for X and Y servos)
    try:
        servos = ServoController(18, 12)
    except RuntimeError as e:
        print(f"Servo initialization failed: {e}")
        exit(1)

    # Red color range in HSV (two ranges for red)
    lower_red1 = np.array([0, 120, 70])    # Lower range for red (0-10)
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])  # Upper range for red (170-180)
    upper_red2 = np.array([180, 255, 255])

    # Jitter reduction parameters
    min_contour_area = 500  # Minimum size to consider as valid object

    # Morphological kernel for noise reduction
    kernel = np.ones((5,5), np.uint8)

    try:
        picam2.start()
        while not shutdown_event.is_set():
            # Process any incoming serial commands
            if not data_queue.empty():
                command = data_queue.get()
                print(f"Processing command from camera: {command}")
                if(command == ">>"):
                    servos.move_to(20, 0)
                elif(command == "<<"):
                    servos.move_to(-20, 0)
                elif(command == "+"):
                    print("moving up")
                    servos.move_to(0, -20)
                elif(command == "–"):
                    print("moving down")
                    servos.move_to(0, 20)
                elif(command == "EQ"):
                    print("moving center")
                    servos.set_angle(90, 90)
                elif(command == "CH"):
                    thread_returned.set()
                    return
                elif((command == "0") and not IS_RECORDING):
                    IS_RECORDING = True
                    filename = f"/home/pi/images/record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"
                    try:
                        picam2.start_recording(encoder, filename)
                    except Exception as ex:
                        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                        message = template.format(type(ex).__name__, ex.args)
                        print(f"in except\n {message}")
                elif(command == "1" and IS_RECORDING):
                    IS_RECORDING = False
                    picam2.stop_recording()
                    picam2.start()
                    print("Recording finished.")

            frame = picam2.capture_array()
            # Convert from RGB to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create masks for blue color (two ranges)
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            # Noise reduction
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the mask
            contours, _ = cv2.findContours(
                mask, 
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)

                # Only proceed if contour is large enough
                if cv2.contourArea(largest_contour) > min_contour_area:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    center_x = x + w // 2
                    center_y = y + h // 2  # Fixed: removed incorrect subtraction
                    print(f"Tracking at: {center_x}, {center_y}")

                    # Update servos with smoothed coordinates
                    servos.update_from_coordinates(center_x, center_y)
            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q') or shutdown_event.is_set():
                break
    finally:
        if IS_RECORDING:
            picam2.stop_recording()
            print("Recording finished.")
        picam2.stop()
        picam2.close()
        servos.cleanup()
        print("closing picam2 and servos")


# =============================================
# Main Program
# =============================================
if __name__ == "__main__":
    shutdown_event = threading.Event()
    thread_returned = threading.Event()
    # Start threads
    serial_thread = threading.Thread(target=serial_reader, args=(shutdown_event,thread_returned,))
    tracker_thread = threading.Thread(target=object_tracker, args=(shutdown_event,thread_returned,))

    serial_thread.start()
    tracker_thread.start()

    # Keep main thread alive
    try:
        while not thread_returned.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        shutdown_event.set()
        serial_thread.join()
        tracker_thread.join()
        if serial_thread.is_alive():
            print("Failed to close the serial thread properly")
        if tracker_thread.is_alive():
            print("Failed to close the tracker thread properly")

