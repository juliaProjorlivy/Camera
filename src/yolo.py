from ultralytics import YOLO
import cv2
from picamera2 import Picamera2, Preview

if __name__ == "__main__":

    # Load YOLOv8 model
    model = YOLO("yolov8n.pt")  # Replace with your YOLOv8 model file

    # Ask the user to input the target object
    target_object = input("Enter the object you want to find (e.g., 'apple', 'orange'): ").strip().lower()
    print(f"Searching for: {target_object}")

    # Open camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480)})  # Set resolution
    picam2.configure(config)
    picam2.start()

    while True:
        frame1 = picam2.capture_array()
        frame = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)

        resized = frame     #cv2.resize(frame,(600,400))  # НЕ СТОИТ!!! проблемы с матрицей калибровки
        # Perform object detection
        results = model(resized)

        # Process detection results
        for result in results:
            boxes = result.boxes  # Get bounding boxes
            for box in boxes:
                class_id = int(box.cls)  # Class ID of the detected object
                label = model.names[class_id]  # Class label (e.g., "apple", "orange")
                if label == target_object:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    # Calculate the center of the bounding box
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    print(f"Found {target_object} at center: ({center_x}, {center_y})")

                    # Draw bounding box and center point
                    cv2.rectangle(resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(resized, (center_x, center_y), 5, (0, 0, 255), -1)
        # Display the frame
        cv2.imshow("Real-Time Object Detection", resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.stop()
    cv2.destroyAllWindows()
