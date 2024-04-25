import cv2
from DogDetector import DogDetector
from TwilioMessage import TwilioMessage
import time

# Initialize the detector and Twilio message service
detector = DogDetector("YOLO/yolov4.weights", "YOLO/yolov4.cfg", "YOLO/coco.names")
twilio_message = TwilioMessage('-------------------', '--------------', '+0-----------')

# Open a video stream or camera
stream_url = 'rtsp://10.0.0.98:8554/mjpeg/1'
cap = cv2.VideoCapture(stream_url)

# Create a window that can be resized
cv2.namedWindow("Dog Detector", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Dog Detector", 800, 600)  # Adjust window size as needed

last_time = time.time()  # Keep track of the last frame time

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Check if 1 second has passed
        current_time = time.time()
        if current_time - last_time >= 1.0:  # 1.0 seconds
            # Detect dogs in the frame
            result_frame, dog_detected = detector.detect_dogs(frame)
            last_time = current_time

            # Display the result
            cv2.imshow("Dog Detector", result_frame)
            if dog_detected:
                twilio_message.send_sms('+-------------', 'Dog(s) detected!')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
