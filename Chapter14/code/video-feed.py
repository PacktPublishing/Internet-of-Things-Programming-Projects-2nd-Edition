import cv2

stream_url = 'rtsp://10.0.0.98:8554/mjpeg/1'
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open stream")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Can't receive frame (stream end?). Exiting ...")
        break

    cv2.imshow('A.R.E.S. Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
