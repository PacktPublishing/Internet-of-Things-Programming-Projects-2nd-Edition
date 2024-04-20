import cv2
import numpy as np

# Load YOLO
net = cv2.dnn.readNet("YOLO/yolov4.weights", "YOLO/yolov4.cfg")
classes = []
layer_names = net.getLayerNames()

# Get output layers using safe indexing
output_layer_indices = net.getUnconnectedOutLayers()

# Load YOLO
net = cv2.dnn.readNet("YOLO/yolov4.weights", "YOLO/yolov4.cfg")
layer_names = net.getLayerNames()
output_layer_indices = net.getUnconnectedOutLayers()

output_layers = [layer_names[i - 1] for i in output_layer_indices]


# Load classes from coco.names
with open("YOLO/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

dog_class_id = classes.index("dog")

# Load image
img = cv2.imread('images/dog.png')
img = cv2.resize(img, None, fx=0.4, fy=0.4)
height, width, channels = img.shape

# Detecting objects
blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Information for each object detected
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5 and class_id == dog_class_id:
            # Object detected
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)

            # Rectangle coordinates
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            boxes.append([x, y, w, h])
            confidences.append(float(confidence))
            class_ids.append(class_id)

# Non-max suppression to eliminate redundant overlapping boxes
indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
for i in indexes.flatten():
    x, y, w, h = boxes[i]
    label = str(classes[class_ids[i]])
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(img, label, (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)

# Display the resulting frame
cv2.imshow("Image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

