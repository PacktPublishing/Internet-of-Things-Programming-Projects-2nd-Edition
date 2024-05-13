import cv2
import numpy as np

class DogDetector:
    def __init__(self, model_weights, model_cfg, class_file):
        # Load YOLO model
        self.net = cv2.dnn.readNet(model_weights, model_cfg)
        self.layer_names = self.net.getLayerNames()

        # Safely handle the output layers indices depending on dimensionality
        output_layer_indices = self.net.getUnconnectedOutLayers()
        if output_layer_indices.ndim == 1:
            self.output_layers = [self.layer_names[i - 1] for i in output_layer_indices]
        else:
            self.output_layers = [self.layer_names[i[0] - 1] for i in output_layer_indices]

        # Load class names
        with open(class_file, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.dog_class_id = self.classes.index("dog")

    def detect_dogs(self, frame):
        img_resized = cv2.resize(frame, None, fx=0.4, fy=0.4)
        height, width, channels = img_resized.shape
        dog_detected = False

        blob = cv2.dnn.blobFromImage(img_resized, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == self.dog_class_id:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        
        if indexes is not None and len(indexes) > 0:
            dog_detected = True
            # If indexes are valid, flatten them to handle in the loop
            indexes = indexes.flatten()

            for i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                cv2.rectangle(img_resized, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img_resized, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return img_resized, dog_detected

