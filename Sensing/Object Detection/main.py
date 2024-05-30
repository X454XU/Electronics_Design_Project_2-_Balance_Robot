import torch
import cv2
import numpy as np

global model
global cap


def getPosition(results):
    predictions = results.pred[0]   #[x1, y1, x2, y2, confidence, class]
    '''
    x1, y1: Coordinates of the top-left corner of the bounding box.
    x2, y2: Coordinates of the bottom-right corner of the bounding box.
    confidence: Confidence score for the detection (ranging from 0 to 1).
    class: Class index of the detected object.
    '''
    for *box, conf, cls in predictions:
        x1, y1, x2, y2 = box  # Bounding box coordinates
        confidence = conf.item()  # Confidence score
        class_id = int(cls.item())  # Class ID
        print(f"Bounding Box: [{x1}, {y1}, {x2}, {y2}], Confidence: {confidence}, Class ID: {class_id}")

def run():
    model = torch.hub.load('ultralytics/yolov5', 'yolov5x', force_reload=True)
    cap= cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()

        # Make detections
        results = model(frame)
        #print(results)
        getPosition(results)
        cv2.imshow('YOLO', np.squeeze(results.render()))

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    run()