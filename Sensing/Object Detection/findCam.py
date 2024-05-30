import cv2

def find_camera_index():
    index = 0
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.read()[0]:
            break
        else:
            print(f"Camera index {index} works")
            cap.release()
        index += 1

find_camera_index()
