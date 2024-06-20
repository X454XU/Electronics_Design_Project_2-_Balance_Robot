import cv2
import numpy as np
import requests
import json
import time

def stream(frame):
    _, img_encoded = cv2.imencode('.jpg', frame)
    response = requests.post(stream_url, data=img_encoded.tobytes(), headers={'Content-Type': 'image/jpeg'})
    
    time.sleep(0.1)
    return response

def auto(command):
    data = {'auto':command}
    data_json = json.dumps(data)
    response = requests.post(auto_url, json = data_json)
    return response
        
def nothing(x):
    pass

def turnLeft():
    print("a")
    auto("a")

def goStraight():
    print("w")
    auto("w")

def turnRight():
    print("d")
    auto("d")

def stop():
    print("idle_up")
    auto("idle_up")

# Define the links for raspberry pi communcation
# #http://10.191.71.116:5000/stream
server_ip = '192.168.43.101:5000'
source_url = 'http://'+server_ip+'/od_video_feed'
stream_url = 'http://'+server_ip+'/stream'
auto_url = 'http://'+server_ip+'/auto'

#flags for movement
left = False
right = False
straight = False


while (True):
    response = requests.get(source_url)
    if response.status_code == 200:
        image_data = response.content
    else:
        raise Exception(f"Failed to retrieve image from {source_url}, status code: {response.status_code}")
    image_array = np.frombuffer(image_data, np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    
    if image is not None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([107, 124, 131])
        upper = np.array([116, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        #make a binary image from mask
        ret2, bin = cv2.threshold(mask, 127, 255, 0)

        #apply open and close morphologies to eliminate inpurities in color detection
        morph_k = 10
        m_open = cv2.morphologyEx(bin, cv2.MORPH_OPEN, np.ones((morph_k, morph_k), np.uint8))


        #find the (second if counting the background) largest connected component
        connect = cv2.connectedComponentsWithStats(m_open, 4, cv2.CV_32S)
        stats = connect[2]
        centroids = connect[3]

        if (len(stats) > 1):
            sorted_i = np.argsort(-stats[:, -1])
            stat = stats[sorted_i[1]]
            (cx, cy) = centroids[sorted_i[1]]
            cv2.rectangle(image, (stat[0], stat[1]), (stat[0] + stat[2], stat[1] + stat[3]), (0, 255, 0), 3)
            cv2.circle(image, (int(cx), int(cy)), 1, (0,255, 0), 3)
            cv2.putText(image, "following", (stat[0], stat[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)
            cv2.putText(image, "center", (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 255, 0), thickness=2)

            #process where to go
            h, w, channels = image.shape

            cv2.line(image, (w//3, 0), (w//3, h), (0, 0, 255), 1)
            cv2.line(image, (2 * w // 3, 0), (2 * w // 3, h), (0, 0, 255), 1)

            if (int(cx) < w//3):
                turnLeft()
            elif (int(cx) > w//3 * 2):
                turnRight()
            else:
                if (stat[1] == 0 or stat[1] + stat[3] == h):
                    stop()
                else:
                    goStraight()


        #show the initial image for debug
        cv2.imshow("image", image)
        
        stream(image)

cv2.destroyAllWindows()