# ===== LIBRARY IMPORTS =====
import cv2
import numpy as np
from flask import Flask, request, Response
import threading
import time
import math
import heapq

# ===== GLOBAL VARIABLES ======
CAM_INDEX = 1 # 1 FOR OBS, 0 FOR WEBCAM
thr = 0
steer = 0
speed = 0
turn_to = 0
recv_html_X = -6000
recv_html_Y = -6000
recv_html_bearing = 0
path = []

# ===== INTERSECTIONS WITH COORDS =====
nodes = {
    "1": [-6000,-6000], "2": [-5844,-6006], "3": [-5729,-5920], "4": [-5225,-6062],
    "5": [-5129,-5428], "6": [-5043,-4238], "7": [-5215,-3936], "8": [-5669,-3373],
    "9": [-4846,-4036], "10": [-3088,-4793], "11": [-791,-5240], "12": [-198,-4668],
    "13": [1134,-3754], "14": [1891,-4501], "15": [2697,-4939], "16": [3361,-4910],
    "17": [2924,-5320], "18": [2059,-6352], "19": [2488,-8295], "20": [2634,-8383],
    "21": [2514,-8435], "22": [-2299,-10154], "23": [-3379,-6635], "24": [-3617,-5912],
    "25": [-4011,-6479], "26": [-7924,-11563], "27": [6888,-9724], "28": [-838,-5646],
    "29": [3705,-5364]
}


# ===== ROADS ======
edges = {
    1: [1,2], 2: [2,3], 3: [3,4], 4: [4,5], 5: [5,3], 6: [5,6], 7: [6,7], 8: [7,9],
    9: [9,6], 10: [7,8], 11: [9,10], 12: [10,24], 13: [24,23], 14: [23,25], 15: [25,24],
    16: [4,25], 17: [23,22], 18: [22,26], 19: [10,11], 20: [11,28], 21: [11,12],
    22: [12,13], 23: [12,18], 24: [13,14], 25: [14,15], 26: [15,16], 27: [15,17],
    28: [16,17], 29: [17,18], 30: [18,19], 31: [19,20], 32: [20,21], 33: [21,19],
    34: [20,27], 35: [21,22], 36: [22,23], 37: [22,26], 38: [16,29]
}

def router():
    global turn_to

    # Flag to check if we are inside any node
    in_node = False

    for idx, (name, coords) in enumerate(nodes.items()):
        if all(abs(a - b) <= 20 for a, b in zip(coords, [recv_html_X, recv_html_Y])):
            in_node = True

            if name in path:
                current_index = path.index(name)
                if current_index < len(path) - 1:
                    next_node = path[current_index + 1]
                    next_coords = nodes[next_node]

                    dx = next_coords[0] - coords[0]
                    dy = next_coords[1] - coords[1]

                    target_angle = math.degrees(math.atan2(dy, dx)) % 360

                    # Difference clockwise and anticlockwise
                    diff = (target_angle - recv_html_bearing) % 360

                    if diff < 180:
                        turn_to = 1
                        
                    else:
                        turn_to = -1  # anticlockwise

                    print(f"At node {name}. Next: {next_node}, Target angle: {target_angle:.2f}, Bearing: {recv_html_bearing}, Turn: {turn_to}")
                else:
                    turn_to = 0  # At final node, no turning needed
            else:
                turn_to = 0  # Node not in path, nothing to do

            break  # No need to check further nodes

    if not in_node:
        turn_to = 0

#-------------------------------------------------------------------------------------------------------------------------

# === HTML SERVER FOR BULLET VENV CONNECTIVITY ===
app = Flask(__name__)

@app.route('/numbers', methods=['GET'])
def handle_numbers():
    global thr, steer, speed

    # = DECODE RECV FROM SW = 
    recv_html_X = int(request.args.get('num1', 0))
    recv_html_Y = int(request.args.get('num2', 0))  
    recv_html_bearing = int(request.args.get('num3', 0))
    
    print(f"Received: X coord={recv_html_X}, Y coord={recv_html_Y}, bearing={recv_html_bearing}")

    #  = ENCODE SEND TO SW =

    print(f"Returned: thr={4}, steer={steer}, bearing={recv_html_bearing}")

    # SEND TXT TO SW, MAKE SURE URL IS CORRECT
    return Response(f"{4},{steer},{recv_html_bearing}", mimetype='text/plain')

#-------------------------------------------------------------------------------------------------------------------------

def camera_loop():
    
    global thr, steer, speed, turn_to
    # == CAM INITIALIZE ==
    
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        h, w = frame.shape[:2] # RETURNS ONLY 2 VALUES, by default returns 3 values

        # === BOUNDING BOX (TRAPEZOIDAL SHAPED MASK) ===
        trapezoid_mask = np.zeros((h, w), dtype=np.uint8)
        pts = np.array([
            [w // 2 - w // 3, int(h * 0.8)],
            [w // 2 + w // 3, int(h * 0.8)],
            [w // 2 + w // 8, int(h / 4)],
            [w // 2 - w // 8, int(h / 4)]
        ], dtype=np.int32)
        cv2.fillPoly(trapezoid_mask, [pts], 255)

        # === HSV MASK OF ROAD ===
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 60, 70])

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #CONVERT to HSV BECAUSE WHY NOT
        mask_dark = cv2.inRange(hsv_frame, lower_dark, upper_dark) #FIND ROAD USING DEFINED MASK PARAMETERS
        final_mask = cv2.bitwise_and(mask_dark, trapezoid_mask) #COMBINE MASK TO REDUCE MASK AREA

        # === CONTOUR MAKING ===
        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR)
        cv2.polylines(output, [pts], isClosed=True, color=(255, 0, 0), thickness=2) # TRAPEZOID DRAW

        cx_whole = cy_whole = cx_left = cy_left = cx_right = cy_right = None

        if contours:
            largest = max(contours, key=cv2.contourArea) #FIND LARGEST CONTOUR
            cv2.drawContours(output, [largest], -1, (0, 255, 0), 2) #DRAW TO SCREEN

            M = cv2.moments(largest) #LARGEST CONTOUR CONVERTED TO MOMENT
            if M["m00"] != 0: 
                cx_whole = int(M["m10"] / M["m00"]) #SUMMATION OF X POINTS / AREA
                cy_whole = int(M["m01"] / M["m00"]) #SUMMATION OF Y POINTS / AREA
                cv2.circle(output, (cx_whole, cy_whole), 5, (0, 255, 0), -1)

                # Draw centroid line
                cv2.line(output, (cx_whole, 0), (cx_whole, h), (0, 0, 255), 2)

                # === Split mask by centroid line ===
                left_mask = np.zeros_like(final_mask)
                right_mask = np.zeros_like(final_mask)

                left_mask[:, :cx_whole] = final_mask[:, :cx_whole]
                right_mask[:, cx_whole:] = final_mask[:, cx_whole:]

                # Find left contour
                contours_left, _ = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours_left:
                    largest_left = max(contours_left, key=cv2.contourArea)
                    M_left = cv2.moments(largest_left)
                    if M_left["m00"] != 0:
                        cx_left = int(M_left["m10"] / M_left["m00"])
                        cy_left = int(M_left["m01"] / M_left["m00"])
                        cv2.circle(output, (cx_left, cy_left), 5, (255, 0, 0), -1)

                # Find right contour
                contours_right, _ = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours_right:
                    largest_right = max(contours_right, key=cv2.contourArea)
                    M_right = cv2.moments(largest_right)
                    if M_right["m00"] != 0:
                        cx_right = int(M_right["m10"] / M_right["m00"])
                        cy_right = int(M_right["m01"] / M_right["m00"])
                        cv2.circle(output, (cx_right, cy_right), 5, (0, 0, 255), -1)

                steer_value = 0

                if turn_to == 1 and cx_right is not None:
                    steer_value = (cx_right - (w // 2)) * 0.75
                    cv2.circle(output, (cx_right, cy_right), 15, (0, 0, 255), -1)
                elif turn_to == -1 and cx_left is not None:
                    steer_value = (cx_left - (w // 2)) * 0.75
                    cv2.circle(output, (cx_left, cy_left), 15, (255, 0, 0), -1)
                elif cx_whole is not None:
                    steer_value = (cx_whole - (w // 2)) * 0.75
                    cv2.circle(output, (cx_whole, cy_whole), 15, (0,255,0),-1_
                steer = steer_value
                print(steer)
                
        cv2.imshow("Frame", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=8080)
