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
recv_html_X = 0
recv_html_Y = 0
recv_html_bearing = 0
path = ["3","4","5","3","2","1"]

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

# === SHORTEST PATH FINDING ALGORITHM ===

def find_shortest_path(start_x, start_y, target_node):
    global path
    path = []  # Reset the global path list
    
    # Convert target_node to string
    # Allows us to compare with keys of nodes dictionary
    
    target_node = str(target_node)
    
    # Convert edge list to adjacency list with weights (Euclidean distances)
    adj_list = {str(i): [] for i in range(1, 30)}
    for edge in edges.values():
        n1, n2 = str(edge[0]), str(edge[1])
        x1, y1 = nodes[n1]
        x2, y2 = nodes[n2]
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        adj_list[n1].append((n2, distance))
        adj_list[n2].append((n1, distance))  # Undirected graph

    # == Closest Node Finder ==
    # Find closest node to start_x, start_y
    # ABSTRACTS ROUTE SELECTION, NO NEED FOR USER TO SET START NODE

    min_dist = float('inf')
    root_node = None
    for node, (x, y) in nodes.items():
        dist = math.sqrt((start_x - x)**2 + (start_y - y)**2)
        if dist < min_dist:
            min_dist = dist
            root_node = node
    
    # ==== Dijkstra's algorithm ====
    distances = {node: float('inf') for node in nodes}
    distances[root_node] = 0
    predecessors = {node: None for node in nodes}
    pq = [(0, root_node)]
    visited = set()
    
    while pq:
        curr_dist, curr_node = heapq.heappop(pq)
        if curr_node in visited:
            continue
        visited.add(curr_node)
        
        if curr_node == target_node:
            break
            
        for neighbor, weight in adj_list[curr_node]:
            if neighbor in visited:
                continue
            distance = curr_dist + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = curr_node
                heapq.heappush(pq, (distance, neighbor))
    
    # Reconstruct path
    if distances[target_node] == float('inf'):
        path = []  # No path exists
        return path
    
    curr = target_node
    while curr is not None:
        path.append(curr)
        curr = predecessors[curr]
    path.reverse()
    path_str = [str(x) for x in path]
    path = path_str

def router():
    global thr, steer, speed, turn_to, recv_html_X, recv_html_Y, recv_html_bearing

    in_node = False

    for idx, (name, coords) in enumerate(nodes.items()):
        if all(abs(a - b) <= 20 for a, b in zip(coords, [recv_html_X, recv_html_Y])):
            in_node = True
            if name in path:
                current_index = path.index(name)
                if current_index < len(path) - 1:
                
                    print("INSIDE NODE")
                    print(turn_to)
                    print(recv_html_X,recv_html_Y)

                    next_node = path[current_index + 1]
                    next_coords = nodes[next_node]

                    dx = next_coords[0] - coords[0]
                    dy = next_coords[1] - coords[1]

                    target_angle = math.degrees(math.atan2(dy, dx)) % 360

                    # Difference clockwise and anticlockwise
                    diff = (target_angle - recv_html_bearing) % 360

                    if diff < 180:
                        turn_to = -1
                    else:
                        turn_to = 1  # anticlockwise
                    print(f"At node {name}. Next: {next_node}, Target angle: {target_angle:.2f}, Bearing: {recv_html_bearing}, Turn: {turn_to}")
                else:
                    turn_to = 0
            else:
                turn_to = 0
            break
    if not in_node:
        turn_to = 0

#-------------------------------------------------------------------------------------------------------------------------

# === HTML SERVER FOR BULLET VENV CONNECTIVITY ===
app = Flask(__name__)

@app.route('/numbers', methods=['GET'])
def handle_numbers():
    global thr, steer, speed, turn_to, recv_html_X, recv_html_Y, recv_html_bearing

    # = DECODE RECV FROM SW = 
    recv_html_X = int(request.args.get('num1', 0))
    recv_html_Y = int(request.args.get('num2', 0))  
    recv_html_bearing = int(request.args.get('num3', 0))
    
    #print(f"Received: X coord={recv_html_X}, Y coord={recv_html_Y}, bearing={recv_html_bearing}") # DEBUGGING PURPOSE

    #  = ENCODE SEND TO SW =

    #print(f"Returned: thr={1}, steer={steer}, bearing={recv_html_bearing}") # DEBUGGING PURPOSE

    # SEND TXT TO SW, MAKE SURE URL MATCHES ON RECV END IS CORRECT
    return Response(f"{1},{steer},{recv_html_bearing}", mimetype='text/plain')

#-------------------------------------------------------------------------------------------------------------------------

def camera_loop():
    
    global thr, steer, speed, turn_to, recv_html_X, recv_html_Y, recv_html_bearing
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
        upper_dark = np.array([180, 80, 70])

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #CONVERT to HSV BECAUSE WHY NOT
        mask_dark = cv2.inRange(hsv_frame, lower_dark, upper_dark) #FIND ROAD USING DEFINED MASK PARAMETERS
        final_mask = cv2.bitwise_and(mask_dark, trapezoid_mask) #COMBINE MASK TO REDUCE MASK AREA

        # === CONTOUR MAKING ===
        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = hsv_frame
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
                router()
                if turn_to == 1 and cx_right is not None:
                    steer_value = (cx_right - (w // 2)) * 0.75
                    cv2.circle(output, (cx_right, cy_right), 15, (0, 0, 255), -1)
                elif turn_to == -1 and cx_left is not None:
                    steer_value = (cx_left - (w // 2)) * 0.75
                    cv2.circle(output, (cx_left, cy_left), 15, (255, 0, 0), -1)
                elif cx_whole is not None:
                    steer_value = (cx_whole - (w // 2)) * 0.75
                    cv2.circle(output, (cx_whole, cy_whole), 15, (0,255,0),-1)
                steer = steer_value
                print(steer)
                
        cv2.imshow("Frame", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    destination = str(input("List of Destinations : \nHarrison Airbase\nMount Toddy\nFishing Village\nOneil Airbase\nNorth Harbor\nOlsen Bay\nQuarry\n"))
    destination = destination.lower()
    if destination == "harrison airbase":
        go_to_location = 1
    elif destination == "mount todday":
        go_to_location = 26
    elif destination == "fishing village":
        go_to_location = 27
    elif destination == "oneil airbase":
        go_to_location = 29        
    elif destination == "north harbor":
        go_to_location = 13
    elif destination == "olsen bay":
        go_to_location = 8
    elif destination =="quarry":
        go_to_location = 28
    else:
        print("Invalid destination")    
    find_shortest_path(-6000, -6000, go_to_location)
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    
    app.run(host='0.0.0.0', port=8080)
    
