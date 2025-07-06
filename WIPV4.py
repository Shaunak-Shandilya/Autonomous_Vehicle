# FRAME IS MASK
# OUTPUT IS CONTOUR

import cv2
import numpy as np
from flask import Flask, request, Response
import threading
import time

# === PARAMETERS ===
CAM_INDEX = 1  # VirtualCam or webcam index

# === Globals for sharing data ===
thr = 0
steer = 0
speed = 0

# === Flask App ===
app = Flask(__name__)

@app.route('/numbers', methods=['GET'])
def handle_numbers():
    global offset_x, thr, speed

    # = DECODE RECV FROM SW = 
    recv_html_X = int(request.args.get('num1', 0))
    recv_html_Y = int(request.args.get('num2',0))  
    recv_html_speed = int(request.args.get('num3', 0))
    
    print(f"Received: X coord={recv_html_X}, Y coord={recv_html_Y}, speed={recv_html_speed}")

    #  = ENCODE SEND TO SW =  
    reply_html_thr = 1
    reply_html_steer = steer
    reply_html_speed = 100

    print(f"Returned: thr={reply_html_thr}, steer={reply_html_steer}, speed={reply_html_speed}")

    # Return plain text for Stormworks (not JSON)
    return Response(f"{thr},{steer},{speed}", mimetype='text/plain')


# === Video processing thread ===
def camera_loop():
    global thr, steer

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera index {CAM_INDEX}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # == TRAPEZOID CREATOR ==
        
        h, w = frame.shape[:2]

        trapezoid_mask = np.zeros((h, w), dtype=np.uint8)
        bottom_width = w
        top_width = w // 4

        pts = np.array([
            [w // 2 - bottom_width // 2, h*0.8],
            [w // 2 + bottom_width // 2, h*0.8],
            [w // 2 + top_width, int(h / 4)],
            [w // 2 - top_width, int(h / 4)]
        ], dtype=np.int32)

        cv2.fillPoly(trapezoid_mask, [pts], 255)

        # == MASKING PARAMETERS (HSV) ==
        
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 50, 70])  # max road color

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # == REAL MASKING BASED ON COLOR AND BOUNDING BOX(TRAPEZOID MASK) ===
        
        mask_dark = cv2.inRange(frame, lower_dark, upper_dark)
        final_mask = cv2.bitwise_and(mask_dark, trapezoid_mask)

        masked_result = cv2.bitwise_and(frame, frame, mask=final_mask)
        output = frame.copy()

        cv2.polylines(output, [pts], isClosed=True, color=(255, 0, 0), thickness=2)


        
        # === CONTOUR CODE ===


        
        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.line(output, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)

            largest = max(contours, key=cv2.contourArea)
            cv2.drawContours(output, [largest], -1, (0, 255, 0), 2)

            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # CENTROID OF CONTOUR
                cv2.circle(output, (cx, cy), 5, (0, 255, 0), -1) 

                center_x = w // 2
                offset_x = cx - center_x
                steer = offset_x

                #---------------------------------------------------------------------------------------------------------------------------X

                if cv2.pointPolygonTest(largest, (cx, cy+40), False) == -1:
                # --- SPLIT THE FRAME VERTICALLY AND FIND BOTH SIDES ---
                left_half = frame[:, :center_x]
                right_half = frame[:, center_x:]

                # Left mask
                mask_left = cv2.inRange(left_half, lower_dark, upper_dark)
                contours_left, _ = cv2.findContours(mask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours_left:
                    largest_left = max(contours_left, key=cv2.contourArea)
                    M_left = cv2.moments(largest_left)
                    if M_left["m00"] != 0:
                        cx_left = int(M_left["m10"] / M_left["m00"])
                        cy_left = int(M_left["m01"] / M_left["m00"])
                        cv2.circle(output, (cx_left, cy_left), 5, (255, 0, 0), -1)
                        print(f"Left centroid: ({cx_left}, {cy_left})")

                # Right mask
                mask_right = cv2.inRange(right_half, lower_dark, upper_dark)
                contours_right, _ = cv2.findContours(mask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours_right:
                    largest_right = max(contours_right, key=cv2.contourArea)
                    M_right = cv2.moments(largest_right)
                    if M_right["m00"] != 0:
                        cx_right = int(M_right["m10"] / M_right["m00"]) + center_x  # adjust!
                        cy_right = int(M_right["m01"] / M_right["m00"])
                        cv2.circle(output, (cx_right, cy_right), 5, (0, 0, 255), -1)
                        print(f"Right centroid: ({cx_right}, {cy_right})")

                #---------------------------------------------------------------------------------------------------------------------------X
                
                print(f"Centroid X: {cx}, Offset: {offset_x}")
            else:
                offset_x = 0
        else:
            offset_x = 0

        cv2.imshow("Frame", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# === Run both ===


if __name__ == '__main__':
    # Start camera in a separate thread
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()

    # Run Flask server
    app.run(host='0.0.0.0', port=8080)
    
