import cv2
import numpy as np
from flask import Flask, request, Response
import threading
import time

# === PARAMETERS ===
CAM_INDEX = 1  # VirtualCam or webcam index

# === Globals for sharing data ===
offset_x = 0  # steer value (will be updated in video thread)
thr = 0
speed = 0

# === PID mc ===
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output

# Create PID with only P and D for simplicity
pid = PID(Kp=0.05, Ki=0.0, Kd=0.01)


# === Flask App ===
app = Flask(__name__)

@app.route('/numbers', methods=['GET'])
def handle_numbers():
    global offset_x, thr, speed

    # Get values from query (Stormworks input)
    thr = int(request.args.get('num1', 0))
    steer = offset_x  # use the current offset from camera thread
    speed = int(request.args.get('num3', 0))

    print(f"Received: thr={thr}, steer={steer}, speed={speed}")

    # Double them
    thr *= 2
    steer *= 2
    speed *= 2

    print(f"Returned: thr={thr}, steer={steer}, speed={speed}")

    # Return plain text for Stormworks (not JSON)
    return Response(f"{thr},{steer},{speed}", mimetype='text/plain')

# === Video processing thread ===
def camera_loop():
    global offset_x

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print(f"Could not open camera index {CAM_INDEX}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        h, w = frame.shape[:2]

        trapezoid_mask = np.zeros((h, w), dtype=np.uint8)
        bottom_width = w
        top_width = w // 4

        pts = np.array([
            [w // 2 - bottom_width // 2, h],
            [w // 2 + bottom_width // 2, h],
            [w // 2 + top_width, int(h / 4)],
            [w // 2 - top_width, int(h / 4)]
        ], dtype=np.int32)

        cv2.fillPoly(trapezoid_mask, [pts], 255)

        # change to HSV
        lower_dark = np.array([0, 0, 0])
        upper_dark = np.array([180, 50, 70])  # adjust to your road color

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask_dark = cv2.inRange(frame, lower_dark, upper_dark)
        final_mask = cv2.bitwise_and(mask_dark, trapezoid_mask)

        masked_result = cv2.bitwise_and(frame, frame, mask=final_mask)
        output = frame.copy()

        cv2.polylines(output, [pts], isClosed=True, color=(255, 0, 0), thickness=2)

        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            cv2.line(output, (w // 2, 0), (w // 2, h), (0, 0, 255), 2)

            largest = max(contours, key=cv2.contourArea)
            cv2.drawContours(output, [largest], -1, (0, 255, 0), 2)

            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(output, (cx, cy), 5, (255, 255, 0), -1)

                center_x = w // 2
                offset_x = cx - center_x

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
    
