import cv2
import apriltag
import numpy as np
import threading
import time

class CameraStream:
    def __init__(self, width=640, height=480, fps=15):
        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        self.ret, self.frame = self.cap.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.cap.release()

def detect_apriltags(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    
    for tag in tags:
        print(f"Detected tag ID: {tag.tag_id}")
        corners = [(int(pt[0]), int(pt[1])) for pt in tag.corners]
        for i in range(len(corners)):
            cv2.line(frame, corners[i], corners[(i + 1) % len(corners)], (0, 255, 0), 2)
        
        center = (int(tag.center[0]), int(tag.center[1]))
        cv2.putText(frame, str(tag.tag_id), center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return frame

last_detection_time = 0
debounce_interval = 2  # seconds

def detect_strips(frame):
    global last_detection_time
    current_time = time.time()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 55, 255])

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask_red1 | mask_red2
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    mask = cv2.bitwise_or(mask_red, mask_white)

    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)
    kernel = np.ones((5, 5), np.uint8)
    morphed_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_CLOSE, kernel)
    morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(morphed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    tape_detected = False
    for contour in contours:
        if cv2.contourArea(contour) > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            tape_detected = True

    if tape_detected and (current_time - last_detection_time > debounce_interval):
        print("tape")
        last_detection_time = current_time

    return frame


def main():
    options = apriltag.DetectorOptions(
        families="tag36h11",
        border=1,
        nthreads=1,
        quad_decimate=1.0,
    )
    detector = apriltag.Detector(options)
    
    camera = CameraStream().start()
    
    while True:
        frame = camera.read()
        
        # Detect AprilTags and draw bounding boxes
        frame = detect_apriltags(frame, detector)
        
        # Detect strips on the original color frame
        #frame = detect_strips(frame)
        
        cv2.imshow('Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    camera.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
