#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO


bridge = CvBridge()
model_letters = YOLO('Computer_Vision_Functions/alphabet.pt')
model_signs = YOLO('Computer_Vision_Functions/SignModel.pt')
current_model = model_letters
is_letter_mode = True

lower_green = np.array([35, 50, 50])
upper_green = np.array([85, 255, 255])
lower_red1 = np.array([0, 50, 50])            #COLORS RANGE FOR RED AND GREEN
upper_red1 = np.array([10, 255, 255])         #IF IT DOESNT WORK, TRY CHANGING THESE VALUES
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

def initialize_cv(): #Should be used in main file
    pub_detections = rospy.Publisher('/detections', String, queue_size=10)
    return pub_detections

def switch_to_signs():
    global current_model, is_letter_mode
    current_model = model_signs
    is_letter_mode = False

def switch_to_letters():
    global current_model, is_letter_mode
    current_model = model_letters
    is_letter_mode = True

def is_green_background(crop_img):
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    green_pixels = cv2.countNonZero(mask_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    red_pixels = cv2.countNonZero(mask_red)
    return green_pixels > 2 * red_pixels and green_pixels > 100

def process_image(msg, pub_detections):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.resize(cv_image, (640, 640))
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge Error: {e}")
        return None
    
    results = current_model(cv_image, verbose=False)
    detections = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                if is_letter_mode:
                    letter = model_letters.names[cls]
                    crop = cv_image[max(0, y1-10):min(cv_image.shape[0], y2+10),
                                  max(0, x1-10):min(cv_image.shape[1], x2+10)]
                    if is_green_background(crop):
                        detections.append(f"SAFE_LETTER: {letter}")
                    else:
                        detections.append(f"PENALTY_LETTER: {letter}")
                else:
                    sign = model_signs.names[cls]
                    detections.append(f"SIGN: {sign}")
    
    if detections and pub_detections:
        pub_detections.publish("\n".join(detections))
    return detections

def handle_detection(detections):
    for det in detections or []:
        if "SAFE_LETTER" in det:
            letter = det.split(": ")[1]
            return "collect", letter
        elif "PENALTY_LETTER" in det:
            return "skip", None
        elif "SIGN" in det:
            sign = det.split(": ")[1]
            return "turn", sign
    return "none", None

def run_camera_inference(camera_index=2):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        cap.release()
        return []

    frame_resized = cv2.resize(frame, (640, 640))
    results = current_model(frame_resized, verbose=False)
    detections = []

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if conf > 0.5:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                if is_letter_mode:
                    letter = model_letters.names[cls]
                    crop = frame_resized[max(0, y1-10):min(frame_resized.shape[0], y2+10),
                                         max(0, x1-10):min(frame_resized.shape[1], x2+10)]
                    if is_green_background(crop):
                        detections.append(f"SAFE_LETTER: {letter}")
                    else:
                        detections.append(f"PENALTY_LETTER: {letter}")
                else:
                    sign = model_signs.names[cls]
                    detections.append(f"SIGN: {sign}")

    cap.release()
    return detections

if __name__ == "__main__":
    # Test camera indices
    for index in range(4):  # Try indices 0-3
        print(f"Testing camera index {index}")
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow(f"Camera {index}", frame)
                cv2.waitKey(1000)  # Show for 1 second
                print(f"Camera {index} works!")
            cap.release()
        cv2.destroyAllWindows()