# ==========================
# SO100ARM Main Python Code
# ==========================
# Handles: image capture, processing, target detection,
# coordinate mapping, and arm control communication.

import cv2
import numpy as np
import serial
import math
import time

# -----------------------------
# (1) INITIAL SETUP
# -----------------------------

# Initialize serial connection to Arduino (for real hardware)
# ser = serial.Serial('/dev/ttyUSB0', 9600)   # uncomment for real use

# Camera index (0 for built-in / USB camera)
cap = cv2.VideoCapture(0)

# Helper function: nothing yet, just in case for trackbars
def nothing(x): pass


# -----------------------------
# (2) IMAGE CAPTURE & PREPROCESSING
# -----------------------------
def preprocess_frame(frame):
    """Convert to grayscale, enhance contrast, and reduce noise."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)

    # Reduce noise with Gaussian blur
    blur = cv2.GaussianBlur(enhanced, (5,5), 0)

    return blur


# -----------------------------
# (3) VEIN ENHANCEMENT
# -----------------------------
def enhance_veins(img):
    """Enhance dark vein-like patterns."""
    # Morphological top-hat and black-hat filtering
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)
    blackhat = cv2.morphologyEx(img, cv2.MORPH_BLACKHAT, kernel)

    # Combine results
    result = cv2.add(img, tophat)
    result = cv2.subtract(result, blackhat)
    return result


# -----------------------------
# (4) THRESHOLDING AND VEIN DETECTION
# -----------------------------
def detect_veins(img):
    """Convert to binary image and find main vein contours."""
    # Adaptive thresholding for robust detection
    thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 15, 5)

    # Morphological closing to connect broken lines
    kernel = np.ones((5,5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Find contours (each contour = possible vein)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return closed, contours


# -----------------------------
# (5) TARGET SELECTION
# -----------------------------
def select_best_vein(contours):
    """Select the longest, clearest vein as the puncture target."""
    if len(contours) == 0:
        return None

    # Choose contour with maximum perimeter (length)
    best_contour = max(contours, key=lambda c: cv2.arcLength(c, False))

    # Find centroid of that contour
    M = cv2.moments(best_contour)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy), best_contour


# -----------------------------
# (6) MAPPING TO ROBOT COORDINATES
# -----------------------------
def map_to_robot(x, y, frame_shape):
    """Map image coordinates (x,y) to robot workspace coordinates."""
    width, height = frame_shape[1], frame_shape[0]
    # Assume simple linear scaling (for demo)
    scale_x, scale_y = 0.5, 0.5   # calibration constants (mm per pixel)
    base_offset_x, base_offset_y = 100, 50
    robot_x = base_offset_x + x * scale_x
    robot_y = base_offset_y + y * scale_y
    return robot_x, robot_y


# -----------------------------
# (7) INVERSE KINEMATICS + COMMAND
# -----------------------------
def send_to_arm(x, y):
    """Compute angles for 2-link arm (simplified) and send via serial."""
    L1, L2 = 150, 120  # link lengths in mm
    dx, dy = x, y
    d = math.hypot(dx, dy)

    if d > (L1 + L2):
        print("Target out of reach.")
        return

    # Law of cosines
    angle2 = math.acos((d**2 - L1**2 - L2**2)/(2*L1*L2))
    angle1 = math.atan2(dy, dx) - math.atan2(L2*math.sin(angle2), L1 + L2*math.cos(angle2))

    # Convert to degrees
    a1, a2 = math.degrees(angle1), math.degrees(angle2)
    print(f"Target angles -> θ1: {a1:.1f}°, θ2: {a2:.1f}°")

    # Send to Arduino (example string)
    # ser.write(f"{a1:.1f},{a2:.1f}\n".encode())


# -----------------------------
# (8) MAIN LOOP
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    processed = preprocess_frame(frame)
    enhanced = enhance_veins(processed)
    binary, contours = detect_veins(enhanced)

    target, best_contour = select_best_vein(contours)
    display = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

    if target is not None:
        cx, cy = target
        cv2.circle(display, (cx, cy), 6, (0,0,255), -1)
        cv2.drawContours(display, [best_contour], -1, (0,255,0), 2)

        robot_x, robot_y = map_to_robot(cx, cy, frame.shape)
        send_to_arm(robot_x, robot_y)
        cv2.putText(display, f"Target: ({cx},{cy})", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    else:
        cv2.putText(display, "No veins detected", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imshow("SO100ARM - Vein Detection", display)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
