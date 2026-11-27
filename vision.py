"""Red Velcro detector (15 cm square). Works on any BGR frame from OpenCV."""
import cv2, numpy as np

SQUARE_SIDE_M = 0.15          # manual p.30: 15 cm red Velcro square
MIN_AREA_PX = 400             # ignore specks

def red_offset(frame, side_m=SQUARE_SIDE_M):
    """Return (forward_m, right_m) from the image centre to the red square, or None.
    Assumes the camera looks DOWN/forward-down: image up = forward, image right = right.
    Metres-per-pixel comes from the square's own apparent size (known 15 cm), so no
    separate distance sensor is needed."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA_PX: return None
    M = cv2.moments(c); cx, cy = M["m10"]/M["m00"], M["m01"]/M["m00"]
    h, w = frame.shape[:2]
    m_per_px = side_m / np.sqrt(area)
    return ((h/2 - cy) * m_per_px, (cx - w/2) * m_per_px)

class Camera:
    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)
    def offset(self):
        ok, f = self.cap.read()
        return red_offset(f) if ok else None
