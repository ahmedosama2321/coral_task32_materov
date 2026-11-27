"""Live test of the red-square detector on your PC webcam.
Hold a RED square of paper (or open a red square on your phone) in front of the camera.
Shows the detected centre and the (forward, right) offset in metres. Press q to quit.
Assumes the square is 15 cm per side (vision.SQUARE_SIDE_M) - use a 15 cm one for real distances.
Use:  python test_webcam.py          (camera 0)    |    python test_webcam.py 1   (camera 1)
"""
import sys, cv2
from vision import red_offset
cap = cv2.VideoCapture(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
if not cap.isOpened(): sys.exit("Camera not found - try: python test_webcam.py 1")
while True:
    ok, f = cap.read()
    if not ok: break
    h, w = f.shape[:2]
    cv2.drawMarker(f, (w//2, h//2), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
    o = red_offset(f)
    txt = "no red square" if o is None else f"forward {o[0]*100:+.1f} cm   right {o[1]*100:+.1f} cm"
    cv2.putText(f, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("red square test (q = quit)", f)
    if cv2.waitKey(1) & 0xFF == ord("q"): break
cap.release(); cv2.destroyAllWindows()
