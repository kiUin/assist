import cv2

CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open camera. Try CAM_INDEX=1.")

# TODO: adjust these to match your setup
BOWL_ROI  = (200, 120, 320, 320)   # x, y, w, h
SPOON_ROI = (560, 180, 160, 120)

def draw_roi(frame, roi, color, label):
    x, y, w, h = roi
    cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
    cv2.putText(frame, label, (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

print("Press ESC to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    draw_roi(frame, BOWL_ROI, (0,255,0), "BOWL_ROI")
    draw_roi(frame, SPOON_ROI, (255,0,0), "SPOON_ROI")

    cv2.imshow("ROI Viewer", frame)
    if (cv2.waitKey(1) & 0xFF) == 27:
        break

cap.release()
cv2.destroyAllWindows()
