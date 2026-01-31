import cv2
import numpy as np

CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open camera. Try CAM_INDEX=1.")

# TODO: adjust this to where the spoon tip appears at your presentation point
SPOON_ROI = (560, 180, 160, 120)  # x, y, w, h

baseline = None
THRESH = 12.0  # TODO: tune (higher = less sensitive)

def crop(frame, roi):
    x, y, w, h = roi
    return frame[y:y+h, x:x+w]

print("Controls: press 's' to capture EMPTY spoon baseline, ESC to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    roi = crop(frame, SPOON_ROI)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # Draw ROI
    x, y, w, h = SPOON_ROI
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2)

    diff_score = None
    bite = False

    if baseline is not None:
        diff = cv2.absdiff(gray, baseline)
        diff_score = float(diff.mean())
        bite = diff_score > THRESH

        cv2.putText(frame, f"diff={diff_score:.1f}  BITE={'YES' if bite else 'NO'}",
                    (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0,255,0) if bite else (0,0,255), 2)

    else:
        cv2.putText(frame, "Press 's' to capture empty spoon baseline",
                    (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

    cv2.imshow("Bite Detector", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    elif key == ord('s'):
        baseline = gray.copy()
        print("Captured empty spoon baseline.")

cap.release()
cv2.destroyAllWindows()
