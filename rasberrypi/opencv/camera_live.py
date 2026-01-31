import cv2

CAM_INDEX = 0  # try 1 if 0 doesn't work

cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Could not open camera. Try CAM_INDEX=1.")

print("Press ESC to quit.")
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame grab failed")
        break

    cv2.imshow("Camera Live", frame)

    if (cv2.waitKey(1) & 0xFF) == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
