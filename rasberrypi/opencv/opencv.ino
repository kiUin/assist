import cv2
import numpy as np
import serial
import time

# -------------------- Serial --------------------
PORT = "/dev/ttyUSB0"   # might be /dev/ttyACM0
BAUD = 115200

def send(ser, msg):
    ser.write((msg.strip() + "\n").encode("utf-8"))

def read_nonblock(ser):
    out = []
    while ser.in_waiting:
        out.append(ser.readline().decode("utf-8", errors="ignore").strip())
    return out

def wait_for_ok(ser, timeout=2.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        lines = read_nonblock(ser)
        for ln in lines:
            if ln.startswith("OK") or ln.startswith("ERR"):
                return ln
        time.sleep(0.01)
    return None

# -------------------- Camera --------------------
CAM_INDEX = 0
cap = cv2.VideoCapture(CAM_INDEX)

# -------------------- TUNE THESE ROIs --------------------
# Bowl ROI: x,y,w,h in full frame
BOWL_ROI = (200, 120, 300, 300)     # TODO: adjust for your camera mount
# Spoon ROI (presentation point): x,y,w,h in full frame
SPOON_ROI = (520, 180, 140, 120)    # TODO: adjust for your presentation pose

# Simple “food mask” strategy:
# For MVP, pick ONE food that’s visually distinct (e.g., dark cereal on light bowl).
# Use background subtraction: compare against an "empty bowl baseline" captured once.
empty_bowl_baseline = None
empty_spoon_baseline = None

def roi_crop(frame, roi):
    x,y,w,h = roi
    return frame[y:y+h, x:x+w]

def score_zone(mask, zone_roi):
    x,y,w,h = zone_roi
    return int(np.count_nonzero(mask[y:y+h, x:x+w]))

def compute_food_mask(bowl_img, empty_bowl):
    # Simple abs-diff vs baseline -> threshold
    g = cv2.cvtColor(bowl_img, cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(empty_bowl, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(g, gb)
    _, mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    # Light cleanup
    mask = cv2.medianBlur(mask, 5)
    return mask

def bite_present(spoon_img, empty_spoon):
    g = cv2.cvtColor(spoon_img, cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(empty_spoon, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(g, gb)
    mean_diff = float(diff.mean())
    return mean_diff, (mean_diff > 12.0)  # TODO: tune threshold

def draw_roi(frame, roi, color, label):
    x,y,w,h = roi
    cv2.rectangle(frame, (x,y), (x+w, y+h), color, 2)
    cv2.putText(frame, label, (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def run_next_bite_cycle(ser):
    # 1) Decide zone
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        return

    if empty_bowl_baseline is None:
        print("No empty bowl baseline. Press 'b' first.")
        return
    if empty_spoon_baseline is None:
        print("No empty spoon baseline. Press 's' first.")
        return

    bowl = roi_crop(frame, BOWL_ROI)
    mask = compute_food_mask(bowl, empty_bowl_baseline)

    # Define 3 zones inside bowl ROI (in bowl-local coordinates)
    H, W = mask.shape
    zone_w = W // 3
    zones = {
        "FOOD_LEFT":   (0,         0, zone_w, H),
        "FOOD_CENTER": (zone_w,    0, zone_w, H),
        "FOOD_RIGHT":  (2*zone_w,  0, W - 2*zone_w, H),
    }

    scores = {k: score_zone(mask, v) for k,v in zones.items()}
    best_zone = max(scores, key=scores.get)
    print("Zone scores:", scores, "best:", best_zone)

    # 2) Send commands to ESP32
    send(ser, "TOOL SPOON")                  # or FORK based on your voice mode
    wait_for_ok(ser)

    send(ser, f"POSE {best_zone}")
    wait_for_ok(ser)

    send(ser, "ACTION SCOOP")
    wait_for_ok(ser)

    send(ser, "POSE PRESENT")
    wait_for_ok(ser)

    time.sleep(0.4)  # let it settle in presentation pose

    # 3) Verify bite present at presentation ROI
    ret, frame2 = cap.read()
    if not ret:
        print("Camera read failed")
        return

    spoon = roi_crop(frame2, SPOON_ROI)
    mean_diff, ok = bite_present(spoon, empty_spoon_baseline)
    print(f"Bite check mean_diff={mean_diff:.1f} present={ok}")

    if not ok:
        # Retry once
        print("Retrying scoop once...")
        send(ser, f"POSE {best_zone}")
        wait_for_ok(ser)
        send(ser, "ACTION SCOOP")
        wait_for_ok(ser)
        send(ser, "POSE PRESENT")
        wait_for_ok(ser)

    print("Cycle done.")

# -------------------- Main --------------------
print("Opening serial...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2.0)

print("Type: press 'a' to ARM, 'b' capture empty bowl, 's' capture empty spoon, 'n' run cycle, 'q' quit")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Draw ROIs
    draw_roi(frame, BOWL_ROI,  (0,255,0), "BOWL_ROI")
    draw_roi(frame, SPOON_ROI, (255,0,0), "SPOON_ROI")

    cv2.imshow("BiteStation", frame)
    k = cv2.waitKey(1) & 0xFF

    if k == ord('q'):
        break
    elif k == ord('a'):
        send(ser, "ARM")
        print(wait_for_ok(ser))
    elif k == ord('b'):
        # Capture empty bowl baseline from current frame
        empty_bowl_baseline = roi_crop(frame, BOWL_ROI).copy()
        print("Captured empty bowl baseline.")
    elif k == ord('s'):
        # Capture empty spoon baseline (make sure spoon is empty & in presentation pose)
        empty_spoon_baseline = roi_crop(frame, SPOON_ROI).copy()
        print("Captured empty spoon baseline.")
    elif k == ord('n'):
        run_next_bite_cycle(ser)

cap.release()
cv2.destroyAllWindows()
ser.close()
