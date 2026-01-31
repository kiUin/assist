import cv2
import numpy as np
import time
import argparse
from serial.tools import list_ports
import serial

def auto_port():
    candidates = []
    for p in list_ports.comports():
        name = p.device.lower()
        if "usbmodem" in name or "usbserial" in name or "wchusbserial" in name:
            candidates.append(p.device)
    return candidates[0] if candidates else None

def send_and_wait(ser, cmd, timeout=2.0):
    ser.write((cmd.strip() + "\n").encode("utf-8"))
    t0 = time.time()
    buf = []
    while time.time() - t0 < timeout:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            buf.append(line)
            if line.startswith("OK") or line.startswith("ERR"):
                return line, buf
        time.sleep(0.01)
    return None, buf

def clamp_roi(x, y, w, h, W, H):
    x = max(0, min(x, W-1))
    y = max(0, min(y, H-1))
    w = max(1, min(w, W-x))
    h = max(1, min(h, H-y))
    return x, y, w, h

def crop(frame, roi):
    x, y, w, h = roi
    return frame[y:y+h, x:x+w]

def food_mask(bowl_bgr, empty_bowl_bgr, thr=25):
    g  = cv2.cvtColor(bowl_bgr, cv2.COLOR_BGR2GRAY)
    g0 = cv2.cvtColor(empty_bowl_bgr, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(g, g0)
    _, m = cv2.threshold(diff, thr, 255, cv2.THRESH_BINARY)
    m = cv2.medianBlur(m, 5)
    return m

def bite_present(spoon_bgr, empty_spoon_bgr, thr=12.0):
    g  = cv2.cvtColor(spoon_bgr, cv2.COLOR_BGR2GRAY)
    g0 = cv2.cvtColor(empty_spoon_bgr, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(g, g0)
    score = float(diff.mean())
    return score, (score > thr)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="auto", help="Serial port (e.g. /dev/tty.usbmodemXXXX) or 'auto'")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cam", type=int, default=0)
    args = ap.parse_args()

    # Serial
    ser = None
    if args.port != "none":
        port = auto_port() if args.port == "auto" else args.port
        if port is None:
            print("No serial port found. Run with --port none for vision-only.")
        else:
            print("Using serial port:", port)
            ser = serial.Serial(port, args.baud, timeout=0.1)
            time.sleep(1.5)

    # Camera
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try --cam 1")

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Could not read frame from camera.")

    H, W = frame.shape[:2]

    # Initial ROIs (edit via trackbars)
    bowl = [int(W*0.20), int(H*0.20), int(W*0.30), int(H*0.45)]
    spoon= [int(W*0.65), int(H*0.25), int(W*0.18), int(H*0.18)]

    empty_bowl = None
    empty_spoon = None

    # Trackbars for fast ROI tuning
    cv2.namedWindow("BiteStation")
    def noop(v): pass

    cv2.createTrackbar("bowl_x", "BiteStation", bowl[0], W-1, noop)
    cv2.createTrackbar("bowl_y", "BiteStation", bowl[1], H-1, noop)
    cv2.createTrackbar("bowl_w", "BiteStation", bowl[2], W, noop)
    cv2.createTrackbar("bowl_h", "BiteStation", bowl[3], H, noop)

    cv2.createTrackbar("spoon_x", "BiteStation", spoon[0], W-1, noop)
    cv2.createTrackbar("spoon_y", "BiteStation", spoon[1], H-1, noop)
    cv2.createTrackbar("spoon_w", "BiteStation", spoon[2], W, noop)
    cv2.createTrackbar("spoon_h", "BiteStation", spoon[3], H, noop)

    cv2.createTrackbar("food_thr", "BiteStation", 25, 80, noop)
    cv2.createTrackbar("bite_thr_x10", "BiteStation", 120, 400, noop)  # 12.0 * 10

    print("Keys: a=ARM  b=empty bowl  s=empty spoon  n=next bite  ESC=quit")

    def do_next_bite_cycle(frame_now):
        nonlocal empty_bowl, empty_spoon

        if ser is None:
            print("No serial connected; running vision-only.")
            return

        if empty_bowl is None or empty_spoon is None:
            print("Capture baselines first: press b (empty bowl) and s (empty spoon).")
            return

        bx = cv2.getTrackbarPos("bowl_x", "BiteStation")
        by = cv2.getTrackbarPos("bowl_y", "BiteStation")
        bw = cv2.getTrackbarPos("bowl_w", "BiteStation")
        bh = cv2.getTrackbarPos("bowl_h", "BiteStation")
        sx = cv2.getTrackbarPos("spoon_x", "BiteStation")
        sy = cv2.getTrackbarPos("spoon_y", "BiteStation")
        sw = cv2.getTrackbarPos("spoon_w", "BiteStation")
        sh = cv2.getTrackbarPos("spoon_h", "BiteStation")

        bx, by, bw, bh = clamp_roi(bx, by, bw, bh, W, H)
        sx, sy, sw, sh = clamp_roi(sx, sy, sw, sh, W, H)

        food_thr = cv2.getTrackbarPos("food_thr", "BiteStation")
        bite_thr = cv2.getTrackbarPos("bite_thr_x10", "BiteStation") / 10.0

        bowl_roi = crop(frame_now, (bx, by, bw, bh))
        mask = food_mask(bowl_roi, empty_bowl, thr=food_thr)

        # Split bowl ROI into 3 vertical zones
        hh, ww = mask.shape
        z = ww // 3
        scores = {
            "FOOD_LEFT":   int(np.count_nonzero(mask[:, 0:z])),
            "FOOD_CENTER": int(np.count_nonzero(mask[:, z:2*z])),
            "FOOD_RIGHT":  int(np.count_nonzero(mask[:, 2*z:ww])),
        }
        best = max(scores, key=scores.get)
        print("Zone scores:", scores, "best:", best)

        # Command sequence
        print(send_and_wait(ser, "ARM")[0])
        print(send_and_wait(ser, "TOOL SPOON")[0])
        print(send_and_wait(ser, f"POSE {best}")[0])
        print(send_and_wait(ser, "ACTION SCOOP")[0])
        print(send_and_wait(ser, "POSE PRESENT")[0])

        time.sleep(0.2)

        # Verify bite at spoon ROI
        ret2, frame2 = cap.read()
        if not ret2:
            print("Camera read failed at verify step")
            return
        spoon_roi = crop(frame2, (sx, sy, sw, sh))
        score, ok = bite_present(spoon_roi, empty_spoon, thr=bite_thr)
        print(f"Verify: bite_score={score:.1f} ok={ok}")

        if not ok:
            print("Retry scoop once...")
            print(send_and_wait(ser, f"POSE {best}")[0])
            print(send_and_wait(ser, "ACTION SCOOP")[0])
            print(send_and_wait(ser, "POSE PRESENT")[0])

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        bx = cv2.getTrackbarPos("bowl_x", "BiteStation")
        by = cv2.getTrackbarPos("bowl_y", "BiteStation")
        bw = cv2.getTrackbarPos("bowl_w", "BiteStation")
        bh = cv2.getTrackbarPos("bowl_h", "BiteStation")
        sx = cv2.getTrackbarPos("spoon_x", "BiteStation")
        sy = cv2.getTrackbarPos("spoon_y", "BiteStation")
        sw = cv2.getTrackbarPos("spoon_w", "BiteStation")
        sh = cv2.getTrackbarPos("spoon_h", "BiteStation")

        bx, by, bw, bh = clamp_roi(bx, by, bw, bh, W, H)
        sx, sy, sw, sh = clamp_roi(sx, sy, sw, sh, W, H)

        # Draw ROIs
        cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (0,255,0), 2)
        cv2.putText(frame, "BOWL", (bx, by-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.rectangle(frame, (sx, sy), (sx+sw, sy+sh), (255,0,0), 2)
        cv2.putText(frame, "SPOON", (sx, sy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        # Live bite score display if baseline exists
        bite_thr = cv2.getTrackbarPos("bite_thr_x10", "BiteStation") / 10.0
        if empty_spoon is not None:
            score, ok_ = bite_present(crop(frame, (sx, sy, sw, sh)), empty_spoon, thr=bite_thr)
            cv2.putText(frame, f"bite_score={score:.1f} ({'YES' if ok_ else 'NO'})",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0,255,0) if ok_ else (0,0,255), 2)
        else:
            cv2.putText(frame, "Press 's' to capture EMPTY spoon baseline",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

        cv2.imshow("BiteStation", frame)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC
            break
        elif k == ord('a'):
            if ser: print(send_and_wait(ser, "ARM")[0])
        elif k == ord('b'):
            empty_bowl = crop(frame, (bx, by, bw, bh)).copy()
            print("Captured empty bowl baseline.")
        elif k == ord('s'):
            empty_spoon = crop(frame, (sx, sy, sw, sh)).copy()
            print("Captured empty spoon baseline.")
        elif k == ord('n'):
            do_next_bite_cycle(frame)

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()

if __name__ == "__main__":
    main()
