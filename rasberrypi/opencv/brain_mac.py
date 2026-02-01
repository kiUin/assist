import argparse
import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np

# Serial optional
try:
    import serial
    from serial.tools import list_ports
except Exception:
    serial = None
    list_ports = None

CFG_PATH = Path("bowl_nomarker_cfg.json")
BASELINE_PATH = Path("bowl_nomarker_baseline.npz")


@dataclass
class Config:
    cam_index: int = 0
    baud: int = 115200

    # One bowl ROI in CAMERA FRAME coordinates
    bowl_x: int = 80
    bowl_y: int = 60
    bowl_w: int = 360
    bowl_h: int = 360

    # Thresholding
    food_thr: int = 25
    min_food_pixels: int = 1200

    # Auto mode
    start_mode: str = "key"       # key/auto
    stable_frames: int = 8
    auto_cooldown_s: float = 3.0

    # Optional auto-find (Hough circle) parameters
    hough_dp_x10: int = 12        # dp = 1.2
    hough_minDist: int = 200
    hough_param1: int = 120       # Canny high threshold
    hough_param2: int = 45        # accumulator threshold
    hough_minR: int = 80
    hough_maxR: int = 220


def load_config() -> Config:
    if CFG_PATH.exists():
        try:
            return Config(**json.loads(CFG_PATH.read_text()))
        except Exception:
            pass
    return Config()


def save_config(cfg: Config) -> None:
    CFG_PATH.write_text(json.dumps(asdict(cfg), indent=2))


def save_baseline(cfg: Config, empty_bowl_roi_bgr: np.ndarray) -> None:
    np.savez_compressed(
        BASELINE_PATH,
        bowl_roi=np.array([cfg.bowl_x, cfg.bowl_y, cfg.bowl_w, cfg.bowl_h], dtype=np.int32),
        empty_bowl=empty_bowl_roi_bgr,
    )


def load_baseline(cfg: Config):
    if not BASELINE_PATH.exists():
        return None
    try:
        z = np.load(BASELINE_PATH, allow_pickle=False)
        bx, by, bw, bh = [int(x) for x in z["bowl_roi"]]
        cfg.bowl_x, cfg.bowl_y, cfg.bowl_w, cfg.bowl_h = bx, by, bw, bh
        return z["empty_bowl"]
    except Exception:
        return None


def clamp_roi(x, y, w, h, W, H):
    x = max(0, min(int(x), W - 1))
    y = max(0, min(int(y), H - 1))
    w = max(1, min(int(w), W - x))
    h = max(1, min(int(h), H - y))
    return x, y, w, h


def crop(img, roi_xywh):
    x, y, w, h = roi_xywh
    return img[y:y + h, x:x + w]


def food_mask(current_bgr, empty_bgr, thr=25):
    g = cv2.cvtColor(current_bgr, cv2.COLOR_BGR2GRAY)
    g0 = cv2.cvtColor(empty_bgr, cv2.COLOR_BGR2GRAY)

    g = cv2.GaussianBlur(g, (5, 5), 0)
    g0 = cv2.GaussianBlur(g0, (5, 5), 0)

    diff = cv2.absdiff(g, g0)
    _, m = cv2.threshold(diff, int(thr), 255, cv2.THRESH_BINARY)

    m = cv2.medianBlur(m, 5)
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
    m = cv2.morphologyEx(m, cv2.MORPH_DILATE, k, iterations=1)
    return m


def pick_zone(mask):
    hh, ww = mask.shape
    z = max(1, ww // 3)
    left = int(np.count_nonzero(mask[:, 0:z]))
    center = int(np.count_nonzero(mask[:, z:2*z]))
    right = int(np.count_nonzero(mask[:, 2*z:ww]))
    scores = {"FOOD_LEFT": left, "FOOD_CENTER": center, "FOOD_RIGHT": right}
    best = max(scores, key=scores.get)
    return best, scores


# ---- Serial helpers ----
def auto_port():
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:
        dev = (p.device or "").lower()
        if "ttyacm" in dev or "ttyusb" in dev:
            return p.device
    return ports[0].device


def drain_serial(ser, max_bytes=4096):
    try:
        if ser.in_waiting:
            ser.read(min(max_bytes, ser.in_waiting))
    except Exception:
        pass


def send_and_wait(ser, cmd, timeout=2.0):
    drain_serial(ser)
    ser.write((cmd.strip() + "\n").encode("utf-8"))
    t0 = time.time()
    lines = []
    while time.time() - t0 < timeout:
        while ser.in_waiting:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                lines.append(line)
                if line.startswith("OK") or line.startswith("ERR"):
                    return line, lines
        time.sleep(0.01)
    return None, lines


def handshake(ser):
    for _ in range(3):
        resp, _ = send_and_wait(ser, "PING", timeout=1.0)
        if resp and resp.startswith("OK"):
            return True
        time.sleep(0.2)
    return False


# ---- Optional bowl auto-find ----
def auto_find_bowl_circle(frame_bgr, cfg: Config):
    """
    Attempts to find a bowl rim as a circle in the full frame.
    Returns (cx, cy, r) or None.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    dp = cfg.hough_dp_x10 / 10.0
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=cfg.hough_minDist,
        param1=cfg.hough_param1,
        param2=cfg.hough_param2,
        minRadius=cfg.hough_minR,
        maxRadius=cfg.hough_maxR,
    )
    if circles is None:
        return None
    circles = np.round(circles[0, :]).astype(int)

    # Choose the largest circle (often the bowl rim)
    circles = sorted(circles, key=lambda c: c[2], reverse=True)
    cx, cy, r = circles[0]
    return (cx, cy, r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="none", help="'auto', 'none', or explicit like /dev/ttyACM0")
    ap.add_argument("--cam", type=int, default=None)
    ap.add_argument("--start", choices=["key", "auto"], default=None)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    cfg = load_config()
    if args.cam is not None:
        cfg.cam_index = args.cam
    if args.start is not None:
        cfg.start_mode = args.start

    # Serial init
    ser = None
    if args.port != "none" and not args.dry_run:
        if serial is None:
            print("pyserial not available; run with --port none or --dry-run.")
        else:
            port = auto_port() if args.port == "auto" else args.port
            if port:
                print("Using serial port:", port)
                ser = serial.Serial(port, cfg.baud, timeout=0.1)
                time.sleep(1.2)
                print("Handshake:", "OK" if handshake(ser) else "NO RESPONSE")
            else:
                print("No serial port found; continuing vision-only.")

    cap = cv2.VideoCapture(cfg.cam_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {cfg.cam_index} (try --cam 1)")

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Could not read camera frame")
    H, W = frame.shape[:2]

    # UI
    cv2.namedWindow("Demo", cv2.WINDOW_NORMAL)

    def noop(_): pass

    # Trackbars for ONE bowl ROI
    cv2.createTrackbar("bowl_x", "Demo", cfg.bowl_x, W - 1, noop)
    cv2.createTrackbar("bowl_y", "Demo", cfg.bowl_y, H - 1, noop)
    cv2.createTrackbar("bowl_w", "Demo", cfg.bowl_w, W, noop)
    cv2.createTrackbar("bowl_h", "Demo", cfg.bowl_h, H, noop)

    cv2.createTrackbar("food_thr", "Demo", cfg.food_thr, 80, noop)
    cv2.createTrackbar("min_food_px", "Demo", cfg.min_food_pixels, 50000, noop)

    # Hough tuning trackbars (optional)
    cv2.createTrackbar("h_dp_x10", "Demo", cfg.hough_dp_x10, 30, noop)
    cv2.createTrackbar("h_p1", "Demo", cfg.hough_param1, 300, noop)
    cv2.createTrackbar("h_p2", "Demo", cfg.hough_param2, 150, noop)
    cv2.createTrackbar("h_minR", "Demo", cfg.hough_minR, min(W, H) // 2, noop)
    cv2.createTrackbar("h_maxR", "Demo", cfg.hough_maxR, min(W, H) // 2, noop)

    empty_bowl = load_baseline(cfg)
    if empty_bowl is None:
        print("No baseline yet. Put EMPTY bowl in place and press 'b'.")
    else:
        print("Loaded baseline.")

    print("\n=== No-marker Bowl Zone Demo ===")
    print("Keys:")
    print("  b = capture EMPTY bowl baseline")
    print("  n = run ONE cycle (key mode)")
    print("  m = toggle key/auto mode")
    print("  f = try AUTO-FIND bowl circle (sets ROI)")
    print("  w = save config")
    print("  ESC = quit\n")

    stable_count = 0
    last_zone = None
    last_auto_cycle_t = 0.0

    def run_cycle(zone: str):
        if zone is None:
            print("[cycle] No zone.")
            return
        if args.dry_run or ser is None:
            print(f"[cycle] DRY-RUN target={zone}")
            return

        def must(cmd, timeout=3.0):
            resp, lines = send_and_wait(ser, cmd, timeout=timeout)
            if resp is None:
                print("[serial] TIMEOUT:", cmd, "last:", lines[-5:])
                return False
            if resp.startswith("ERR"):
                print("[serial]", resp, "for", cmd)
                return False
            print("[serial]", resp)
            return True

        if not must("ARM"): return
        if not must("TOOL SPOON"): return
        if not must(f"POSE {zone}"): return
        if not must("ACTION SCOOP", timeout=6.0): return
        if not must("POSE PRESENT"): return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        raw = frame
        vis = frame.copy()

        # Read UI
        bx = cv2.getTrackbarPos("bowl_x", "Demo")
        by = cv2.getTrackbarPos("bowl_y", "Demo")
        bw = cv2.getTrackbarPos("bowl_w", "Demo")
        bh = cv2.getTrackbarPos("bowl_h", "Demo")
        bx, by, bw, bh = clamp_roi(bx, by, bw, bh, W, H)

        cfg.food_thr = cv2.getTrackbarPos("food_thr", "Demo")
        cfg.min_food_pixels = cv2.getTrackbarPos("min_food_px", "Demo")

        cfg.hough_dp_x10 = max(1, cv2.getTrackbarPos("h_dp_x10", "Demo"))
        cfg.hough_param1 = max(1, cv2.getTrackbarPos("h_p1", "Demo"))
        cfg.hough_param2 = max(1, cv2.getTrackbarPos("h_p2", "Demo"))
        cfg.hough_minR = max(1, cv2.getTrackbarPos("h_minR", "Demo"))
        cfg.hough_maxR = max(cfg.hough_minR + 1, cv2.getTrackbarPos("h_maxR", "Demo"))

        cfg.bowl_x, cfg.bowl_y, cfg.bowl_w, cfg.bowl_h = bx, by, bw, bh
        bowl_roi = (bx, by, bw, bh)

        # Draw ROI + dividers
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        z = max(1, bw // 3)
        cv2.line(vis, (bx + z, by), (bx + z, by + bh), (0, 200, 0), 1)
        cv2.line(vis, (bx + 2*z, by), (bx + 2*z, by + bh), (0, 200, 0), 1)

        best_zone = None
        food_pixels = 0

        if empty_bowl is None:
            cv2.putText(vis, "Press 'b' with EMPTY bowl to capture baseline",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            bowl_now = crop(raw, bowl_roi)
            mask = food_mask(bowl_now, empty_bowl, thr=cfg.food_thr)
            food_pixels = int(np.count_nonzero(mask))
            best_zone, scores = pick_zone(mask)

            cv2.putText(vis, f"best={best_zone} food_px={food_pixels}",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        # Auto start: require same zone stable and enough food pixels
        if cfg.start_mode == "auto" and empty_bowl is not None and best_zone is not None:
            now = time.time()
            cooldown_ok = (now - last_auto_cycle_t) > cfg.auto_cooldown_s
            target_ok = food_pixels >= cfg.min_food_pixels

            if cooldown_ok and target_ok:
                if best_zone == last_zone:
                    stable_count += 1
                else:
                    stable_count = 1
                    last_zone = best_zone
            else:
                stable_count = 0
                last_zone = None

            cv2.putText(vis, f"AUTO stable={stable_count}/{cfg.stable_frames}",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

            if cooldown_ok and stable_count >= cfg.stable_frames:
                latched = best_zone
                stable_count = 0
                last_zone = None
                run_cycle(latched)
                last_auto_cycle_t = time.time()
        else:
            cv2.putText(vis, "KEY mode: press 'n' to run once (toggle with 'm')",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

        cv2.imshow("Demo", vis)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        elif k == ord('w'):
            save_config(cfg)
            print("[saved] bowl_nomarker_cfg.json")
        elif k == ord('b'):
            empty_bowl = crop(raw, bowl_roi).copy()
            save_baseline(cfg, empty_bowl)
            print("[baseline] captured empty bowl")
        elif k == ord('m'):
            cfg.start_mode = "auto" if cfg.start_mode == "key" else "key"
            stable_count = 0
            last_zone = None
            print("[mode]", cfg.start_mode)
        elif k == ord('n'):
            if empty_bowl is None:
                print("[cycle] no baseline; press 'b' first")
            elif best_zone is None or food_pixels < cfg.min_food_pixels:
                print("[cycle] no valid target (not enough food pixels)")
            else:
                run_cycle(best_zone)
        elif k == ord('f'):
            # Try auto-find circle and set ROI around it
            found = auto_find_bowl_circle(raw, cfg)
            if found is None:
                print("[find] no circle found; adjust hough params / lighting / angle")
            else:
                cx, cy, r = found
                # ROI around circle, slightly inset so we capture inside bowl
                pad = int(0.15 * r)
                x = cx - r + pad
                y = cy - r + pad
                w = 2 * (r - pad)
                h = 2 * (r - pad)
                x, y, w, h = clamp_roi(x, y, w, h, W, H)
                cv2.setTrackbarPos("bowl_x", "Demo", x)
                cv2.setTrackbarPos("bowl_y", "Demo", y)
                cv2.setTrackbarPos("bowl_w", "Demo", w)
                cv2.setTrackbarPos("bowl_h", "Demo", h)
                print(f"[find] circle cx={cx} cy={cy} r={r} -> ROI set")

    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()


if __name__ == "__main__":
    main()
