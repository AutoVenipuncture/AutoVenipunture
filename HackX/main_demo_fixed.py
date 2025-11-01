#!/usr/bin/env python3
# main_demo_fixed.py
# Corrected integrated demo for Modules 1 -> 5
# Place hand.jpg in same folder as this script (or edit HAND_IMG path).
# Run: python main_demo_fixed.py
# Requires: opencv-python, numpy

import cv2, os, time, math, numpy as np

# --- Paths ---
BASE_DIR = os.path.dirname(__file__)
HAND_IMG = os.path.join(BASE_DIR, "images/hand.jpg")
VEIN_MAP = os.path.join(BASE_DIR, "vein_map.jpg")

# ---------------- Module 1 & 2: Vein enhancement + coordinate selection ----------------
def module1_and_2_process(hand_path):
    print("[Module1+2] Loading image:", hand_path)
    img = cv2.imread(hand_path)
    if img is None:
        raise SystemExit(f"Cannot find image at {hand_path} - place hand.jpg there.")
    img = cv2.resize(img, (640,480))

    # Contrast enhancement
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    eq = clahe.apply(gray)

    # Top-hat morphological to emphasize vessels
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    tophat = cv2.morphologyEx(eq, cv2.MORPH_TOPHAT, kernel)
    inv = cv2.bitwise_not(tophat)

    # Simple vesselness-like filter (approximated)
    def vesselness(imgf, sigmas=[1,2,3]):
        imgf = imgf.astype(np.float32) / 255.0
        v = np.zeros_like(imgf)
        for s in sigmas:
            ksize = int(6*s+1)
            if ksize % 2 == 0: ksize += 1
            blur = cv2.GaussianBlur(imgf, (ksize, ksize), s)
            dxx = cv2.Sobel(blur, cv2.CV_32F, 2, 0, ksize=3)
            dyy = cv2.Sobel(blur, cv2.CV_32F, 0, 2, ksize=3)
            # combine
            v += np.sqrt(np.maximum(0, dxx*2 + dyy*2))
        if v.max() - v.min() > 1e-9:
            v = (v - v.min()) / (v.max() - v.min())
        return (v*255).astype(np.uint8)

    vessel = vesselness(inv)
    _, th = cv2.threshold(vessel, 30, 255, cv2.THRESH_BINARY)
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
    th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))

    # Skeletonize the binary map
    skel = np.zeros(th.shape, np.uint8)
    ret, img_bin = cv2.threshold(th, 127, 255, 0)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
    done = False
    while(not done):
        eroded = cv2.erode(img_bin, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img_bin, temp)
        skel = cv2.bitwise_or(skel, temp)
        img_bin = eroded.copy()
        if cv2.countNonZero(img_bin)==0:
            done = True

    # Find contours on skeleton and pick the longest as best vein
    cnts, _ = cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out_img = img.copy()
    if cnts:
        best = max(cnts, key=lambda c: cv2.arcLength(c, False))
        M = cv2.moments(best)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx, cy = out_img.shape[1]//2, out_img.shape[0]//2
        cv2.drawContours(out_img, [best], -1, (0,255,0), 1)
        cv2.circle(out_img, (cx,cy), 8, (0,0,255), -1)
        coord = (cx, cy)
    else:
        coord = (out_img.shape[1]//2, out_img.shape[0]//2)
        cv2.circle(out_img, coord, 8, (0,0,255), -1)

    # Create a canvas (800x600) and center the processed image so Module3 has consistent background
    canvas = np.ones((600,800,3), dtype=np.uint8)*255
    h,w = out_img.shape[:2]
    xoff = (800 - w)//2
    yoff = (600 - h)//2
    canvas[yoff:yoff+h, xoff:xoff+w] = out_img

    # save vein map
    cv2.imwrite(VEIN_MAP, canvas)
    print(f"[Module1+2] Vein map written to {VEIN_MAP}. Selected coord (local): {coord}")

    # show visuals for judges
    cv2.imshow("Original (resized)", img)
    cv2.imshow("Vesselness (approx)", vessel)
    cv2.imshow("Binary veins", th)
    cv2.imshow("Skeleton", skel)
    cv2.imshow("Annotated selection", out_img)
    print("[Module1+2] Press any key to continue to Module 3...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # convert coord to canvas coords (for module3)
    full_coord = (coord[0] + xoff, coord[1] + yoff)
    return full_coord

# ---------------- Module 3: Robust CCD IK reaching target ----------------
def module3_reach_target(target):
    print("[Module3] Running CCD IK to reach target:", target)
    WIDTH, HEIGHT = 800, 600
    BASE = np.array([WIDTH//2, HEIGHT-50], dtype=float)
    LINK_LENGTHS = [100, 90, 70, 50, 40, 30]
    NUM_JOINTS = len(LINK_LENGTHS)
    # sensible starting angles (deg)
    angles = np.array([10.0, 20.0, -5.0, 10.0, -8.0, 5.0], dtype=float)

    def forward_fk(angles_deg):
        pts = [tuple(BASE.astype(int))]
        x,y = float(BASE[0]), float(BASE[1])
        total = 0.0
        for L, a in zip(LINK_LENGTHS, angles_deg):
            total += math.radians(a)
            x += L * math.cos(total)
            y -= L * math.sin(total)
            pts.append((int(round(x)), int(round(y))))
        return pts

    def distance(a,b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def ccd_iteration(target_pt, angles_deg, step_fraction=0.45):
        pts = forward_fk(angles_deg)
        for i in reversed(range(NUM_JOINTS)):
            joint = np.array(pts[i], dtype=float)
            end = np.array(pts[-1], dtype=float)
            to_end = end - joint
            to_target = np.array(target_pt, dtype=float) - joint
            if np.linalg.norm(to_end) < 1e-6 or np.linalg.norm(to_target) < 1e-6:
                continue
            a1 = math.atan2(to_end[1], to_end[0])
            a2 = math.atan2(to_target[1], to_target[0])
            delta = math.degrees(a2 - a1)
            # apply fraction for smooth convergence
            angles_deg[i] += delta * step_fraction
            # normalize
            angles_deg[i] = ((angles_deg[i] + 180) % 360) - 180
        return angles_deg

    # load vein map background created earlier
    bg = cv2.imread(VEIN_MAP)
    if bg is None:
        bg = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8)*255

    trail = []
    max_frames = 800
    for frame_idx in range(max_frames):
        # do several small CCD passes to converge faster per frame
        for _ in range(3):
            angles = ccd_iteration(target, angles, step_fraction=0.5)
        pts = forward_fk(angles)
        tip = np.array(pts[-1], dtype=float)
        trail.append(tuple(tip.astype(int)))

        frame = bg.copy()
        # draw trail
        for i in range(1, len(trail)):
            cv2.line(frame, trail[i-1], trail[i], (200,0,200), 2)
        # draw arm links and joints
        for i in range(len(pts)-1):
            color = (max(0, 50 + i*30), max(0, 200 - i*18), max(0, 100 + i*25))
            cv2.line(frame, pts[i], pts[i+1], color, 6, lineType=cv2.LINE_AA)
            cv2.circle(frame, pts[i], 6, (10,10,10), -1)
        # draw end effector and target
        cv2.circle(frame, tuple(tip.astype(int)), 8, (255,255,0), -1)
        cv2.circle(frame, tuple(target), 10, (0,0,255), -1)
        dist = distance(tip, target)
        status = "ALIGNED" if dist < 5 else f"MOVING ({dist:.1f}px)"
        cv2.putText(frame, f"Status: {status}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,150,0) if dist<5 else (0,0,200), 2)
        cv2.putText(frame, f"Frame: {frame_idx}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        cv2.imshow("Module3 - SO100ARM Alignment (fixed)", frame)
        key = cv2.waitKey(15)
        if key == 27:
            cv2.destroyAllWindows(); return False
        if dist < 5:
            cv2.putText(frame, "Alignment reached ✅", (200,100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,150,0), 3)
            cv2.imshow("Module3 - SO100ARM Alignment (fixed)", frame)
            cv2.waitKey(700)
            cv2.destroyAllWindows()
            print("[Module3] Alignment success.")
            # return final angles and tip for potential use
            return True
    cv2.destroyAllWindows()
    print("[Module3] Failed to converge within frames.")
    return False

# ---------------- Module 4: Blood suction simulation ----------------
def module4_suction_sim():
    print("[Module4] Starting suction simulation...")
    WIDTH, HEIGHT = 800, 600
    TARGET = (WIDTH//2 + 50, HEIGHT//2 - 100)
    SUCTION_TIME = 3.0
    start_time = time.time()
    while True:
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        elapsed = time.time() - start_time
        phase = min(elapsed / SUCTION_TIME, 1.0)
        offset = int(5 * math.sin(phase * math.pi * 4))
        tip_moving = (TARGET[0], TARGET[1] + offset)
        cv2.circle(frame, TARGET, 10, (0,0,255), -1)
        cv2.circle(frame, tip_moving, 4, (0,0,0), -1)
        cv2.circle(frame, tip_moving, 2, (0,0,255), -1)
        # syringe fill
        x,y,w,h = 100,200,60,200
        cv2.rectangle(frame, (x,y),(x+w,y+h),(50,50,50),2)
        fill_h = int(h*phase)
        cv2.rectangle(frame, (x+3,y+h-fill_h),(x+w-3,y+h-3),(0,0,255), -1)
        # live readout
        pressure = 101 + 18*phase
        volume = 5*phase
        cv2.putText(frame, f"Pressure: {pressure:.1f} kPa", (520, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        cv2.putText(frame, f"Volume: {volume:.2f} ml", (520, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        cv2.putText(frame, "Status: Extracting...", (520, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,150), 2)
        cv2.imshow("Module4 - Blood Suction (fixed)", frame)
        key = cv2.waitKey(30)
        if key == 27:
            cv2.destroyAllWindows(); return False
        if phase >= 1.0:
            break
    final = frame.copy()
    cv2.putText(final, "Extraction Complete ✅", (220,120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,150,0), 3)
    cv2.imshow("Module4 - Blood Suction (fixed)", final)
    cv2.waitKey(900)
    cv2.destroyAllWindows()
    print("[Module4] Suction simulation done.")
    return True

# ---------------- Module 5: Monitoring Dashboard ----------------
def module5_dashboard():
    print("[Module5] Showing monitoring dashboard...")
    WIDTH, HEIGHT = 900, 600
    RUN_TIME = 8
    start_time = time.time()
    logs = ["[INFO] System initialized.", "[INFO] All modules OK."]
    while time.time() - start_time < RUN_TIME:
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        t = (time.time() - start_time) / RUN_TIME
        hr = 75 + 8 * math.sin(t*2*math.pi)
        bp = 95 + 8 * math.cos(t*1.5*math.pi)
        temp = 36.6 + 0.4 * math.sin(t*math.pi)
        # bars
        def draw_bar(frame, label, val, y, color):
            x,w,h = 100,250,28
            cv2.rectangle(frame, (x,y),(x+w,y+h),(230,230,230), -1)
            fill = int(w * (val/120))  # normalize to 120 for visuals
            cv2.rectangle(frame, (x,y),(x+fill,y+h), color, -1)
            cv2.rectangle(frame, (x,y),(x+w,y+h),(0,0,0),2)
            cv2.putText(frame, f"{label}: {val:.1f}", (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0),2)
        draw_bar(frame, "Heart Rate (bpm)", hr, 120, (200,80,80))
        draw_bar(frame, "Blood Pressure (mmHg)", bp, 180, (80,80,200))
        draw_bar(frame, "Temperature (C)", temp, 240, (80,200,100))
        # module status
        modules = ["Module1 Vein", "Module2 Map", "Module3 Arm", "Module4 Suction", "Module5 Monitor"]
        for i, m in enumerate(modules):
            color = (0,150,0) if True else (180,180,180)
            cv2.circle(frame, (500,100 + i*50), 10, color, -1)
            cv2.putText(frame, m, (520, 105 + i*50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0),2)
        # logs
        for i, log in enumerate(logs[-5:]):
            cv2.putText(frame, log, (100, 360 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0),2)
        cv2.putText(frame, "Module5 - Monitoring Dashboard", (180,40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0),3)
        cv2.imshow("Module5 - Dashboard (fixed)", frame)
        if cv2.waitKey(80) == 27:
            break
    cv2.destroyAllWindows()
    print("[Module5] Dashboard finished.")

# ---------------- Main sequence ----------------
def main():
    print("=== Integrated SO100ARM Demo (Modules 1->5) — FIXED ===")
    target = module1_and_2_process(HAND_IMG)
    print("[MAIN] Target on full canvas:", target)
    ok = module3_reach_target(target)
    if not ok:
        print("[MAIN] Module3 failed. Exiting.")
        return
    ok2 = module4_suction_sim()
    if not ok2:
        print("[MAIN] Module4 aborted. Exiting.")
        return
    module5_dashboard()
    print("[MAIN] Demo finished.")

if __name__ == '__main__':
    main()
