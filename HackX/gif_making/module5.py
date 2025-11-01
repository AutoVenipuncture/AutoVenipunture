import cv2
import numpy as np
import time
import random
import math
import imageio  # <-- added for saving GIFs

# ---------------- CONFIG ----------------
WIDTH, HEIGHT = 900, 600
FPS_DELAY = 80
RUN_TIME = 10  # seconds for the demo
OUTPUT_GIF = "module5_dashboard.gif"
# ----------------------------------------

def draw_bar(frame, label, value, unit, pos_y, color):
    x, y, w, h = 100, pos_y, 250, 30
    cv2.rectangle(frame, (x, y), (x+w, y+h), (230,230,230), -1)
    fill = int(w * (value / 100))
    cv2.rectangle(frame, (x, y), (x+fill, y+h), color, -1)
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,0), 2)
    cv2.putText(frame, f"{label}: {value:.1f} {unit}", (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

def draw_module_status(frame, step):
    modules = [
        "Module 1: Vein Detection",
        "Module 2: Vein Mapping",
        "Module 3: Arm Alignment",
        "Module 4: Blood Suction",
        "Module 5: Monitoring Dashboard"
    ]
    x, y = 450, 100
    for i, m in enumerate(modules):
        color = (0,150,0) if i < step else (150,150,150)
        cv2.circle(frame, (x-20, y + i*60), 10, color, -1)
        cv2.putText(frame, m, (x, y + i*60 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

def draw_logs(frame, logs):
    x, y = 100, 400
    for i, log in enumerate(logs[-5:]):  # show last 5 logs
        cv2.putText(frame, log, (x, y + i*25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 2)

def simulate_dashboard():
    start_time = time.time()
    logs = [
        "[INFO] System initialized...",
        "[INFO] Modules calibrated.",
        "[INFO] Awaiting vitals data..."
    ]
    step = 5  # All modules completed
    frames = []  # store each frame for GIF

    while time.time() - start_time < RUN_TIME:
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        elapsed = time.time() - start_time
        t = elapsed / RUN_TIME

        # Simulate vitals
        hr = 75 + 10 * math.sin(t * 2 * math.pi)
        bp = 95 + 10 * math.cos(t * 1.5 * math.pi)
        temp = 36.5 + 0.5 * math.sin(t * math.pi)

        draw_bar(frame, "Heart Rate", hr, "bpm", 120, (255,100,100))
        draw_bar(frame, "Blood Pressure", bp, "mmHg", 180, (100,100,255))
        draw_bar(frame, "Temperature", temp , "°C", 240, (100,255,100))
        draw_module_status(frame, step)

        # Generate random log messages
        if random.random() < 0.1:
            msg = random.choice([
                "[INFO] Pressure within range.",
                "[INFO] Blood flow stable.",
                "[INFO] Auto-calibration complete.",
                "[ALERT] Minor vibration detected — corrected.",
                "[INFO] Extraction process normal."
            ])
            logs.append(msg)

        draw_logs(frame, logs)

        # Title & frame
        cv2.putText(frame, "Module 5 - Monitoring & Safety Dashboard", (160, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 3)
        cv2.rectangle(frame, (90, 90), (380, 280), (0,0,0), 2)

        # Show live (optional)
        cv2.imshow("Module 5 - Smart Feedback System", frame)
        key = cv2.waitKey(FPS_DELAY)
        if key == 27:
            break

        # Save frame to list (convert to RGB for GIF)
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

    # End frame
    final = frame.copy()
    cv2.putText(final, "System Check Complete ✅", (250, 500),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,150,0), 3)
    cv2.imshow("Module 5 - Smart Feedback System", final)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

    # Add final frame for smoother ending
    frames += [cv2.cvtColor(final, cv2.COLOR_BGR2RGB)] * 5

    # Save as GIF
    imageio.mimsave(OUTPUT_GIF, frames, fps=int(1000/FPS_DELAY))
    print(f"✅ Saved animation as {OUTPUT_GIF}")

if __name__ == "__main__":
    simulate_dashboard()
