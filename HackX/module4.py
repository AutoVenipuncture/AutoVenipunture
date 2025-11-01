import cv2
import numpy as np
import time
import math

# ----------------- CONFIG -----------------
WIDTH, HEIGHT = 800, 600
BASE = (WIDTH//2, HEIGHT-50)
TARGET = (BASE[0] + 50, BASE[1] - 200)
SUCTION_TIME = 3.0  # seconds
FPS_DELAY = 30
# ------------------------------------------

def draw_syringe(frame, level):
    """Draw a syringe filling up with blood."""
    x, y = 100, 200
    w, h = 60, 200
    cv2.rectangle(frame, (x, y), (x+w, y+h), (50,50,50), 2)
    fill_h = int(h * level)
    cv2.rectangle(frame, (x+3, y+h-fill_h), (x+w-3, y+h-3), (0,0,255), -1)
    cv2.putText(frame, "Blood Volume", (x-20, y-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
    cv2.putText(frame, f"{int(level*5)} ml", (x+5, y+h+25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

def draw_puncture(frame, tip, phase):
    """Show puncture motion and blood drops."""
    offset = int(5 * math.sin(phase * math.pi * 2))
    tip_moving = (tip[0], tip[1] + offset)
    cv2.circle(frame, tip_moving, 6, (0,0,0), -1)
    cv2.circle(frame, tip_moving, 3, (0,0,255), -1)

    # dripping blood effect
    if phase > 0.2:
        drop_y = int(tip[1] + 30 * (phase - 0.2))
        cv2.circle(frame, (tip[0], drop_y), 4, (0,0,255), -1)

def simulate_module4():
    # Create clean white background
    frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255

    # Show initial state
    cv2.circle(frame, TARGET, 10, (0,0,255), -1)
    cv2.putText(frame, "Target vein", (TARGET[0]+10, TARGET[1]-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)

    cv2.putText(frame, "Initializing blood extraction system...", (200, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
    cv2.imshow("Module 4 - Automated Blood Suction", frame)
    cv2.waitKey(1000)

    # Simulate suction
    start_time = time.time()
    while True:
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
        elapsed = time.time() - start_time
        phase = min(elapsed / SUCTION_TIME, 1.0)

        # Draw target and arm tip
        cv2.circle(frame, TARGET, 10, (0,0,255), -1)
        draw_puncture(frame, TARGET, phase)

        # Draw syringe fill level
        draw_syringe(frame, phase)

        # Simulate live data
        pressure = 101 + 15 * phase
        volume = 5 * phase
        cv2.putText(frame, f"Pressure: {pressure:.1f} kPa", (550, 200),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        cv2.putText(frame, f"Volume: {volume:.1f} ml", (550, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
        cv2.putText(frame, "Status: Extracting...", (550, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,150), 2)

        cv2.imshow("Module 4 - Automated Blood Suction", frame)
        key = cv2.waitKey(FPS_DELAY)
        if key == 27:
            break
        if phase >= 1.0:
            break

    # Final success frame
    final = frame.copy()
    cv2.putText(final, "Extraction Complete ✅", (250, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,150,0), 3)
    cv2.putText(final, "Module 4 completed successfully!", (230, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,100,0), 2)
    cv2.imshow("Module 4 - Automated Blood Suction", final)
    cv2.waitKey(1500)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    simulate_module4()