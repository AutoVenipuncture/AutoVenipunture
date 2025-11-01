# module3_demo_improved.py
# Improved arm alignment simulation with inverse kinematics + smooth motion
import cv2
import numpy as np
import time
import math

# ---------- CONFIG ----------
WIDTH, HEIGHT = 800, 600
BASE = np.array([WIDTH//2, HEIGHT-50])   # base of the arm (pixel coords)
L1 = 170   # link 1 length (pixels)
L2 = 120   # link 2 length (pixels)
GRIPPER_LEN = 30
FPS_DELAY = 20  # ms between frames
STEPS = 120     # interpolation steps for smooth motion
ELBOW = 'down'  # 'down' or 'up' elbow solution
# ----------------------------

def clamp(v, a, b): return max(a, min(b, v))

def solve_ik(xy):
    """
    Analytic 2-link planar inverse kinematics.
    Input: xy (tuple) target in pixel coordinates.
    Returns: (reachable(bool), theta1_deg, theta2_deg, elbow_choice)
    Angles are in degrees, where theta1 is base->link1 measured ccw, theta2 is link2 relative to link1.
    """
    x = xy[0] - BASE[0]
    y = BASE[1] - xy[1]  # invert y for math coords
    r = math.hypot(x, y)
    # reachability
    if r > (L1 + L2) or r < abs(L1 - L2):
        return (False, 0.0, 0.0, None)
    # law of cosines
    cos_q2 = clamp((r*r - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    q2a = math.acos(cos_q2)  # elbow-down
    q2b = -q2a                # elbow-up
    # compute base angle
    phi = math.atan2(y, x)
    k1 = L1 + L2 * math.cos(q2a)
    k2 = L2 * math.sin(q2a)
    q1a = phi - math.atan2(k2, k1)

    k1b = L1 + L2 * math.cos(q2b)
    k2b = L2 * math.sin(q2b)
    q1b = phi - math.atan2(k2b, k1b)

    if ELBOW == 'down':
        q1, q2 = q1a, q2a
        choice = 'down'
    else:
        q1, q2 = q1b, q2b
        choice = 'up'

    # convert to degrees for readability
    return (True, math.degrees(q1), math.degrees(q2), choice)

def forward_kinematics_deg(theta1_deg, theta2_deg):
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    x1 = BASE[0] + int(L1 * math.cos(t1))
    y1 = BASE[1] - int(L1 * math.sin(t1))
    x2 = x1 + int(L2 * math.cos(t1 + t2))
    y2 = y1 - int(L2 * math.sin(t1 + t2))
    return ( (x1,y1), (x2,y2) )

def ease_in_out(t):
    # smooth sinusoidal easing 0..1
    return 0.5 - 0.5 * math.cos(math.pi * clamp(t,0.0,1.0))

# ---------- MAIN ----------
def simulate(target):
    canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255
    # initial rest pose (angles)
    cur_a1, cur_a2 = 20.0, 40.0
    trail = []

    # solve IK for target
    reachable, tgt_a1, tgt_a2, choice = solve_ik(target)
    if not reachable:
        print("[!] Target not reachable by arm. Choose closer point.")
        # still show target and reachable circle
    else:
        print(f"[+] Solved IK -> theta1={tgt_a1:.1f}°, theta2={tgt_a2:.1f}° (elbow {choice})")

    # prepare interpolation frames
    frames = []
    for step in range(STEPS+1):
        t = ease_in_out(step / STEPS)
        a1 = cur_a1 + (tgt_a1 - cur_a1) * t if reachable else cur_a1
        a2 = cur_a2 + (tgt_a2 - cur_a2) * t if reachable else cur_a2
        frames.append((a1,a2))

    # run animation
    for (a1,a2) in frames:
        frame = canvas.copy()
        # draw workspace circle
        cv2.circle(frame, tuple(BASE), L1+L2, (230,230,230), 1)
        # draw base
        cv2.circle(frame, tuple(BASE), 8, (0,0,0), -1)
        # compute joints
        (j1, tip) = forward_kinematics_deg(a1, a2)
        # save tip to trail
        trail.append(tip)
        # draw trail
        for i in range(1, len(trail)):
            cv2.line(frame, trail[i-1], trail[i], (200,0,200), 2)
        # draw links
        cv2.line(frame, tuple(BASE), j1, (0,150,0), 6)
        cv2.line(frame, j1, tip, (0,100,200), 6)
        # draw joints
        cv2.circle(frame, j1, 6, (0,0,0), -1)
        cv2.circle(frame, tip, 6, (0,0,0), -1)
        # draw gripper direction (simple)
        grip_x = int(tip[0] + GRIPPER_LEN * math.cos(math.radians(a1 + a2)))
        grip_y = int(tip[1] - GRIPPER_LEN * math.sin(math.radians(a1 + a2)))
        cv2.line(frame, tip, (grip_x,grip_y), (50,50,50), 3)
        # draw target
        color_target = (0,0,255) if reachable else (0,165,255)
        cv2.circle(frame, tuple(target), 10, color_target, -1)
        cv2.putText(frame, "Target", (target[0]+12, target[1]-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_target, 2)

        # textual HUD
        cv2.putText(frame, f"Target coord: {target}", (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        cv2.putText(frame, f"Joint1: {a1:.1f} deg   Joint2: {a2:.1f} deg", (10,45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2)
        if reachable:
            cv2.putText(frame, "Alignment status: MOVING...", (10,75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,150), 2)
        else:
            cv2.putText(frame, "Alignment status: UNREACHABLE", (10,75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        cv2.imshow("Module 3 - SO100ARM Alignment Simulator", frame)
        key = cv2.waitKey(FPS_DELAY)
        if key == 27:  # ESC to quit
            break

    # final pause & success message
    if reachable:
        print("[+] Alignment reached. Final joint angles:")
        print(f"    theta1 = {tgt_a1:.2f} deg, theta2 = {tgt_a2:.2f} deg")
        print("    (These angles can be sent to Arduino/servo controller via serial.)")
        final_frame = frame.copy()
        cv2.putText(final_frame, "Alignment reached ✅", (10,110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,150,0), 3)
        cv2.imshow("Module 3 - SO100ARM Alignment Simulator", final_frame)
        cv2.waitKey(1200)
    else:
        cv2.waitKey(1200)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    # default demo target (you can replace this with coordinates from Module2)
    default_target = (BASE[0] + 50, BASE[1] + 50)
    simulate(default_target)