# save as vein_demo.py
import cv2
import numpy as np
import os

# === Setup output folder ===
out_dir = "output_steps"
os.makedirs(out_dir, exist_ok=True)

def save_step(name, img):
    path = os.path.join(out_dir, f"{name}.png")
    cv2.imwrite(path, img)
    print(f"Saved: {path}")

# === Load input ===
img = cv2.imread('images/hand.jpg')  # use your photo
if img is None:
    raise SystemExit("Place hand.jpg in same folder")

img = cv2.resize(img, (640, 480))
save_step("1_original", img)

# === Step 1: Grayscale + CLAHE ===
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
eq = clahe.apply(gray)
save_step("2_clahe", eq)

# === Step 2: Top-hat ===
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
tophat = cv2.morphologyEx(eq, cv2.MORPH_TOPHAT, kernel)
save_step("3_tophat", tophat)

# === Step 3: Invert ===
inv = cv2.bitwise_not(tophat)
save_step("4_inverted", inv)

# === Step 4: Vessel enhancement ===
def vesselness(img, sigmas=[1,2,3]):
    imgf = img.astype(np.float32) / 255.0
    v = np.zeros_like(imgf)
    for s in sigmas:
        ksize = int(6*s+1) if int(6*s+1)%2==1 else int(6*s+2)
        blur = cv2.GaussianBlur(imgf, (ksize, ksize), s)
        dxx = cv2.Sobel(blur, cv2.CV_32F, 2, 0, ksize=3)
        dyy = cv2.Sobel(blur, cv2.CV_32F, 0, 2, ksize=3)
        dxy = cv2.Sobel(blur, cv2.CV_32F, 1, 1, ksize=3)
        v += np.sqrt(np.maximum(0, dxx**2 + dyy**2))
    v = (v - v.min()) / (v.max() - v.min() + 1e-9)
    return (v*255).astype(np.uint8)

vessel = vesselness(inv)
save_step("5_vesselness", vessel)

# === Step 5: Threshold and cleanup ===
_, th = cv2.threshold(vessel, 30, 255, cv2.THRESH_BINARY)
th = cv2.morphologyEx(th, cv2.MORPH_OPEN, kernel)
th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
save_step("6_binary", th)

# === Step 6: Skeletonization ===
size = np.size(th)
skel = np.zeros(th.shape, np.uint8)
ret, img_bin = cv2.threshold(th, 127, 255, 0)
element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
done = False
while not done:
    eroded = cv2.erode(img_bin, element)
    temp = cv2.dilate(eroded, element)
    temp = cv2.subtract(img_bin, temp)
    skel = cv2.bitwise_or(skel, temp)
    img_bin = eroded.copy()
    if cv2.countNonZero(img_bin)==0:
        done = True
save_step("7_skeleton", skel)

# === Step 7: Contour + Coordinate mark ===
cnts, _ = cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if cnts:
    best = max(cnts, key=lambda c: cv2.arcLength(c, False))
    M = cv2.moments(best)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        cx, cy = 320, 240
    cv2.drawContours(img, [best], -1, (0,255,0), 1)
    cv2.circle(img, (cx,cy), 8, (0,0,255), -1)
    coord = (cx, cy)
else:
    coord = (320,240)
    cv2.circle(img, coord, 8, (0,0,255), -1)
save_step("8_final_output", img)

print("Target coordinate (x,y):", coord)
print(f"All processing steps saved in '{out_dir}/'")

# === Optional display ===
cv2.imshow("Final Vein Map", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
