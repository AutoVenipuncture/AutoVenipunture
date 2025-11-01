# save as vein_demo.py
import cv2
import numpy as np

# load
img = cv2.imread('images/hand.jpg')  # use your photo
if img is None:
    raise SystemExit("Place hand.jpg in same folder")

# resize for speed
img = cv2.resize(img, (640, 480))

# 1. Convert to grayscale and apply CLAHE (improves local contrast)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
eq = clahe.apply(gray)

# 2. Top-hat morphological to enhance bright/dark tubular structures
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
tophat = cv2.morphologyEx(eq, cv2.MORPH_TOPHAT, kernel)

# 3. Invert so veins (darker) become bright features for vessel-enhancement
inv = cv2.bitwise_not(tophat)

# 4. Frangi-like vessel enhancement approximation using Hessian (simple)
# Use Gaussian derivatives to boost line-like structures
def vesselness(img, sigmas=[1,2,3]):
    imgf = img.astype(np.float32) / 255.0
    v = np.zeros_like(imgf)
    for s in sigmas:
        ksize = int(6*s+1) if int(6*s+1)%2==1 else int(6*s+2)
        blur = cv2.GaussianBlur(imgf, (ksize, ksize), s)
        # second derivatives (approx)
        dxx = cv2.Sobel(blur, cv2.CV_32F, 2, 0, ksize=3)
        dyy = cv2.Sobel(blur, cv2.CV_32F, 0, 2, ksize=3)
        dxy = cv2.Sobel(blur, cv2.CV_32F, 1, 1, ksize=3)
        # vesselness measure (simplified)
        v += np.sqrt(np.maximum(0, dxx*2 + dyy*2))
    v = (v - v.min()) / (v.max() - v.min() + 1e-9)
    return (v*255).astype(np.uint8)

vessel = vesselness(inv)

# 5. Threshold and morphological clean-up
_,th = cv2.threshold(vessel, 30, 255, cv2.THRESH_BINARY)
th = cv2.morphologyEx(th, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)))

# 6. Skeletonize to emphasize lines
size = np.size(th)
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

# 7. Find contours on skeleton and pick the longest/widest (best candidate)
cnts, _ = cv2.findContours(skel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
if cnts:
    best = max(cnts, key=lambda c: cv2.arcLength(c, False))
    M = cv2.moments(best)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    else:
        cx,cy = 320,240
    cv2.drawContours(img, [best], -1, (0,255,0), 1)
    cv2.circle(img, (cx,cy), 8, (0,0,255), -1)
    coord = (cx,cy)
else:
    coord = (320,240)
    cv2.circle(img, coord, 8, (0,0,255), -1)

# show results
cv2.imshow("Original", img)
cv2.imshow("Vesselness", vessel)
cv2.imshow("Binary Veins", th)
cv2.imshow("Skeleton", skel)
print("Target coordinate (x,y):", coord)

cv2.waitKey(0)
cv2.destroyAllWindows()