import cv2
import numpy as np
import matplotlib.pyplot as plt

# ================================
# 1. Threshold + Preprocess
# ================================
def preprocess_frame(frame_bgr, min_area=500):
    """แปลงภาพ BGR -> Binary Combined"""
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame = cv2.GaussianBlur(img_rgb, (5, 5), 0)
    frame = cv2.medianBlur(frame, 5)

    # แปลงภาพเป็น LAB (จาก frame ที่อ่านมาจากวิดีโอ)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0]
    A = lab[:, :, 1]
    B = lab[:, :, 2]

    # สร้าง mask สีเขียว (A ต่ำ, B สูง)
    green_mask = np.zeros_like(A, dtype=np.uint8)
    green_mask[(A < 120) & (B > 130)] = 1

    # สร้าง mask สีแดง (A สูง, B ต่ำ~กลาง)
    red_mask = np.zeros_like(A, dtype=np.uint8)
    red_mask[(A > 140) & (B < 140)] = 1

    # เอาสีเขียวออก เหลือเฉพาะพื้นที่สีแดง
    lab_no_green = lab.copy()
    lab_no_green[green_mask == 1] = 0

    # แปลงกลับเป็น RGB เพื่อทำ gradient
    img_rgb_no_green = cv2.cvtColor(lab_no_green, cv2.COLOR_LAB2RGB)
    gray = cv2.cvtColor(img_rgb_no_green, cv2.COLOR_RGB2GRAY)

    # Sobel x, y
    abs_sobel_x = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3))
    abs_sobel_y = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3))
    gradx = np.zeros_like(gray, dtype=np.uint8)
    grady = np.zeros_like(gray, dtype=np.uint8)
    gradx[(abs_sobel_x >= 50) & (abs_sobel_x <= 100)] = 1
    grady[(abs_sobel_y >= 50) & (abs_sobel_y <= 100)] = 1

    # Magnitude
    mag = np.sqrt(abs_sobel_x**2 + abs_sobel_y**2)
    mag = (mag / (np.max(mag)/255.0 + 1e-6)).astype(np.uint8)
    mag_binary = np.zeros_like(mag)
    mag_binary[(mag >= 30) & (mag <= 100)] = 1


    # Direction
    dir_binary = np.zeros_like(gray)
    absgraddir = np.arctan2(abs_sobel_y, abs_sobel_x + 1e-9)
    dir_binary[(absgraddir >= 0.7) & (absgraddir <= 1.3)] = 1


    # White pixel (เน้นเส้นสีขาว)
    white_binary = np.zeros_like(gray, dtype=np.uint8)
    white_binary[gray > 180] = 1

    # Combined
    combined = np.zeros_like(gray, dtype=np.uint8)
    combined[((gradx == 1) & (grady == 1)) |
             ((mag_binary == 1) & (dir_binary == 1)) |
             (white_binary == 1)] = 1

    # ทำ coutour เอาพื้นที่ที่ pixel น้อยๆในรูป ออก
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < 100:  # กำหนดขนาดพื้นที่ขั้นต่ำ
            cv2.drawContours(combined, [cnt], 0, 0, -1)

    return combined


# ================================
# 2. Perspective Transform
# ================================
def perspective_transform(binary, frame_size):
    """ทำ Perspective Transform"""
    h, w = frame_size

    ROI_BASE = np.float32([[0, 500],[1280, 500],[900, 200],[400, 200]])
    sx, sy = w/1280.0, h/720.0
    roi_points = np.float32([[p[0]*sx, p[1]*sy] for p in ROI_BASE])

    dst = np.float32([
        [w*0.25, h*1.0],
        [w*0.75, h*1.0],
        [w*0.75, h*0.0],
        [w*0.25, h*0.0]
    ])

    M = cv2.getPerspectiveTransform(roi_points, dst)
    Minv = cv2.getPerspectiveTransform(dst, roi_points)

    warped = cv2.warpPerspective(binary, M, (w, h))
    
    return warped, M, Minv


# ================================
# 3. Lane Finding (single center line)
# ================================
def find_center_line(binary_warped, nwindows=9, margin=100, minpix=50):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:, :], axis=0)
    base_x = int(np.argmax(histogram))  # จุดพีคเดียว

    window_height = binary_warped.shape[0] // nwindows
    nonzero = binary_warped.nonzero()
    nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])

    x_current = base_x
    lane_inds = []

    for window in range(nwindows):
        win_y_low  = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_x_low  = x_current - margin
        win_x_high = x_current + margin

        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
        lane_inds.append(good_inds)

        if len(good_inds) > minpix:
            x_current = int(np.mean(nonzerox[good_inds]))

    lane_inds = np.concatenate(lane_inds)
    x, y = nonzerox[lane_inds], nonzeroy[lane_inds]

    return x, y


# ================================
# 4. Fit single line & Params
# ================================
def compute_lane_params(binary_warped):
    x, y = find_center_line(binary_warped)
    H, W = binary_warped.shape[:2]

    results = {"theta": np.nan, "b": np.nan, "detected": False}

    if len(x) >= 50:
        m, b = np.polyfit(y, x, 1)  # y เป็นแกนตั้ง, x เป็นแกนนอน
        theta = np.degrees(np.arctan(m))
        b_centered = b - (W // 2)

        results = {"theta": theta, "b": b_centered, "detected": True}

    return results

# ================================
# 5. Full Pipeline
# ================================
def process_frame(frame_bgr):
    """อินพุต: frame BGR → เอาท์พุต: (theta, b, detected)"""
    binary = preprocess_frame(frame_bgr)
    warped, M, Minv = perspective_transform(binary, frame_bgr.shape[:2])
    params = compute_lane_params(warped)
    plot_lane_lines(frame_bgr, warped, Minv,
                params["theta"], params["b"], params["detected"])
    
    return params["theta"], params["b"], params["detected"]


def plot_lane_lines(frame_bgr, warped, Minv, theta, b, detected):
    H, W = warped.shape[:2]

    # ----- Step 1: เตรียมแกน y สำหรับ Bird’s eye view -----
    y_vals = np.linspace(0, H-1, num=H)
    if detected:
        m = np.tan(np.radians(theta))
        x_vals = m * y_vals + (b + W//2)  # ย้าย origin กลับจาก center
    else:
        x_vals = np.array([])
        y_vals = np.array([])

    # ----- Step 2: Bird's-eye view -----
    bird_eye_vis = cv2.cvtColor((warped*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    # ตัดขอบตามแนวยาว ฝั่งละ 100 px
    bird_eye_vis = bird_eye_vis[:, 100:W-100]
    warped = warped[:, 100:W-100]
    W = W - 200  # ปรับขนาด W หลังตัดขอบ
    x_vals = x_vals - 100  # ปรับตำแหน่ง x ให้ตรงกับภาพที่ถูกตัดขอบ
    # cv2.imshow("Bird's Eye View", bird_eye_vis)

    

    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.astype(np.int32)
        cv2.polylines(bird_eye_vis, [pts], isClosed=False, color=(0,0,255), thickness=3)

        # วาดแกน (0,0) ที่กึ่งกลางภาพ
        cv2.line(bird_eye_vis, (W//2,0), (W//2,H), (0,255,0), 1)
        cv2.line(bird_eye_vis, (0,H//2), (W,H//2), (255,0,0), 1)

    # ----- Step 3: Original view (inverse warp line กลับ) -----
    orig_vis = frame_bgr.copy()
    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.reshape(-1,1,2).astype(np.float32)
        pts = cv2.perspectiveTransform(pts, Minv)  

        pts_int = pts.astype(np.int32)
        cv2.polylines(orig_vis, [pts_int], isClosed=False, color=(0,0,255), thickness=3)

        # วาดแกน (0,0) กลางภาพ
        cv2.line(orig_vis, (W//2,0), (W//2,H), (0,255,0), 1)
        cv2.line(orig_vis, (0,H//2), (W,H//2), (255,0,0), 1)

    # Resize combined visualization to fit screen width
    # Resize orig_vis to match bird_eye_vis width
    if orig_vis.shape[1] != bird_eye_vis.shape[1]:
        orig_vis = cv2.resize(orig_vis, (bird_eye_vis.shape[1], bird_eye_vis.shape[0]))
    
    screen_width = 720  
    combined_vis = np.vstack([bird_eye_vis, orig_vis])
    h, w = combined_vis.shape[:2]
    scale = screen_width / w if w > screen_width else 1.0
    if scale < 1.0:
        combined_vis = cv2.resize(combined_vis, (int(w * scale), int(h * scale)))
    # cv2.imshow("Lane Detection (Bird's Eye View + Original)", combined_vis)
    




