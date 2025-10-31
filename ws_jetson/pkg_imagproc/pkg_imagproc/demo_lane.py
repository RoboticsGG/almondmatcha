# demo_lane.py
import os
import cv2
import argparse
import numpy as np
import matplotlib.pyplot as plt

# ------------------ Threshold helpers ------------------
def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
    else:
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
    denom = np.max(abs_sobel) if np.max(abs_sobel) > 0 else 1.0
    scaled = np.uint8(255 * abs_sobel / denom)
    binary = np.zeros_like(scaled, dtype=np.uint8)
    binary[(scaled >= thresh[0]) & (scaled <= thresh[1])] = 1
    return binary

def mag_thresh(img, sobel_kernel=3, mag_thresh=(0, 255)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    mag = np.sqrt(sx**2 + sy**2)
    denom = (np.max(mag) / 255.0) if np.max(mag) > 0 else 1.0
    mag = (mag / denom).astype(np.uint8)
    binary = np.zeros_like(mag, dtype=np.uint8)
    binary[(mag >= mag_thresh[0]) & (mag <= mag_thresh[1])] = 1
    return binary

def dir_threshold(img, sobel_kernel=3, thresh=(0, np.pi/2)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    absgraddir = np.arctan2(np.absolute(sy), np.absolute(sx) + 1e-9)
    binary = np.zeros_like(gray, dtype=np.uint8)
    binary[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1
    return binary

def hls_select(img, thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    s = hls[:, :, 2]
    binary = np.zeros_like(s, dtype=np.uint8)
    binary[(s > thresh[0]) & (s <= thresh[1])] = 1
    return binary

def combined_thresh(img_rgb):
    gradx      = abs_sobel_thresh(img_rgb, orient='x', sobel_kernel=3, thresh=(20, 100))
    grady      = abs_sobel_thresh(img_rgb, orient='y', sobel_kernel=3, thresh=(20, 100))
    mag_binary = mag_thresh(img_rgb, sobel_kernel=3, mag_thresh=(30, 100))
    dir_binary = dir_threshold(img_rgb, sobel_kernel=15, thresh=(0.7, 1.3))
    hls_binary = hls_select(img_rgb, thresh=(90, 255))
    gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
    white_binary = np.zeros_like(gray, dtype=np.uint8)
    white_binary[gray > 200] = 1
    combined = np.zeros_like(dir_binary, dtype=np.uint8)
    combined[((gradx == 1) & (grady == 1)) |
             ((mag_binary == 1) & (dir_binary == 1)) |
             (hls_binary == 1) |
             (white_binary == 1)] = 1
    return combined

# ------------------ Perspective helpers ------------------
# for 1280x720 resolution
def get_roi_points_scaled(frame_w, frame_h, roi_base, base_w=1280, base_h=720):
    sx = frame_w / float(base_w)
    sy = frame_h / float(base_h)
    return np.float32([[p[0]*sx, p[1]*sy] for p in roi_base])

def simple_roi_matrices(frame_w, frame_h, roi_points,
                        dst_left=0.25, dst_right=0.75, dst_top=0.0, dst_bottom=1.0): # dst_left=0.1, dst_right=0.9 this is for wide perspective
    dst = np.float32([
        [frame_w * dst_left,  frame_h * dst_bottom],
        [frame_w * dst_right, frame_h * dst_bottom],
        [frame_w * dst_right, frame_h * dst_top],
        [frame_w * dst_left,  frame_h * dst_top]
    ])
    M    = cv2.getPerspectiveTransform(roi_points, dst)
    Minv = cv2.getPerspectiveTransform(dst, roi_points)
    return M, Minv

def perspective_transform(img, M):
    h, w = img.shape[:2]
    return cv2.warpPerspective(img, M, (w, h))

# ------------------ Lane finding ------------------
def find_right_lane_pixels(binary_warped, nwindows=9, margin=100, minpix=50):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    out_img = np.dstack([binary_warped, binary_warped, binary_warped]) * 255
    midpoint = histogram.shape[0] // 2
    rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint)
    window_height = binary_warped.shape[0] // nwindows

    nonzero  = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    rightx_current = rightx_base
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low  = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_x_low  = rightx_current - margin
        win_x_high = rightx_current + margin

        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                     (nonzerox >= win_x_low)  & (nonzerox < win_x_high)).nonzero()[0]
        right_lane_inds.append(good_inds)
        if len(good_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_inds]))

    right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) else np.array([], dtype=int)
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    if rightx.size > 0:
        out_img[righty, rightx] = [0, 0, 255]
    return rightx, righty, out_img

def fit_right_polynomial(binary_warped, spacing_px=30, nwindows=9, margin=100, minpix=50):
    rightx, righty, out_img = find_right_lane_pixels(binary_warped, nwindows, margin, minpix)
    if rightx.size < 50:
        return {
            'right_fit': None, 'ploty': None, 'right_fitx': None,
            'points_bev': [], 'points_orig': [], 'vis': out_img
        }

    right_fit = np.polyfit(righty, rightx, 2)   # x = a*y^2 + b*y + c
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    step = max(1, int(spacing_px))
    y_idx = np.arange(0, len(ploty), step, dtype=int)
    pts_bev = [(int(right_fitx[i]), int(ploty[i])) for i in y_idx
               if 0 <= right_fitx[i] < binary_warped.shape[1]]

    vis = out_img.copy()
    for (x, y) in pts_bev:
        cv2.circle(vis, (x, y), 5, (255, 0, 0), -1)
    return {
        'right_fit': right_fit, 'ploty': ploty, 'right_fitx': right_fitx,
        'points_bev': pts_bev, 'points_orig': [], 'vis': vis
    }

# ------------------ Drawing & angle ------------------
def draw_lane(undist_rgb, binary_warped, left_fitx, right_fitx, ploty, Minv):
    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    pts_left  = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))
    cv2.fillPoly(color_warp, np.int32([pts]), (0, 255, 0))
    h, w = undist_rgb.shape[:2]
    newwarp = cv2.warpPerspective(color_warp, Minv, (w, h))
    result = cv2.addWeighted(undist_rgb, 1.0, newwarp, 0.3, 0)
    return result

def line_xy_from_points(points):
    if points is None or len(points) < 2:
        raise ValueError("need >= 2 points")
    ys = np.array([p[1] for p in points], dtype=np.float64)
    xs = np.array([p[0] for p in points], dtype=np.float64)
    m, b = np.polyfit(ys, xs, 1)  # x = m*y + b
    return float(m), float(b)

def angle_vs_vertical_top_sign_vis(result_dict, warped_binary):
    """
    ฟิตเส้น x = m*y + b จากจุด BEV ของเส้นขวา แล้วคำนวณมุมกับแกนตั้ง (เหมือน notebook)
    พร้อมวาดเส้นและแสดงค่ามุมบนภาพ (BEV)
    """
    pts = result_dict['points_bev']
    H, W = warped_binary.shape[:2]
    img_vis = (np.dstack([warped_binary, warped_binary, warped_binary]) * 255).astype(np.uint8)
    if pts is None or len(pts) < 2 or result_dict['right_fit'] is None:
        cv2.putText(img_vis, 'Right lane not found', (40, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2, cv2.LINE_AA)
        return img_vis, None
    # ฟิตเส้น x = m*y + b
    ys = np.array([p[1] for p in pts], dtype=np.float64)
    xs = np.array([p[0] for p in pts], dtype=np.float64)
    m, b = np.polyfit(ys, xs, 1)  # x = m*y + b
    # มุมกับแกนตั้ง (signed)
    angle_signed_deg = (np.degrees(np.arctan(m)))
    angle_abs_deg = abs(angle_signed_deg)
    # วาดเส้นที่ฟิต
    y0, y1 = 0, H-1
    x0 = int(np.clip(m*y0 + b, 0, W-1))
    x1 = int(np.clip(m*y1 + b, 0, W-1))
    cv2.line(img_vis, (x0, y0), (x1, y1), (0, 255, 0), 3)
    # วาดเส้นตั้งกลางภาพ
    cx = W // 2
    cv2.line(img_vis, (cx, 0), (cx, H-1), (0, 255, 255), 2)
    # จุดตัดกับเส้นตั้ง (ถ้ามี)
    intersection = None
    if abs(m) > 1e-9:
        y_intersect = (cx - b) / m
        if 0 <= y_intersect < H:
            intersection = (int(cx), int(y_intersect))
            cv2.circle(img_vis, intersection, 6, (0, 0, 255), -1)
    # จุดกึ่งกลางภาพ
    cv2.circle(img_vis, (W//2, H//2), 5, (255, 0, 0), -1)
    
    b = b - cx  # ปรับ b ให้เป็นระยะจากกึ่งกลางภาพ
    
    # # แสดงค่ามุม
    # txt1 = f"x = {m:.6f} * y + {b:.6f}"
    # txt2 = f"Angle to VERTICAL (signed): {angle_signed_deg:.3f} deg"
    # txt3 = f"Angle to VERTICAL (absolute): {angle_abs_deg:.3f} deg"
    # cv2.putText(img_vis, txt1, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
    # cv2.putText(img_vis, txt2, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
    # cv2.putText(img_vis, txt3, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
    
    return img_vis, angle_signed_deg

# ------------------ Per-frame pipeline ------------------
def process_frame(bgr_frame, M, Minv, offset_px=1000):
    rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    combined_binary = combined_thresh(rgb)
    warped_binary = perspective_transform(combined_binary, M)
    result = fit_right_polynomial(warped_binary, spacing_px=30, nwindows=9, margin=100, minpix=50)

    H, W = warped_binary.shape[:2]
    angle_val = np.nan
    m_yx = np.nan
    b_yx = np.nan
    detected = False

    # visualization default = original frame
    vis = bgr_frame.copy()

    if result['right_fit'] is not None:
        detected = True
        right_fit  = result['right_fit']
        ploty      = result['ploty']
        right_fitx = result['right_fitx']

        # สร้างเส้นซ้ายจากเส้นขวา
        left_fit = right_fit.copy()
        left_fit[2] -= offset_px
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]

        # overlay lane
        overlay_rgb = draw_lane(cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB),
                                warped_binary, left_fitx, right_fitx, ploty, Minv)
        vis = cv2.cvtColor(overlay_rgb, cv2.COLOR_RGB2BGR)

        # angle + m,b
        try:
            m_yx, b_yx = line_xy_from_points(result['points_bev'])
            angle_val = float(np.degrees(np.arctan(m_yx)))
        except Exception:
            m_yx, b_yx, angle_val = np.nan, np.nan, np.nan

        # put small angle view on bottom-right
        angle_vis, _ = angle_vs_vertical_top_sign_vis(result, warped_binary)
        thumb = cv2.resize(angle_vis, (vis.shape[1]//3, vis.shape[0]//3))
        th, tw = thumb.shape[:2]
        vis[-th-10:-10, -tw-10:-10] = thumb
    else:
        detected = False

    # ปรับ b ให้เป็นระยะจากกึ่งกลางภาพ
    b_yx = b_yx - (W // 2.0) if not np.isnan(b_yx) else b_yx

    print('---------------------------------')
    print(f"x = {m_yx:.6f} * y + {b_yx:.6f}")
    print(f"(signed): {angle_val:.3f} deg")
    print(f"detected: {detected}")
    
    return vis, angle_val, b_yx, detected

def show_image_with_grid(img):
    """แสดงภาพพร้อมกริดและตัวเลขช่วยในการเลือกจุด"""
    height, width = img.shape[:2]
    fig, ax = plt.subplots(1, 1, figsize=(15, 10))
    ax.imshow(img)
    for y in range(0, height, height//8):
        ax.axhline(y=y, color='yellow', alpha=0.5, linewidth=1)
        ax.text(10, y, f'y={y}', color='yellow', fontsize=10, fontweight='bold')
    for x in range(0, width, width//8):
        ax.axvline(x=x, color='yellow', alpha=0.5, linewidth=1)
        ax.text(x, height-30, f'x={x}', color='yellow', fontsize=10, fontweight='bold')
    ax.set_title('Use these coordinates to adjust your ROI points')
    ax.grid(True, alpha=0.3)
    plt.show()
    

def draw_grid_on_image(img, grid_rows=8, grid_cols=8, color=(0,255,255), thickness=1):
        h, w = img.shape[:2]
        step_y = h // grid_rows
        step_x = w // grid_cols
        img_grid = img.copy()
        # draw horizontal lines
        for y in range(0, h, step_y):
            cv2.line(img_grid, (0, y), (w, y), color, thickness)
            cv2.putText(img_grid, f'y={y}', (10, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        # draw vertical lines
        for x in range(0, w, step_x):
            cv2.line(img_grid, (x, 0), (x, h), color, thickness)
            cv2.putText(img_grid, f'x={x}', (x+5, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return img_grid
# ------------------ Main ------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", type=str,  help="input video path (.mp4)")
    ap.add_argument("--image", type=str, help="input image path (.jpg/.png)")
    ap.add_argument("--outtxt", type=str, default="outputs/metrics.txt", help="output text file (single)")
    ap.add_argument("--interval", type=int, default=1, help="process every Nth frame (>=1)")
    ap.add_argument("--offset", type=int, default=1000, help="left lane = right lane offset px")
    ap.add_argument("--window", type=str, default="Lane Demo (q=quit, space=pause)", help="window name")
    ap.add_argument("--grid", action="store_true", help="show grid on image")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.outtxt) or ".", exist_ok=True)

    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise RuntimeError(f"Cannot open image: {args.image}")

        if args.grid:
            show_image_with_grid(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            return
        H, W = bgr.shape[:2]
        ROI_BASE = np.float32([[0-500, 450],[1280+100, 450],[880, 270],[280, 270]])
        roi_points = get_roi_points_scaled(W, H, roi_base=ROI_BASE)
        M, Minv = simple_roi_matrices(W, H, roi_points)
        vis, angle_deg, b_yx, status = process_frame(bgr, M, Minv, offset_px=args.offset)
        cv2.imshow(args.window, vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    cap = cv2.VideoCapture(args.input)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {args.input}")

    ret, first = cap.read()
    if not ret:
        raise RuntimeError("No frames in the input video")
    ROI_BASE = np.float32([[0-500, 450],[1280+100, 450],[880, 270],[280, 270]])
    H, W = first.shape[:2]

    roi_points = get_roi_points_scaled(W, H, roi_base=ROI_BASE)
    M, Minv    = simple_roi_matrices(W, H, roi_points)

    # reset to frame 0
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    # open log (CSV-like header not requested; write only requested values per frame)
    f = open(args.outtxt, "w", encoding="utf-8")

    frame_idx = 0
    paused = False
    cv2.namedWindow(args.window, cv2.WINDOW_NORMAL)

    try:
        while True:
            if not paused:
                ret, bgr = cap.read()
                if not ret:
                    break

                if frame_idx % max(1, args.interval) == 0:
                    vis, angle_deg, m_yx, b_yx = process_frame(bgr, M, Minv, offset_px=args.offset)

                    # log line: angle_to_vertical_signed_deg, m_yx, b_yx
                    def fmt(x):
                        return "nan" if (x is None or (isinstance(x, float) and np.isnan(x))) else f"{x:.6f}"
                    f.write(f"{fmt(angle_deg)},{fmt(m_yx)},{fmt(b_yx)}\n")
                    print(f"Frame {frame_idx}: angle={angle_deg}, m_yx={m_yx}, b_yx={b_yx}")

                    cv2.imshow(args.window, vis)
                frame_idx += 1

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 32:  # space
                paused = not paused

    finally:
        f.close()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
