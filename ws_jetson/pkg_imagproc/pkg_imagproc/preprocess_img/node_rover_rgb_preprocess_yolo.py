import cv2
import numpy as np
import torch

# ROI polygon (เฉพาะบริเวณที่จะตรวจจับ)
ROI_BASE = np.float32([[0, 500], [1280, 500], [750, 180], [400, 180]])

class YOLOObjectRemover:
    def __init__(self, model_path, confidence=0.3):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.conf = confidence
        # (0=foot, 1=sign)
        self.target_classes = [0, 1]

    def calculate_roi_avg_color(self, frame):
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [ROI_BASE.astype(np.int32)], 255)
        mean_val = cv2.mean(frame, mask=mask)[:3]  # (B, G, R)
        return tuple(map(int, mean_val))

    def remove_objects(self, frame, detections, replace_color):
        result_frame = frame.copy()
        mask_roi = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask_roi, [ROI_BASE.astype(np.int32)], 255)

        for detection in detections:
            if int(detection[5]) in self.target_classes:
                x1, y1, x2, y2 = map(int, detection[:4])

                # clip ให้อยู่ในเฟรม
                height, width = frame.shape[:2]
                x1 = max(0, min(x1, width - 1))
                y1 = max(0, min(y1, height - 1))
                x2 = max(0, min(x2, width - 1))
                y2 = max(0, min(y2, height - 1))

                # เฉพาะถ้า bounding box อยู่ใน ROI
                box_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                if mask_roi[box_center[1], box_center[0]] > 0:
                    result_frame[y1:y2, x1:x2] = replace_color

        return result_frame

    def process_video(self, video_path):
        cap = cv2.VideoCapture(video_path)

        if not cap.isOpened():
            print(f"Can't open the video: {video_path}")
            return

        width, height = 1280, 720
        cv2.namedWindow('YOLO Object Remover', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Object Remover', width, height)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nจบวิดีโอแล้ว")
                break

            frame = cv2.resize(frame, (width, height))

            # สีเฉลี่ยจาก ROI
            replace_color = self.calculate_roi_avg_color(frame)

            # ตรวจจับ
            results = self.model(frame)
            detections = results.pandas().xyxy[0].values

            # ลบเฉพาะใน ROI
            result_frame = self.remove_objects(frame, detections, replace_color)

            # แสดง ROI overlay สำหรับ debug
            overlay = result_frame.copy()
            cv2.polylines(overlay, [ROI_BASE.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.imshow('YOLO Object Remover', overlay)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        print(f"ประมวลผลเสร็จสิ้น!")


def main():
    MODEL_PATH = "preprocess_img/model/best_aug1.pt"
    VIDEO_PATH = "D:/video/0003.mp4"
    CONFIDENCE = 0.3

    remover = YOLOObjectRemover(MODEL_PATH, CONFIDENCE)
    remover.process_video(VIDEO_PATH)


if __name__ == "__main__":
    main()
