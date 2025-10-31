import cv2
import numpy as np
import torch

class YOLOObjectRemover:
    def __init__(self, model_path, confidence=0.3):
        """
        เริ่มต้นระบบ YOLO Object Remover
        
        Args:
            model_path: path ไปยัง model file (.pt)
            confidence: ค่า confidence threshold สำหรับ detection (0-1)
        """
        print(f"กำลังโหลด YOLO model: {model_path}")
        
        # โหลด YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.conf = confidence
        
        # กำหนดคลาสที่ต้องการตรวจจับ (0=foot, 1=sign)
        # ตรวจสอบลำดับ class_name จากไฟล์ names ใน folder model
        self.target_classes = [0, 1]
        
        print(f"โหลด model สำเร็จ! Confidence threshold: {confidence}")
        print(f"จะทำการลบวัตถุใน class: {[self.model.names[i] for i in self.target_classes]}")
    
    def remove_objects(self, frame, detections, replace_color):
        """
        แทนที่ bounding boxes ของวัตถุที่กำหนดด้วยสีที่ต้องการ
        
        Args:
            frame: ภาพ frame ต้นฉบับ
            detections: list ของ detection results
            replace_color: สีที่จะใช้แทนที่ (B, G, R)
            
        Returns:
            frame ที่ถูกแทนที่วัตถุแล้ว
        """
        result_frame = frame.copy()
        
        for detection in detections:
            # ตรวจสอบว่าเป็นคลาสที่เราต้องการลบหรือไม่
            if int(detection[5]) in self.target_classes:
                x1, y1, x2, y2 = map(int, detection[:4])
                
                # ตรวจสอบขอบเขต
                height, width = frame.shape[:2]
                x1 = max(0, min(x1, width - 1))
                y1 = max(0, min(y1, height - 1))
                x2 = max(0, min(x2, width - 1))
                y2 = max(0, min(y2, height - 1))
                
                # แทนที่พื้นที่ bounding box ด้วยสีที่กำหนด
                result_frame[y1:y2, x1:x2] = replace_color
        
        return result_frame
    
    def process_video(self, video_path):
        """
        ประมวลผลวิดีโอหลัก
        
        Args:
            video_path: path ไปยังไฟล์วิดีโอ
        """
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            print(f"ไม่สามารถเปิดวิดีโอได้: {video_path}")
            return
        
        # กำหนดขนาดหน้าต่างให้พอดี
        width, height = 1280, 720
        cv2.namedWindow('YOLO Object Remover', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Object Remover', width, height)
        
        print(f"\nเริ่มประมวลผลวิดีโอ...")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("\nจบวิดีโอแล้ว")
                break
            
            # ปรับขนาดเฟรมให้เท่ากัน
            frame = cv2.resize(frame, (width, height))
            
            # ตรวจจับวัตถุด้วย YOLO
            results = self.model(frame)
            detections = results.pandas().xyxy[0].values
            
            # สีที่จะใช้แทนที่ (B, G, R)
            replace_color = (134, 135, 131)
            
            # ลบวัตถุที่ตรวจพบ
            result_frame = self.remove_objects(frame, detections, replace_color)
            
            # แสดงผล
            cv2.imshow('YOLO Object Remover', result_frame)
            
            # รอ 1 มิลลิวินาทีสำหรับเฟรมถัดไป
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # ปิดทุกอย่าง
        cap.release()
        cv2.destroyAllWindows()
        print(f"ประมวลผลเสร็จสิ้น!")

def main():
    """
    ฟังก์ชันหลักสำหรับเรียกใช้งาน
    """
    print("=== YOLO Object Remover ===")
    
    # ตั้งค่า paths - แก้ไขตรงนี้
    MODEL_PATH = "preprocess_img/model/best_aug1.pt"       # path ไปยัง model
    VIDEO_PATH = "D:/video/0003.mp4"        # path ไปยังวิดีโอ
    CONFIDENCE = 0.3                                         # confidence threshold
    
    # เริ่มระบบ
    remover = YOLOObjectRemover(MODEL_PATH, CONFIDENCE)
    remover.process_video(VIDEO_PATH)

if __name__ == "__main__":
    main()