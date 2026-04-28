    #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

# 커스텀 메시지 및 유틸리티 임포트
from sc_interfaces.msg import PersonBbox
from sc_python.utils.deepsort_tracker import DeepSortTracker

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # 1. 파라미터 선언 (실시간 튜닝 가능)
        self.declare_parameter('model_path', 'models/best.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('img_size', 320)  # RPi4 최적화 사이즈

        # 2. 경로 설정 및 모델 로드
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        # 전체 경로가 아닐 경우 패키지 share 디렉토리나 현재 작업 디렉토리 기준 설정
        if not os.path.isabs(model_path):
            model_path = os.path.join(os.getcwd(), 'src/sc_python', model_path)
        
        self.model = YOLO(model_path)
        self.tracker = DeepSortTracker()  # 미리 만들어둔 DeepSORT 유틸리티 사용
        self.bridge = CvBridge()

        # 3. ROS2 인터페이스 설정
        self.image_sub = self.create_subscription(
            Image, '/webcam/image_raw', self.image_callback, 10)
        
        self.bbox_pub = self.create_publisher(PersonBbox, '/person_bbox', 10)

        self.get_logger().info(f'PersonTracker가 가동되었습니다. (모델: {model_path})')

    def image_callback(self, msg):
        # ROS2 이미지 메시지를 OpenCV 형식으로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # YOLOv8 추론
        conf_thres = self.get_parameter('conf_threshold').value
        img_sz = self.get_parameter('img_size').value
        results = self.model.predict(frame, imgsz=img_sz, conf=conf_thres, verbose=False)

        # DeepSORT 입력 형식으로 변환 [[x1, y1, w, h, conf, class_id], ...]
        detections = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls)
                # 사람(0) 또는 물체(2)만 추출하여 트래커로 전달
                if cls == 0 or cls == 2:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    w, h = x2 - x1, y2 - y1
                    conf = float(box.conf)
                    detections.append([x1, y1, w, h, conf, cls])

        # DeepSORT 업데이트
        tracks = self.tracker.update(detections, frame)

        # 추적 중인 타겟이 있다면 첫 번째 정보를 /person_bbox로 발행
        if tracks:
            target = tracks[0] 
            bbox_msg = PersonBbox()
            bbox_msg.header.stamp = self.get_clock().now().to_msg()
            bbox_msg.header.frame_id = "camera_link"
            bbox_msg.x = int(target[0])
            bbox_msg.y = int(target[1])
            bbox_msg.width = int(target[2])
            bbox_msg.height = int(target[3])
            bbox_msg.id = int(target[4])
            
            self.bbox_pub.publish(bbox_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# --- 수정 포인트: if __name__ == '__main__': 블록을 함수 밖(들여쓰기 0)으로 이동 ---
if __name__ == '__main__':
        main()