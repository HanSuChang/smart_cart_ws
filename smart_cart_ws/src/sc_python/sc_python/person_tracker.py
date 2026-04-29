#!/usr/bin/env python3
import os
import rclpy

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from ultralytics import YOLO

# 커스텀 메시지 및 유틸리티 임포트
from sc_interfaces.msg import PersonBbox
from sc_python.utils.deepsort_tracker import DeepSortTracker

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # 1. 파라미터 선언 (실시간 튜닝 가능)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('img_size', 320)        # RPi4 부하 고려해서 320 유지
        self.declare_parameter('image_topic', '/webcam/image_raw')  # USB 웹캠 토픽

        # 2. 경로 설정 및 모델 로드
        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value

        self.model = YOLO(self._resolve_model_path(model_path))
        self.tracker = DeepSortTracker()
        self.bridge = CvBridge()

        # 3. ROS2 인터페이스 설정
        # USB 웹캠 → v4l2_camera 노드 → /webcam/image_raw → 여기서 구독
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.bbox_pub = self.create_publisher(PersonBbox, '/person_bbox', 10)

        self.get_logger().info(
            f'PersonTracker 시작. model={model_path}, topic={image_topic}')

    def _resolve_model_path(self, model_path: str) -> str:
        """모델 파일 경로를 자동으로 탐색합니다."""
        if os.path.isabs(model_path):
            return model_path

        candidate_paths = [
            model_path,
            os.path.join(get_package_share_directory('sc_python'), 'models', model_path),
            os.path.join(os.getcwd(), model_path),
            os.path.join(os.getcwd(), 'src', 'sc_python', 'models', model_path),
        ]

        for path in candidate_paths:
            if os.path.exists(path):
                return path

        self.get_logger().warn(
            f'모델 파일을 찾지 못했습니다. ultralytics가 자동 다운로드합니다: {model_path}')
        return model_path

    def image_callback(self, msg):
        """USB 웹캠 이미지 수신 → YOLO 추론 → DeepSORT 추적 → /person_bbox 발행"""

        # ROS2 Image 메시지 → OpenCV BGR 프레임 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8n 추론 (사람 클래스만)
        conf_thres = self.get_parameter('conf_threshold').value
        img_sz = self.get_parameter('img_size').value
        results = self.model.predict(
            frame, imgsz=img_sz, conf=conf_thres, classes=[0], verbose=False)

        # DeepSORT 입력 형식으로 변환 [[x1, y1, w, h, conf, class_id], ...]
        detections = []
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                w, h = x2 - x1, y2 - y1
                conf = float(box.conf)
                detections.append([x1, y1, w, h, conf, 0])

        # DeepSORT 업데이트
        tracks = self.tracker.update(detections, frame)

        # 사람 찾았을 때 → is_valid=True로 발행
        if tracks:
            target = tracks[0]
            bbox_msg = PersonBbox()
            bbox_msg.header.stamp = self.get_clock().now().to_msg()
            bbox_msg.header.frame_id = 'camera_link'
            bbox_msg.x = int(target[0])
            bbox_msg.y = int(target[1])
            bbox_msg.width = int(target[2])
            bbox_msg.height = int(target[3])
            bbox_msg.track_id = int(target[4])
            bbox_msg.confidence = 1.0
            bbox_msg.is_valid = True
            self.bbox_pub.publish(bbox_msg)

        # 사람 못 찾았을 때 → is_valid=False로 발행 (C++이 타임아웃 처리)
        else:
            bbox_msg = PersonBbox()
            bbox_msg.header.stamp = self.get_clock().now().to_msg()
            bbox_msg.header.frame_id = 'camera_link'
            bbox_msg.is_valid = False
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


if __name__ == '__main__':
    main()