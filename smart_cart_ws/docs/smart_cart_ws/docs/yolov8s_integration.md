# YOLOv8s 연동 코드

## `smart_cart_ws/src/sc_python/sc_python/person_tracker.py`

```python
#!/usr/bin/env python3

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from sc_interfaces.msg import PersonBbox
from sc_python.utils.deepsort_tracker import DeepSortTracker


class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # 파라미터
        self.declare_parameter('model_path', 'yolov8s.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('img_size', 320)
        self.declare_parameter('image_topic', '/webcam/image_raw')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # 모델/트래커 초기화
        self.model = YOLO(self._resolve_model_path(model_path))
        self.tracker = DeepSortTracker()
        self.bridge = CvBridge()

        # ROS I/O
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.bbox_pub = self.create_publisher(PersonBbox, '/person_bbox', 10)

        self.get_logger().info(f'PersonTracker started. model={model_path}, image_topic={image_topic}')

    def _resolve_model_path(self, model_path: str) -> str:
        # 절대경로면 그대로 사용
        if os.path.isabs(model_path):
            return model_path

        # 상대경로 탐색 순서
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
            f'Model path not found in known locations. Using as-is: {model_path}'
        )
        return model_path

    def image_callback(self, msg):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        conf_thres = self.get_parameter('conf_threshold').value
        img_sz = self.get_parameter('img_size').value
        results = self.model.predict(frame, imgsz=img_sz, conf=conf_thres, verbose=False)

        # YOLO 결과 -> DeepSORT 입력
        detections = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls)
                if cls == 0:  # person class
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    w, h = x2 - x1, y2 - y1
                    conf = float(box.conf)
                    detections.append([x1, y1, w, h, conf, cls])

        tracks = self.tracker.update(detections, frame)

        # 첫 번째 트랙을 person_bbox로 발행
        if tracks:
            target = tracks[0]
            bbox_msg = PersonBbox()
            bbox_msg.header.stamp = self.get_clock().now().to_msg()
            bbox_msg.header.frame_id = 'camera_link'
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


if __name__ == '__main__':
    main()
```

## `smart_cart_ws/src/sc_bringup/config/tracker_params.yaml`

```yaml
person_tracker:
  ros__parameters:
    model_path: "yolov8s.pt"      # 모델 파일명 또는 경로
    conf_threshold: 0.5           # confidence threshold
    img_size: 320                 # YOLO inference size
    image_topic: "/webcam/image_raw"  # 입력 이미지 토픽
    show_debug: false
```

## 모델 파일 위치

```text
smart_cart_ws/src/sc_python/sc_python/models/yolov8s.pt
```

## 실행

```bash
cd smart_cart_ws
colcon build --packages-select sc_interfaces sc_python sc_cpp sc_bringup
source install/setup.bash
ros2 launch sc_bringup robot.launch.py
```
