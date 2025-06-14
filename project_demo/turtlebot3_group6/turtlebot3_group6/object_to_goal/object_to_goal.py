import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import numpy as np
import time
import math
from transforms3d.euler import euler2quat

CLASS_NAMES = ['redbox', 'bluebox', 'blackbox']

OBJECT_TO_GOAL = {
    "bluebox": (1.5, 0.6, 3.14),   # x, y, theta (radian)
    "redbox": (3.5, 0.55, math.pi / 2),
    "blackbox": (2.65, 0.05, math.pi)
}
CONFIDENCE_THRESHOLD = 0.5
TIMER_DURATION = 3.0

# HSV 색상 범위 정의
COLOR_RANGES = {
    'redbox': [
        ((0, 100, 50), (10, 255, 255)),
        ((160, 100, 50), (180, 255, 255))
    ],
    'bluebox': [
        ((100, 100, 50), (130, 255, 255))
    ],
    'blackbox': [
        ((0, 0, 0), (180, 255, 50))
    ]
}

class ObjectRecognizerCombined(Node):
    def __init__(self):
        super().__init__('object_recognizer_combined')

        # 최대 3개 목표지 퍼블리셔
        self.goal_publishers = [
            self.create_publisher(PoseStamped, 'detected_goal_1', 10),
            self.create_publisher(PoseStamped, 'detected_goal_2', 10),
            self.create_publisher(PoseStamped, 'detected_goal_3', 10),
        ]

        self.model = tf.keras.models.load_model('/home/lee/color_classifier6.h5')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',   # 실제 토픽명에 맞게 수정하세요!
            self.image_callback,
            10
        )

        self.detected_labels = []
        self.current_label = None
        self.timer_start_time = None
        self.timer_active = False
        self.last_display_state = 0
        self.state_change_time = None
        self.processing = False  # 중복 콜백 방지

        self.get_logger().info("✅ 노드 시작, 이미지 토픽을 기다립니다.")

    def predict(self, frame):
        resized = cv2.resize(frame, (224, 224))
        normalized = resized.astype(np.float32) / 255.0
        input_tensor = normalized.reshape((1, 224, 224, 3))
        pred = self.model.predict(input_tensor, verbose=0)
        class_idx = pred.argmax()
        confidence = pred[0][class_idx]
        label = CLASS_NAMES[class_idx]
        return label, confidence

    def is_color(self, hsv_roi, target_label):
        h, s, v = cv2.split(hsv_roi)
        total_pixels = h.size
        match_count = 0

        for lower, upper in COLOR_RANGES[target_label]:
            h_mask = (h >= lower[0]) & (h <= upper[0])
            s_mask = (s >= lower[1]) & (s <= upper[1])
            v_mask = (v >= lower[2]) & (v <= upper[2])
            match_count += np.count_nonzero(h_mask & s_mask & v_mask)

        ratio = match_count / total_pixels
        return ratio > 0.2

    def image_callback(self, msg):
        if self.processing:
            return
        self.processing = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
        self.processing = False

    def process_frame(self, frame):
        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        target = None
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            if area < 500 or area > (frame.shape[0] * frame.shape[1] * 0.7):
                continue

            roi = frame[y:y+h, x:x+w]
            label, confidence = self.predict(roi)

            # HSV 보정
            if label in ["bluebox", "blackbox"]:
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                if not self.is_color(hsv_roi, label):
                    opposite = "blackbox" if label == "bluebox" else "bluebox"
                    if self.is_color(hsv_roi, opposite):
                        label = opposite

            if confidence >= CONFIDENCE_THRESHOLD or label == "blackbox":
                target = (label, (x, y, w, h))
                break

        current_time = time.time()

        if target:
            label, (x, y, w, h) = target
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(display_frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            if self.current_label != label:
                self.current_label = label
                self.timer_start_time = current_time
                self.timer_active = True
                self.last_display_state = 1
                self.state_change_time = current_time
                self.get_logger().info(f"{label} 인식 시작")
            else:
                elapsed = current_time - self.timer_start_time
                if current_time - self.state_change_time >= 1.0 and self.last_display_state < 3:
                    self.last_display_state += 1
                    self.state_change_time = current_time
                    self.get_logger().info(f"{label} 상태 변화: {self.last_display_state}")

                color_map = {1: (0, 255, 255), 2: (0, 0, 255), 3: (0, 255, 0)}
                color = color_map.get(self.last_display_state, (255, 0, 0))
                cv2.rectangle(display_frame, (x, y), (x + w, y + h), color, 3)

                if elapsed >= TIMER_DURATION and label not in self.detected_labels and len(self.detected_labels) < 3:
                    self.detected_labels.append(label)
                    self.get_logger().info(f"🎯 확정 인식: {label}")
                    self.reset_timer()

                    if len(self.detected_labels) >= 3:
                        self.publish_goals()
                        self.shutdown()
                        return
        else:
            if self.timer_active:
                self.get_logger().info("⏹️ 인식 끊김 - 타이머 초기화")
            self.reset_timer()

        cv2.imshow("Classifier with HSV and Timer", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.shutdown()

    def reset_timer(self):
        self.timer_active = False
        self.current_label = None
        self.timer_start_time = None
        self.last_display_state = 0
        self.state_change_time = None

    def publish_goals(self):
        for i, label in enumerate(self.detected_labels):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()

            x, y, theta = OBJECT_TO_GOAL.get(label, (0.0, 0.0, 0.0))
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # 변환: roll=0, pitch=0, yaw=theta
            q = euler2quat(0, 0, theta)  # returns (w, x, y, z)
            pose.pose.orientation.x = q[1]
            pose.pose.orientation.y = q[2]
            pose.pose.orientation.z = q[3]
            pose.pose.orientation.w = q[0]

            self.goal_publishers[i].publish(pose)
            self.get_logger().info(f"📤 퍼블리시: detected_goal_{i+1} → x:{x}, y:{y}, θ:{theta:.2f} rad")

    def shutdown(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizerCombined()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()

if __name__ == '__main__':
    main()

