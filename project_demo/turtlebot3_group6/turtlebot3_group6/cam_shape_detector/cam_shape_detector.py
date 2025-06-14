import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class CamShapeDetector(Node):
    def __init__(self):
        super().__init__('cam_shape_detector')
        self.publisher = self.create_publisher(PointStamped, '/detected_goal', 10)
        self.image_pub = self.create_publisher(Image, 'processed_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.detected = False  # 한 번만 인식하기 위한 플래그

        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패")
            
    def get_color_name_from_hsv(self, bgr_color):
        bgr_np = np.uint8([[bgr_color]])
        hsv = cv2.cvtColor(bgr_np, cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        if v < 50:
            return "Black"
        elif s < 50 and v > 200:
            return "White"
        elif s < 50:
            return "Gray"

        if h < 10 or h >= 160:
            return "Red"
        elif h < 25:
            return "Orange"
        elif h < 35:
            return "Yellow"
        elif h < 85:
            return "Green"
        elif h < 125:
            return "Blue"
        elif h < 160:
            return "Purple"
        else:
            return "Unknown"

    def timer_callback(self):
        if self.detected:
            return  # 한 번 감지 후 멈춤

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임을 읽을 수 없음")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(cnt)

            if len(approx) == 3:
                shape = "Triangle"
            elif len(approx) == 4:
                shape = "Rectangle"
            elif len(approx) > 4:
                shape = "Circle"
            else:
                shape = "Unknown"

            roi = frame[y:y+h, x:x+w]
            if roi.size > 0:
                avg_color = cv2.mean(roi)[:3]
                bgr_color = (avg_color[0], avg_color[1], avg_color[2])
                color_name = self.get_color_name_from_hsv(bgr_color)
            else:
                color_name = "Unknown"

            label = f"{shape} {color_name}"
            cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 인식된 모양+색에 따라 목표점 결정 (예시)
            goal_point = None
            if shape == "Triangle" and color_name == "Red":
                goal_point = (1.0, 0.5)  # 예: 빨간 삼각형이면 (1.0, 0.5)
            elif shape == "Rectangle" and color_name == "Blue":
                goal_point = (0.5, -0.5)
            elif shape == "Circle" and color_name == "Green":
                goal_point = (0.0, 1.0)

            if goal_point is not None:
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'map'
                point_msg.point.x = goal_point[0]
                point_msg.point.y = goal_point[1]
                point_msg.point.z = 0.0

                self.publisher.publish(point_msg)
                self.get_logger().info(f"목표 발행: {shape} {color_name} -> {goal_point}")
                self.detected = True
                break

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CamShapeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
