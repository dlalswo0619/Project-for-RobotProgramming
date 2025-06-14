import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
import math
import itertools

def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class PathNode(Node):
    def __init__(self):
        super().__init__('path_node')
        self.start_position = None

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.sub1 = self.create_subscription(
            PoseStamped, '/detected_goal_1', self.goal_callback, 10)
        self.sub2 = self.create_subscription(
            PoseStamped, '/detected_goal_2', self.goal_callback, 10)
        self.sub3 = self.create_subscription(
            PoseStamped, '/detected_goal_3', self.goal_callback, 10)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.waypoints = []
        self.current_position = None
        self.path = []
        self.index = 0

        self.paused = False
        self._goal_handle = None

        self.pause_service = self.create_service(
            SetBool,
            'pause_navigation',
            self.pause_callback
        )

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        if self.start_position is None:
            self.start_position = self.current_position
            self.get_logger().info(f"🏁 시작 위치 저장: {self.start_position}")

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        heading = yaw_from_quaternion(q)
        goal = (x, y, heading)
        if goal not in self.waypoints:
            self.waypoints.append(goal)
            self.get_logger().info(f"새 목표점 수신: {goal}")
        if len(self.waypoints) == 3:
            self.get_logger().info("3개 목표점 모두 수신, 경로 계산 시작")
            self.compute_and_navigate()

    def compute_and_navigate(self):
        if self.current_position is None:
            self.get_logger().error("odom 데이터 수신 대기 중...")
            return

        start = self.current_position
        best_order = None
        best_cost = float('inf')

        for perm in itertools.permutations(self.waypoints):
            cost = 0
            curr = start
            for x, y, _ in perm:
                dx = curr[0] - x
                dy = curr[1] - y
                cost += math.hypot(dx, dy)
                curr = (x, y)
            dx = curr[0] - self.start_position[0]
            dy = curr[1] - self.start_position[1]
            cost += math.hypot(dx, dy)
            if cost < best_cost:
                best_cost = cost
                best_order = perm

        self.path = list(best_order)
        self.path.append((self.start_position[0], self.start_position[1], 0.0))
        self.index = 0
        self.get_logger().info(f"📍 최적 경로 계산 완료 (거리: {best_cost:.2f})")
        self.send_next_goal()

    def send_next_goal(self):
        if self.paused:
            self.get_logger().info("⏸️ 주행이 일시정지 상태입니다. 재개될 때까지 대기합니다.")
            return

        if self.index >= len(self.path):
            self.get_logger().info("모든 목표점 도달 완료!")
            self.waypoints.clear()
            return

        x, y, heading = self.path[self.index]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        quat = yaw_to_quaternion(heading)
        goal_pose.pose.orientation = quat

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.index += 1

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._goal_handle = goal_handle
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.send_next_goal()

    def pause_callback(self, request, response):
        if request.data:
            if not self.paused:
                self.paused = True
                self.get_logger().info("⏸️ 주행이 일시정지되었습니다.")
                if self._goal_handle is not None:
                    self.client.cancel_all_goals_async()
            response.success = True
            response.message = "주행 일시정지"
        else:
            if self.paused:
                self.paused = False
                self.get_logger().info("▶️ 주행이 재개되었습니다.")
                self.send_next_goal()
            response.success = True
            response.message = "주행 재개"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
