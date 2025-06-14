import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
import threading

def yaw_to_quaternion(yaw):
    """Z축 회전(yaw, 라디안)을 쿼터니언으로 변환"""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class path_node(Node):
    def __init__(self):
        super().__init__('goal_subscriber')
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.waypoints = []
        self.current_position = None
        self.path = []
        self.index = 0

        # 입력 스레드 시작
        self.input_thread = threading.Thread(target=self.read_goals_from_terminal)
        self.input_thread.daemon = True
        self.input_thread.start()

    def read_goals_from_terminal(self):
        """터미널에서 x, y, heading(라디안) 입력받기"""
        while rclpy.ok():
            try:
                x = float(input("Enter x coordinate: "))
                y = float(input("Enter y coordinate: "))
                heading = float(input("Enter goal heading (radian): "))
                
                self.waypoints.append((x, y, heading))
                self.get_logger().info(f"Added waypoint: ({x}, {y}, heading={heading})")
                    
                if len(self.waypoints) >= 3:
                    self.compute_and_navigate()
                    self.waypoints.clear()
            except ValueError:
                self.get_logger().error("Invalid input! Please enter numbers only.")
            except Exception as e:
                self.get_logger().error(f"Error: {str(e)}")
                break

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def compute_and_navigate(self):
        if self.current_position is None:
            self.get_logger().warn("Waiting for initial position...")
            return

        self.start_point = self.current_position
        self.path = self.waypoints.copy()
        self.index = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.index >= len(self.path):
            self.get_logger().info("All goals reached!")
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
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = path_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

