import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

class NavigateToGoal(Node):
    def __init__(self):
        super().__init__('navigate_to_goal')
        
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal() 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map" 
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        quaternion = self.yaw_to_quaternion(yaw)
        
        goal_msg.pose.pose.orientation.x = quaternion[0]
        goal_msg.pose.pose.orientation.y = quaternion[1]
        goal_msg.pose.pose.orientation.z = quaternion[2]
        goal_msg.pose.pose.orientation.w = quaternion[3]
        
        self._client.wait_for_server()
        self.get_logger().info(f"Sending goal to ({x}, {y}) with yaw {yaw}")

        future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def yaw_to_quaternion(self, yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        return [qx, qy, qz, qw]

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Current position: {feedback.feedback.current_pose.pose.position.x}, {feedback.feedback.current_pose.pose.position.y}")
        
    def goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info("Goal accepted!")
        else:
            self.get_logger().info("Goal rejected!")

        if result.status == 3:  
            self.get_logger().info("Goal reached!")
        elif result.status == 4: 
            self.get_logger().info("Goal failed!")

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()

    while rclpy.ok():
        coordinates = input("목표 좌표를 x y yaw 형식으로 입력하세요 (예: 1.0 1.0 1.0), 종료하려면 'exit' 입력: ")
        
        if coordinates.lower() == 'exit':
            break
        
        try:
            x, y, yaw = map(float, coordinates.split())
        except ValueError:
            print("유효하지 않은 좌표 형식입니다. x, y, yaw 좌표를 공백으로 구분하여 입력하세요.")
            continue
        
        node.send_goal(x, y, yaw)
        
        rclpy.spin_once(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

