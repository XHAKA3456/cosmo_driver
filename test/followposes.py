import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from action_msgs.msg import GoalStatus


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
        
        # 추가적인 상태 정보 출력
        if feedback.feedback.status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().info("현재 목표 수행 중...")
        elif feedback.feedback.status == GoalStatus.STATUS_CANCELING:
            self.get_logger().info("목표 취소 중...")

    def goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info("Goal accepted!")
        else:
            self.get_logger().info("Goal rejected!")
            return

        # 결과 상태 확인
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("목표에 도달했습니다!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("목표가 취소되었습니다.")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("목표 수행 실패!")


def get_waypoints_from_input():
    waypoints = []
    print("목표 좌표를 입력하세요 (x, y, yaw 형식): ")
    while True:
        user_input = input("좌표 입력 (예: 2.0 3.0 1.57), 입력을 완료하면 xx를 누르세요: ")
        if user_input.strip() == 'xx':
            break
        try:
            x, y, yaw = map(float, user_input.split())
            waypoints.append((x, y, yaw))
        except ValueError:
            print("잘못된 입력입니다. 올바른 형식으로 입력하세요.")
    
    return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToGoal()

    while True:
        waypoints = get_waypoints_from_input()

        for x, y, yaw in waypoints:
            node.send_goal(x, y, yaw)
            rclpy.spin_once(node)
        
        continue_input = input("다른 목표 좌표를 입력하시겠습니까? (y/n): ")
        if continue_input.lower() != 'y':
            break

    rclpy.shutdown()


if __name__ == '__main__':
    main()
