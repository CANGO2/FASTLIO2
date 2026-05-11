import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from interface.srv import Relocalize
import math

class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initialpose_bridge')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.callback,
            10
        )
        self.cli = self.create_client(Relocalize, '/localizer/relocalize')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('localizer service 대기중...')

        self.map_path = '/home/nuc/GlobalMap2.pcd'
        self.get_logger().info('InitialPose Bridge started')

        # 시작하자마자 초기 위치로 relocalize 직접 호출
        self.send_initial_pose()

    def send_initial_pose(self):
        req = Relocalize.Request()
        req.pcd_path = self.map_path
        req.x     = -19.35
        req.y     =  7.312
        req.z     =  0.178
        req.yaw   = -1.52
        req.pitch =  0.0
        req.roll  =  0.0
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)
        self.get_logger().info('초기 위치 relocalize 호출')

    def callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f'Initial pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}deg')
        req = Relocalize.Request()
        req.pcd_path = self.map_path
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.pitch = 0.0
        req.roll = 0.0
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        result = future.result()
        if result.success:
            self.get_logger().info(f'Relocalize 성공: {result.message}')
        else:
            self.get_logger().warn(f'Relocalize 실패: {result.message}')

def main():
    rclpy.init()
    node = InitialPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
