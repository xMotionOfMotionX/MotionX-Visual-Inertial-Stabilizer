import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class VirtualMotionXCore(Node):
    def __init__(self):
        super().__init__('virtual_motionx_core')
        # 어댑터가 쏘는 표준 데이터를 구독합니다.
        self.create_subscription(Imu, '/motionx/raw_inertia', self.process_callback, 10)
        self.get_logger().info('🚀 Virtual MotionX Core (Jetson Dummy) Started!')

    def process_callback(self, msg):
        # 1. 멀미 저감 보정값 계산 (시뮬레이션)
        dx_comp = -msg.linear_acceleration.y * 0.5  # 가속도의 반대 방향 보정
        dy_comp = -msg.linear_acceleration.z * 0.5
        
        # 2. 위험 감지 (Emergency Stop logic)
        # Jerk(가속도 변화량)가 임계값 $J_{limit}$를 넘는지 체크
        jerk_magnitude = abs(msg.linear_acceleration.x) 
        
        if jerk_magnitude > 10.0:
            self.get_logger().error('🚨 EMERGENCY: Dangerous Motion Detected! Sending STOP Signal.')
        else:
            self.get_logger().info(f'✅ Stabilizing... Correction Vector: [{dx_comp:.4f}, {dy_comp:.4f}]')

def main():
    rclpy.init()
    node = VirtualMotionXCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()