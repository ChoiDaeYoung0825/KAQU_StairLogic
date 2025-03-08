import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32 #임의로

class PitchController(Node): # 소재정
  def __init__(self):
        super().__init__('Pitch_controller_node')
        self.subscription = self.create_subscription(
            Float32,  #임의로
            '/angle_in',
            self.angle_callback,
            10
        )
        self.subscription  # prevent unused variable warning
    
  def discretize_pose(self, angle: float) -> float: #소재정
        """
        연속 각도(angle)를 지정된 각도 값으로 변경
        """
        if angle <= -30:
            return np.radians(-30)
        elif angle <= -15:
            return np.radians(-15)
        elif angle >= 30:
            return np.radians(30)
        elif angle >= 15:
            return np.radians(15)
        else:
            return np.radians(0)

    def get_pitch_matrix(self, angle: float) -> np.ndarray: #김남윤
        # 1) angle을 디스크리트화하여 포즈 결정
        pose = self.discretize_pose(angle)

        # 2) 포즈에 해당하는 Pitch 라디안
        pitch_rad = self.pose_to_angle[pose]  # 완전히 다른 값 가능

        # 3) X축 기준 회전행렬(프로젝트 좌표계에 맞춰 수정 가능)
        cos_p = np.cos(pitch_rad)
        sin_p = np.sin(pitch_rad)
        pitch_mat = np.array([
            [1,    0,     0],
            [0, cos_p, -sin_p],
            [0, sin_p,  cos_p]
        ])
        return pitch_mat
          
