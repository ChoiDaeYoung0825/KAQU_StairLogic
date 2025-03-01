import rclpy
import numpy as np
from import Float32 #임의로

class Pitch_controller(node) #소재정
  def __init__(self):
        super().__init__('Pitch_controller_node')
        self.subscription = self.create_subscription(
            Float32,  #임의로
            '/angle_in',
            self.angle_callback,
            10
        )
        self.subscription  # prevent unused variable warning
    
    self.pose_to_angle = {
            -2: np.radians(-30),
            -1: np.radians(-15),
             0: np.radians(0),
             1: np.radians(15),
             2: np.radians(30)
        }
    
  def discretize_pose(self, angle: float) -> int: #소재정
        """
        연속 각도(angle)를 -2, -1, 0, 1, 2 중 하나로 디스크리트화
        """
        if angle < -30:
            return -2
        elif angle < -15:
            return -1
        elif angle > 15:
            return 1
          elif angle > 30:
            return 1
        else:
            return 0

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
          
