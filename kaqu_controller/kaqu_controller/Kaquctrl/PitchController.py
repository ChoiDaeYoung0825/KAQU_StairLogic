import rclpy
from rclpy.node import Node
import numpy as np
import tf2.transformations as tft
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

def get_pitch_matrix(self, angle: float) -> np.ndarray:
    radian = self.discretize_pose(angle)
  
    #X축 기준 회전행렬(프로젝트 좌표계에 맞춰 수정 가능)
    pitch_mat_hom = tft.rotation_matrix(radian, (1, 0, 0))
  
    #생성된 4x4 행렬에서 상위 3x3 부분(회전 행렬만 해당)을 추출합니다.
    pitch_mat = pitch_mat_hom[0:3, 0:3]
  
    #radian을 받을지 pitch_mat를 받을지 선택 가능
    return pitch_mat
