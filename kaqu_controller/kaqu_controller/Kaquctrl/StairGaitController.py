import rclpy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.Kaquctrl.GaitController import GaitController
from kaqu_controller.Kaquctrl.PIDController import PID_controller
# from kaqu_controller.Kaquctrl.PitchController import 
from kaqu_controller.KaquIK.KinematicsCalculations import rotxyz, rotz
from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

class StairGaitController(GaitController): # 강동륜 님
        self.use_imu = use_imu
        self.use_button = True
        self.autoRest = True
        self.trotNeeded = True

        leg = LegParameters()

        contact_phases = np.array([
            [1, 1, 1, 0],  # 0: Leg swing
            [1, 0, 1, 1],  # 1: Moving stance forward
            [1, 0, 1, 1],
            [1, 1, 1, 0]
        ])


    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기
        return state.foot_location
class StairSwingController(object): # 이태웅 님
class StairStanceController(object): # 천종욱 님
