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
    def __init__(self, default_stance, stance_time, swing_time, time_step, use_imu):
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

        z_error_constant = 0.02 * 4  # This constant determines how fast we move toward the goal in the z direction

        self.stance_time = leg.gait.stair_stance_time   #stair_stance_time 적용 (값은 KaquParams.py에서 변경해야 함) - 강동륜
        self.swing_time = leg.gait.stair_swing_time     #stair_swing_time 적용
        z_leg_lift = leg.gait.stair_z_leg_lift          #stair_z_leg_lift 적용
        # 부모 클래스 (GaitController) 호출
        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)

        self.max_x_vel = leg.gait.max_x_vel
        self.max_y_vel = leg.gait.max_y_vel
        self.max_yaw_rate = leg.gait.max_yaw_rate
        # swing이랑 stance에 정보주는거?
        self.swingController = StairSwingController(self.stance_ticks, self.swing_ticks, self.time_step,
                                                    self.phase_length, z_leg_lift, self.default_stance)

        self.stanceController = StairStanceController(self.phase_length, self.stance_ticks, self.swing_ticks,
                                                      self.time_step, z_error_constant)

        # TODO : 게인값 조율
        self.pid_controller = PID_controller(0., 0., 0.)  # 게인이 0이라 효력 X

    # 선속도와 각속도를 스케일링
    def updateStateCommand(self, msg, command):
        command.velocity[0] = (0.5 + (msg.axes[2] * 0.5)) / 1.5 * self.max_x_vel
        command.velocity[1] = (0.5+ (msg.axes[2] * 0.5)) / 1.5 * self.max_y_vel
        command.yaw_rate = msg.axes[6] * self.max_yaw_rate

        print(f"Velocity X: {command.velocity[0]}, Y: {command.velocity[1]}, Yaw Rate: {command.yaw_rate}")

    def step(self, state, command):
        if self.autoRest:  # 멈춰있으면 자동으로 휴식 자세
            if command.velocity[0] == 0 and command.velocity[1] == 0 and command.yaw_rate == 0:
                if state.ticks % (2 * self.phase_length) == 0:
                    self.trotNeeded = False
            else:
                self.trotNeeded = True

        print(f"Trot Needed: {self.trotNeeded}, Velocity: {command.velocity}, Yaw Rate: {command.yaw_rate}")

        if self.trotNeeded:  # 움직이고 있으면
            contact_modes = self.contacts(state.ticks)  # 접지 배열 가져오기
            new_foot_locations = np.zeros((3, 4))

            for leg_index in range(4):
                contact_mode = contact_modes[leg_index]
                if contact_mode == 1:  # 접지해 있다면 스탠스
                    new_location = self.stanceController.next_foot_location(leg_index, state, command)
                else:  # 아니라면 스윙 반환
                    swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)
                    new_location = self.swingController.next_foot_location(swing_proportion, leg_index, state, command)
                new_foot_locations[:, leg_index] = new_location
       

            # imu compensation IMU 보정
            if self.use_imu:
                compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
                roll_compensation = -compensation[0]
                pitch_compensation = -compensation[1]

                rot = rotxyz(roll_compensation, pitch_compensation, 0)
                new_foot_locations = np.matmul(rot, new_foot_locations)

            state.ticks += 1
            return new_foot_locations

        else:  # 안 움직이고 있으면
            temp = self.default_stance.copy()
            temp[2] = [command.robot_height] * 4
            return temp


    def run(self, state, command):  # 외부에서 이 컨트롤러를 사용할 때 호출하는 최종 메서드: 한 틱씩 step()을 돌려 발 위치 업데이트
        state.foot_location = self.step(state, command)
        state.robot_height = command.robot_height  # 현재 높이를 state에 넣기
        return state.foot_location

class StairSwingController(object): # 이태웅 님
class StairStanceController(object): # 천종욱 님
