
#!/usr/bin/env python3
"""
쓰러진 컵을 인식하고 보정된 오프셋으로 똑바로 세우는(Uprighting) 시나리오 노드.
"""

import time
import numpy as np

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node
from ._perception import calculate_cup_orientation
from ._motion import get_gripper_pose_by_cup
from scipy.spatial.transform import Rotation as R


# =====================================================================
# 테스트 토글: 카메라와 욜로가 없어도 모션을 테스트하려면 True로 설정
USE_MOCK_VISION = False
# =====================================================================

CUP_LENGTH_M = 0.12  
CUP_DIAMETER_M = 0.072 
CUP_RADIUS_M   = CUP_DIAMETER_M / 2.0 


class MockGripper:
    """가상 환경 테스트를 위해 실제 Modbus 통신을 우회하는 가짜 그리퍼 클래스"""
    def open_gripper(self):
        print("[Mock Gripper] 가상 그리퍼 열림 (110mm)")
        
    def close_gripper(self):
        print("[Mock Gripper] 가상 그리퍼 닫힘")
        
    def move_gripper(self, width, force=None):
        print(f"[Mock Gripper] 가상 그리퍼 너비 이동 -> {width/10.0}mm")


class YoloCupUprightingNode(BaseMoveItPickNode):
    NODE_NAME        = "yolo_cup_uprighting_node"
    MOVEIT_NODE_NAME = "yolo_cup_uprighting_py"
    WINDOW_NAME      = "Cup Uprighting"

    def __init__(self):
        super().__init__()
        #if USE_MOCK_VISION:
            #self.get_logger().info("가상 모드: 통신 우회를 위해 Mock Gripper를 활성화합니다.")
            #self.gripper = MockGripper()
            # Action Server가 완전히 준비될 때까지 약간의 대기 시간(딜레이)을 줍니다.
            #time.sleep(2.0)

    
    def _select_target(self, detections):
        """
        현재 YOLO 모델의 실제 클래스 이름('cup')을 찾아 신뢰도가 가장 높은 객체를 선택
        """
        if not detections:
            return None
            
        target_candidates = [d for d in detections if d["cls_name"] == "cup"] 
        
        if not target_candidates:
            return None
            
        return max(target_candidates, key=lambda d: d["conf"])
    
    
    def run_yolo(self, frame):
        """MOCK 모드일 경우 가상의 쓰러진 컵 데이터를 반환, 아니면 부모(진짜 YOLO) 호출"""
        if USE_MOCK_VISION:
            return [{
                "cx": 320, "cy": 240, "conf": 0.95,
                "cls_id": 99, "cls_name": "toppled_cup",
                "box": (200, 150, 440, 330), # 가로로 누워있는 가상의 바운딩 박스
                "size": 240, "depth": 0.5
            }]
        else:
            return super().run_yolo(frame)

    def pixel_to_base(self, px, py):
        """MOCK 모드일 경우 가상의 3D 공간 좌표 반환, 아니면 진짜 카메라 Depth 매핑 호출"""
        if USE_MOCK_VISION:
        
            # 실제 추출된 X: 0.487m, Y: -0.022m
            # 역산된 컵 표면 Z: 0.051m
            return (0.487, -0.022, 0.051)
        else:
            return super().pixel_to_base(px, py)
        

    def detect_and_pick(self, frame: np.ndarray):
        log = self.get_logger()
        if self.picking:
            log.warn("이미 시퀀스 실행 중입니다.")
            return

        detections = self.run_yolo(frame)
        self._detections = detections
        target = self._select_target(detections)
        
        if target is None:
            log.warn("쓰러진 컵을 찾을 수 없습니다.")
            return

        base = self.pixel_to_base(target["cx"], target["cy"])
        if base is None:
            log.error("픽셀 -> 베이스 3D 좌표 변환 실패.")
            return
        bx, by, bz = base
        
        # 각도 추출
        if USE_MOCK_VISION:
        
            cup_theta = np.radians(-116.91)
        else:
            cup_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

        self.picking = True
        try:
            self._pick_and_straighten(bx, by, bz, cup_theta)
        finally:
            self.picking = False

    
        
    # def move_to_observation_pose(self):

    #     log = self.get_logger()
    #     log.info("[Init] 테이블 관찰 자세로 이동 중...")
        
    #     # 1. 제시된 조인트 각도(도 단위) 및 라디안 변환
    #     joint_deg = [3.0, -12.7, 44.0, -9.0, 133.0, 90.0]
    #     joint_values = [np.deg2rad(angle) for angle in joint_deg]

    #     # 2. manipulator 플래닝 컴포넌트 준비
    #     arm_component = self.robot.get_planning_component("manipulator")
    #     arm_component.set_start_state_to_current_state()
        
    #     goal_state = arm_component.get_start_state()
        
    #     # 가져온 상태 객체의 'manipulator' 관절 그룹에 목표 각도를 덮어씌웁니다.
    #     goal_state.set_joint_group_positions("manipulator", joint_values)
    #     goal_state.update() # 상태 갱신
        
    #     # 완성된 상태 객체를 플래너의 목표(Goal)로 설정합니다.
    #     arm_component.set_goal_state(robot_state=goal_state)
    #     # ==========================================================
        
    #     # 3. 경로 플래닝 및 실행
    #     plan_result = arm_component.plan()

    #     if plan_result:
    #         log.info("관찰 자세 경로 생성 성공. 이동을 시작합니다.")
    #         self.robot.execute("manipulator", plan_result.trajectory)
    #         time.sleep(1.5) # 로봇이 완전히 멈출 때까지 대기
    #         return True
    #     else:
    #         log.error("관찰 자세 플래닝 실패!")
    #         return False
       


    def _pick_and_straighten(self, bx, by, bz, cup_theta):
        log = self.get_logger()
        
        target_ori = get_gripper_pose_by_cup(cup_theta)

     
        TABLE_Z = 0.0 
        floor_z = TABLE_Z


        Z_OFFSET = cfg.Z_OFFSET  # 0.20m (20cm)

        PICK_CLEARANCE = 0.02 
        
        pick_z = floor_z + CUP_RADIUS_M + Z_OFFSET + PICK_CLEARANCE
        place_z = floor_z + (CUP_LENGTH_M / 2.0) + Z_OFFSET
        
        safe_z = floor_z + 0.25 + Z_OFFSET
        
        log.info(f"== 컵 구출 시퀀스 준비 (각도: {np.degrees(cup_theta):.1f}도) ==")
          


        log.info("[1-1] 상공 진입 (Z=25cm)")
        
        arm_component = self.robot.get_planning_component("manipulator")
        arm_component.set_start_state_to_current_state()
        current_state = arm_component.get_start_state()
        
        # 'link_6' 끝단의 현재 공간 좌표와 방향(Quaternion) 추출
        current_pose = current_state.get_pose("link_6") 
        
        current_ori = {
            "x": current_pose.orientation.x,
            "y": current_pose.orientation.y,
            "z": current_pose.orientation.z,
            "w": current_pose.orientation.w
        }
        
        # 추출한 현재 방향(current_ori)을 유지하면서 Z축만 상공으로 이동
        self.plan_pose(bx, by, safe_z, current_ori)
        time.sleep(1.0)


        log.info("[1-2] 상공에서 컵 방향으로 정렬")
        self.plan_pose(bx, by, safe_z, target_ori)
        time.sleep(1.0)

        log.info("[2] 컵 집기 시작")
        self.plan_pose(bx, by, pick_z, target_ori)
        self.gripper.close_gripper()
        log.info("[2] 컵 집기 완료")
        time.sleep(1.0)

        log.info("[3] Lift Up (다시 바닥 기준 25cm 상공으로 리프트업)")
        self.plan_pose(bx, by, safe_z, target_ori)
        time.sleep(1.0)

        
        log.info("[4] 동적 직립화 궤적 탐색 시작...")

        quat_A = R.from_euler('xyz', [90, 0, np.degrees(cup_theta)], degrees=True).as_quat()
        ori_A = {"x": float(quat_A[0]), "y": float(quat_A[1]), "z": float(quat_A[2]), "w": float(quat_A[3])}

        quat_B = R.from_euler('xyz', [270, 0, np.degrees(cup_theta)], degrees=True).as_quat()
        ori_B = {"x": float(quat_B[0]), "y": float(quat_B[1]), "z": float(quat_B[2]), "w": float(quat_B[3])}

        dx = (CUP_LENGTH_M / 2.0) * np.cos(cup_theta)
        dy = (CUP_LENGTH_M / 2.0) * np.sin(cup_theta)
        place_x = bx - dx
        place_y = by - dy
        place_z = 0.07

        log.info("-> 옵션 A(Roll=90) 경로 플래닝 시도 중...")
        success = self.plan_pose(place_x, place_y, place_z + 0.15, ori_A)

        if success:
            log.info("=> 옵션 A 채택 성공! (관절 한계 안전)")
            best_ori = ori_A
        else:
            log.warn("=> 옵션 A IK 실패. 옵션 B(Roll=270)로 우회 탐색합니다...")
            success = self.plan_pose(place_x, place_y, place_z + 0.15, ori_B)
            
            if success:
                log.info("=> 옵션 B 채택 성공! (안전한 반대 방향으로 컵을 세웁니다)")
                best_ori = ori_B
            else:
                log.error("=> 치명적 오류: 양쪽 방향 모두 직립화 궤적 생성에 실패했습니다.")
                return 

        log.info("[4-1] 공중에서 컵 수직 정렬 완료")
        
        log.info(f"[4-2] Z-Height Adjustment (Z: {place_z:.3f})")
        self.plan_pose(place_x, place_y, place_z + 0.02, best_ori)
        
        log.info("[5] Place & Release")
        self.plan_pose(place_x, place_y, place_z, best_ori)
        self.gripper.open_gripper()
        time.sleep(1.0)
        
        log.info("[6] Retract")
        self.plan_pose(place_x, place_y, place_z + 0.15, best_ori)
        log.info("== 시퀀스 완료 ==")

def main(args=None):
    run_node(YoloCupUprightingNode)


if __name__ == "__main__":
    main()
