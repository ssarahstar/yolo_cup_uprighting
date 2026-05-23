
#!/usr/bin/env python3
"""
yolo_cup_recover_node.py
쓰러진 컵을 인식하고 보정된 오프셋으로 똑바로 세우는(Uprighting) 시나리오 노드.
"""

import time
import numpy as np

from . import _config as cfg
from ._base_node import BaseMoveItPickNode, run_node
from ._perception import calculate_cup_orientation
from ._motion import get_gripper_pose_by_cup

# =====================================================================
# 테스트 토글: 카메라와 욜로가 없어도 모션을 테스트하려면 True로 설정
USE_MOCK_VISION = True 
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
        if USE_MOCK_VISION:
            self.get_logger().info("가상 모드: 통신 우회를 위해 Mock Gripper를 활성화합니다.")
            self.gripper = MockGripper()
            # Action Server가 완전히 준비될 때까지 약간의 대기 시간(딜레이)을 줍니다.
            time.sleep(2.0)

    def _select_target(self, detections):
        """
        'toppled_cup' (쓰러진 컵) 클래스 중 신뢰도가 가장 높은 객체 선택
        (현재 YOLO 모델 컵/뚜껑만 구분 , 추후 클래스 ID 변경 예정)
        """
        if not detections:
            return None
            
        # 추후 'toppled_cup'으로 명명한 클래스만 필터링
        target_candidates = [d for d in detections if d["cls_name"] == "toppled_cup"]
        
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
            s
        bx, by, bz = base
        
        # 각도 추출
        if USE_MOCK_VISION:
            #cup_theta = np.pi / 4.0 
            #실제 로봇의 Yaw 각도(-26.91도)를 재현하기 위한 컵 각도 역산 적용
            cup_theta = np.radians(-116.91)
        else:
            cup_theta = calculate_cup_orientation(self.depth_image, target["box"], frame)

        self.picking = True
        try:
            self._pick_and_straighten(bx, by, bz, cup_theta)
        finally:
            self.picking = False


    def _pick_and_straighten(self, bx, by, bz, cup_theta):
        log = self.get_logger()
        
        # 컵의 쓰러진 각도에 맞춘 그리퍼 진입 쿼터니언 계산
        target_ori = get_gripper_pose_by_cup(cup_theta)
        
        # 정밀 파지 및 안착 높이(Z) 계산 (지름 데이터 적용!)
        # bz는 컵 표면 최상단의 높이이므로, 반지름(3.6cm)만큼 내려가야 컵의 중심축입니다.
        pick_z = bz - CUP_RADIUS_M
        
        # 바닥의 절대 높이 = 컵 중심축 높이 - 반지름
        floor_z = pick_z - CUP_RADIUS_M
        
        # 세웠을 때 로봇이 유지해야 할 높이 = 바닥 높이 + 컵 길이의 절반
        place_z = floor_z + (CUP_LENGTH_M / 2.0)
        
        log.info(f"== 컵 구출 시퀀스 시작 (각도: {np.degrees(cup_theta):.1f}도) ==")
        
        # 1. 컵 상공 진입 (충돌 방지를 위해 미리 그리퍼를 9cm 너비로 엽니다)
        log.info("[1] Approach (Opening gripper to 90mm)")
        self.gripper.move_gripper(900)  # RG2 단위: 1/10 mm -> 900 = 90mm
        time.sleep(0.5)
        self.plan_pose(bx, by, pick_z + 0.1, target_ori)
        
        # 2. 중심축까지 하강 및 파지
        log.info(f"[2] Descend to center axis (Z: {pick_z:.3f}) & Grip")
        self.plan_pose(bx, by, pick_z, target_ori)
        self.gripper.close_gripper()
        time.sleep(1.0)
        

        # 3. 수직 리프트업 (마찰 회피)
        log.info("[3] Lift Up (관절 꼬임 방지를 위해 높게 들어 올림)")
        self.plan_pose(bx, by, pick_z + 0.25, target_ori)
        
        

        # 4. 수직 자세(home_ori)로 회전 + 회전 반경 오프셋 보정
        dx = (CUP_LENGTH_M / 2.0) * np.cos(cup_theta)
        dy = (CUP_LENGTH_M / 2.0) * np.sin(cup_theta)

        place_x = bx - dx
        place_y = by - dy
        place_z = floor_z + (CUP_LENGTH_M / 2.0)

        
        # 4-1. 바닥에 닿기 전, 공중(place_z + 0.15)에서 컵을 먼저 수직으로 돌립니다.
        log.info("[4-1] 공중에서 컵 수직 정렬")
        self.plan_pose(place_x, place_y, place_z + 0.15, self.home_ori)

        # 4-2. 수직을 유지한 채 바닥으로 수직 하강하여 안착합니다.
        log.info(f"[4-2] Z-Height Adjustment (Z: {place_z:.3f})")
        self.plan_pose(place_x, place_y, place_z + 0.02, self.home_ori)
        
        # 5. 조심스럽게 완전히 바닥에 닿은 후 릴리즈
        log.info("[5] Place & Release")
        self.plan_pose(place_x, place_y, place_z, self.home_ori)
        self.gripper.open_gripper()
        time.sleep(1.0)

        
        # 6. 안전 상공 복귀
        log.info("[6] Retract")
        self.plan_pose(place_x, place_y, place_z + 0.1, self.home_ori)
        log.info("== 시퀀스 완료 ==")
    
    # ==========================================================
    # [임시 테스트용] p키 입력 대기를 무시하고 자동 실행하는 함수

    # ==========================================================
    def run(self):
        import time
        import rclpy
        import numpy as np

        self.get_logger().info("==== [자동 테스트 모드] ====")
        
        # 1. 로봇 초기화 및 Home 위치 이동 (부모 클래스의 필수 기능 실행)
        if hasattr(self, 'initialize_home'):
            self.initialize_home()
            
        self.get_logger().info("3초 뒤 컵 구출 시퀀스를 자동으로 시작합니다...")
        time.sleep(3.0)
        
        # 2. 카메라가 없으므로 가상의 빈 이미지(dummy)를 만들어서 강제 전달
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.detect_and_pick(dummy_frame)
        
        self.get_logger().info("==== [테스트 시퀀스 완료] ====")
        self.get_logger().info("RViz 화면을 확인할 수 있도록 노드를 끄지 않고 유지합니다.")
        
        # 3. 모션이 끝나고 프로그램이 바로 꺼지지 않도록 대기 상태 유지
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    run_node(YoloCupUprightingNode)


if __name__ == "__main__":
    main()
