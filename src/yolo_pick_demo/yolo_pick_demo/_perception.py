"""YOLO 추론 + 카메라 좌표 변환.

Hand-Eye 행렬, pixel→base 변환, YOLO 검출을 함수로 제공.
"""

from pathlib import Path

import cv2  
import numpy as np

from ament_index_python.packages import get_package_share_directory

from . import _config as cfg
from ._motion import get_ee_matrix


def bbox_size(box) -> int:
    """bbox max(w, h) — 길이/지름 대표값."""
    x1, y1, x2, y2 = box
    return max(x2 - x1, y2 - y1)


def load_hand_eye():
    """T_gripper2camera.npy 로드 (mm → m)."""
    calib_file = (
        Path(get_package_share_directory("yolo_pick_demo"))
        / "config" / "T_gripper2camera.npy"
    )
    g2c = np.load(str(calib_file)).astype(float)
    g2c[:3, 3] /= 1000.0   # mm → m
    return g2c, calib_file


def transform_to_base(robot, gripper2cam, cam_xyz_m):
    """카메라 좌표 (m) → base 좌표 (m). 현재 EE 자세 기준."""
    coord = np.append(np.array(cam_xyz_m, dtype=float), 1.0)
    base2ee  = get_ee_matrix(robot)
    base2cam = base2ee @ gripper2cam
    return (base2cam @ coord)[:3]


def pixel_to_base(robot, gripper2cam, depth_image, intrinsics,
                  px: int, py: int, logger):
    """픽셀 + depth 이미지 → base 좌표 (m). 실패 시 None."""
    if depth_image is None or intrinsics is None:
        logger.warn("frame/intrinsics 아직 준비 안됨")
        return None

    h, w = depth_image.shape[:2]
    if not (0 <= px < w and 0 <= py < h):
        logger.warn("pixel 범위 초과")
        return None

    z_raw = depth_image[py, px]
    if z_raw == 0:
        logger.warn(f"depth=0 at ({px}, {py})")
        return None

    z_m = (float(z_raw) / 1000.0
           if depth_image.dtype == np.uint16 else float(z_raw))

    fx, fy   = intrinsics["fx"],  intrinsics["fy"]
    ppx, ppy = intrinsics["ppx"], intrinsics["ppy"]

    cam_x = (px - ppx) * z_m / fx
    cam_y = (py - ppy) * z_m / fy
    cam_z = z_m

    base = transform_to_base(robot, gripper2cam, (cam_x, cam_y, cam_z))
    logger.info(
        f"pixel({px},{py}) cam({cam_x:.3f},{cam_y:.3f},{cam_z:.3f}) "
        f"-> base({base[0]:.3f},{base[1]:.3f},{base[2]:.3f}) m"
    )
    return tuple(float(v) for v in base)


def _depth_at(depth_image, cx: int, cy: int) -> float:
    """픽셀의 depth (m). 없으면 inf."""
    if depth_image is None:
        return float("inf")
    h, w = depth_image.shape[:2]
    if not (0 <= cx < w and 0 <= cy < h):
        return float("inf")
    z_raw = depth_image[cy, cx]
    if z_raw == 0:
        return float("inf")
    return (float(z_raw) / 1000.0
            if depth_image.dtype == np.uint16 else float(z_raw))


def run_yolo(yolo, frame: np.ndarray, depth_image=None) -> list[dict]:
    """YOLO 추론. 각 detection 에 cx/cy/conf/cls/bbox/size/depth 포함."""
    results = yolo(frame, verbose=False)[0]
    detections = []

    for box in results.boxes:
        conf   = float(box.conf[0])
        cls_id = int(box.cls[0])
        if conf < cfg.YOLO_CONF_THRESH:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        cls_name = yolo.names.get(cls_id, str(cls_id))

        detections.append({
            "cx": cx, "cy": cy,
            "conf": conf,
            "cls_id": cls_id,
            "cls_name": cls_name,
            "box": (x1, y1, x2, y2),
            "size": bbox_size((x1, y1, x2, y2)),
            "depth": _depth_at(depth_image, cx, cy),
        })

    return detections



def calculate_cup_orientation(depth_image, bbox, frame=None):
    """
    YOLO Bounding Box 또는 영상 데이터를 받아 컵이 누워있는 각도(theta, 라디안)를 계산
    추후 세그멘테이션 마스크 / 정밀 PCA 알고리즘 연동 예정
    """
    # 1. Bounding Box 정보 추출
    x1, y1, x2, y2 = bbox
    
    # 임시 알고리즘 (현재 단계): 박스의 가로/세로 비율을 통해 단순 각도 추정
    # 가로가 길면 x축과 평행(0도), 세로가 길면 y축과 평행(90도)하다고 가정
    width = x2 - x1
    height = y2 - y1
    
    if width > height:
        theta = 0.0  # 누워있는 상태 (가로)s
    else:
        theta = np.pi / 2.0  # 누워있는 상태 (세로)
        
    # TODO (추후 확장): frame ROI를 잘라내어 cv2.PCACompute 또는 cv2.fitLine 적용
    # roi = frame[y1:y2, x1:x2]
    # ... 이미지 처리 및 외곽선 추출 로직 ...
    
    return theta
