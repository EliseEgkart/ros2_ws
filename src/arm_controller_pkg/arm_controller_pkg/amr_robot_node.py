import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from arm_interfaces.srv import Cargo, GetTargetPose
from sml_msgs.srv import ArmCommand
from std_srvs.srv import Trigger
import rbpodo as rb
import numpy as np
import time
import threading


ROBOT_IP = "10.0.2.8"

HOME_JOINT_DEG        = np.array([-90.0,  0.0,   90.0,  0.0, 90.0,  0.0])
MOVING_JOINT_DEG      = np.array([-90.0, -26.02, 140.8, 0.0, 65.22, 0.0])
VISION_LOAD_JOINT_DEG = np.array([-90.0,  13.28,  75.45, 0.0, 91.27, 0.0])
# np.array([-90.0, 13.82, 83.37, 0.0, 82.82, 0.0]), z=280.28
# np.array([-90.0, 13.43, 80.49, 0.0, 86.09, 0.0]), z=301.28
# np.array([-90.0, 13.28,  75.45, 0.0, 91.27, 0.0]), z=331.28 main
# np.array([-90.0, 13.70, 69.94, 0.0, 96.36, 0.0]), z=361.28
# np.array([-90.0, 14.26, 66.11, 0.0, 99.62, 0.0]),  z=381.28

# 슬롯 2~8 공통 경유점 (슬롯 1은 경로가 달라 별도 관리)
SLOT_COMMON_WPS = [
    np.array([-90.0,   -20.81,  107.71, 0.0,  93.11,  0.0]),
    np.array([-160.21, -8.27,  125.95, 0.46,  60.24,  0.0]),
    np.array([-220.0,  -11.96,   57.40, 0.0, 100.40,  0.0]),
]

# LOAD 시 슬롯별 최종 접근 위치 (슬롯 2~8)
LOAD_SLOT_JOINTS = {
    2: np.array([-267.47,   8.45, 34.61,  -1.11, 113.49,  1.83]),
    3: np.array([-252.14,  11.15, 31.41,  -7.71, 114.77, 13.17]),
    4: np.array([-239.48,  20.95, 17.82, -13.53, 120.01, 21.54]),
    5: np.array([-225.70,  -3.09, 48.50, -18.08, 116.14, 33.65]),
    6: np.array([-242.58, -10.98, 55.32, -11.60, 114.11, 20.65]),
    7: np.array([-284.66,   5.96, 33.57,   0.0,  140.47, -14.65]),
    8: np.array([-304.82,  19.69, 15.69,   0.0,  144.62, -34.82]),
}

# 슬롯별 웨이포인트: 슬롯 1은 독립 경로, 슬롯 2~8은 공통 경유점 + 슬롯별 최종 위치
SLOT_WAYPOINTS = {
    1: [
        np.array([-90.0, -20.81, 107.71, 0.0, 93.11, 0.0]),
        np.array([-15.0, -36.42, 117.55, 0.0, 98.86, 0.0]),
        np.array([35.0, 15.0, 23.0, 0.0, 100.0, 0.0]),
        np.array([73.17, 20.33, 29.56, 0.84, 127.89, -16.84]),
    ],
    **{slot: SLOT_COMMON_WPS + [joint] for slot, joint in LOAD_SLOT_JOINTS.items()}
}

# 슬롯 7, 8 언로드 전용 웨이포인트 (로드 경로와 다름)
UNLOAD_SLOT_WAYPOINTS = {
    7: [
        np.array([-90.0,  -20.81, 107.71,   0.0,  93.11,   0.0]),
        np.array([-15.0,  -36.42, 117.55,   0.0,  98.86,   0.0]),
        np.array([ 89.71, -21.9,   43.02, -12.68, 116.79,  11.67]),
        np.array([156.75, -85.1,  122.9,  -71.25,  76.01,  35.48]),
    ],
    8: [
        np.array([-90.0,  -20.81, 107.71,   0.0,  93.11,   0.0]),
        np.array([-15.0,  -36.42, 117.55,   0.0,  98.86,   0.0]),
        np.array([-30.34, -34.83, 116.39,  68.02,  96.42,  73.61]),
        np.array([ -5.72, -34.3,  116.0,   90.83,  95.6,   98.3]),
    ],
}
# 인덱스 0~5: 내려놓는 순서에 따라 사용 (unload 전용)
DELIVERY_WAYPOINTS = {
    0: [
        np.array([-106.29, 23.23, 96.43, 0.0, 60.35, -16.28]),
    ],
    1: [
        np.array([-91.40, 20.078, 100.7, 0.0, 59.22, -1.39]),
    ],
    2: [
        np.array([-75.10, 22.13, 97.94, 0.0, 59.94, 14.91]),
    ],
    3: [
        np.array([-78.43, 43.34, 66.28, 0.0, 70.38, 11.58]),
    ],
    4: [
        np.array([-90.71, 41.46, 69.26, 0.0, 69.28, -0.71]),
    ],
    5: [
        np.array([-103.28, 44.63, 64.19, 0.0, 71.19, -13.28]),
    ],
    # 6번: 슬롯1 전용 내려놓기 포인트.
    #      완성품 unload 는 delivery_idx 와 무관하게 무조건 이 포인트로 간다.
    6: [
        np.array([-88.55, 39.48, 126.07, -49.84, -63.94, 11.46]),
    ],
}

# 완성품 unload 후 비전 검증용 조인트 포인트.
# 실제 카메라가 내려놓은 완성품을 위에서 볼 수 있는 자세로 교체해서 사용한다.
PRODUCT_VERIFY_WAYPOINTS = {
    6: [
        np.array([-61.89, 12.56, 94.17, 0, 73.27, 28.11]),
    ],
}

# 완성품 층 그룹 — delivery 시 내려놓는 높이(z)가 그룹별로 다르다.
PRODUCT_FLOOR_1   = {34, 13, 81}               # 1층:   배터리, 마그넷, 이스탑
PRODUCT_FLOOR_2_5 = {442, 241, 462, 8518}      # 2층반: 당근, 신호등, 스몰트리, 버거
PRODUCT_FLOOR_2   = {711, 4482, 48132, 46262}  # 2층:   망치, 큰당근, 아이스크림, 빅트리

# 완성품(Products) ID 묶음 = 위 3개 층 그룹의 합집합.
# 이 집합에 속하면 unload 시 완성품 전용 경로를 탄다.
# (raw material 1~8 은 포함하지 않음 -> 기존 DELIVERY_WAYPOINTS 흐름 그대로)
FINISHED_PRODUCTS = PRODUCT_FLOOR_1 | PRODUCT_FLOOR_2_5 | PRODUCT_FLOOR_2

# 완성품 unload 시 사용할 고정 delivery 인덱스.
# 완성품은 처리 순서(delivery_idx)와 무관하게 항상 DELIVERY_WAYPOINTS[6] 으로 간다.
PRODUCT_DELIVERY_IDX = 6
PRODUCT_SLOT = 1  # 완제품 보관 슬롯 → 언로드 시 항상 PRODUCT_DELIVERY_IDX로 고정

# 경기 당일 워크벤치 스테이션 아이디를 여기에 입력
# 워크벤치 스테이션에서만 언로드 후 검증 로직 실행
WORKBENCH_STATION_IDS = {1}

# --- LOAD 비전/오프셋 상수 ---
CAM_X_OFF = -51.0
CAM_Y_OFF = 32.0
LOAD_Z_DOWN_MM = 55.0
LOAD_Z_UP_MM = -55.0
Z_OFFSET = -85.0
Z_MARGIN = 40.0
SCAN_Y_OFFSETS_MM = [0.0, 200.0, -200.0]
SCAN_Y_AXIS_INDEX = 1
SCAN_SETTLE_TIME_SEC = 0.3
SCAN_VISION_RETRIES_PER_POSE = 1
SCAN_MAX_CYCLES = 3
PRODUCT_VERIFY_SETTLE_TIME_SEC = 0.3
PRODUCT_VERIFY_VISION_RETRIES = 1
# 파지(grip) 직전, 이동 정지 후 기계 진동이 잦아들 시간(초). 최소값으로 잡음.
# 0 에 가까울수록 빠르지만, 흔들리는 중에 잡으면 파지 실패 위험 -> 0.05~0.1 권장.
GRIP_SETTLE_TIME_SEC = 0.1

# # --- LOAD yaw(rz) 보정 상수 ---
# # 특정 완성품은 파지 방향을 맞추기 위해 비전 yaw 에 고정 오프셋(deg)을 더한다.
# # e_stop(81), burger(8518), big_tree(46262) -> -90도
# # 여기 없는 object_id 는 오프셋 0 (비전 yaw 그대로 사용).
# YAW_OFFSET_DEG = {
#     81: -90.0,     # e_stop
#     8518: -90.0,   # burger
#     46262: 0.0,  # big_tree
# }

# --- 제품별 파지 오프셋 (LOAD 전용) ---
# YAW_OFFSET_DEG 와 같은 방식으로 object_id 별로 파지 보정을 따로 준다.
# 비전이 준 좌표 위에 "더해지는" 추가 보정값. 전역 CAM/Z 오프셋은 그대로 두고
# 제품마다 미세 보정만 얹는다. (yaw 가 p.yaw + offset 으로 더해지던 것과 동일 패턴)
#
#   x   : 최종 접근 move_l_rel 의 tool x 병진에 더할 값 (mm)
#   y   : 최종 접근 move_l_rel 의 tool y 병진에 더할 값 (mm)
#   z   : 파지 깊이(tool z)에 더할 값 (mm, +면 더 깊이)
#   yaw : 파지 회전(rz)에 더할 값 (deg)
#
# 생략한 키는 0. 테이블에 없는 object_id 도 전부 0.
PICK_OFFSET_DEFAULT = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}

PICK_OFFSET = {
    # --- Raw Materials ---
    1: {},  # 2x2_red
    2: {},  # 2x2_green
    3: {},  # 2x2_blue
    4: {},  # 2x2_yellow
    5: {},  # 4x2_red
    6: {},  # 4x2_green
    7: {},  # 4x2_blue
    8: {},  # 4x2_yellow
    # --- Products ---
    34:    {},               # battery
    13:    {},               # magnet
    81:    {'x': -10.0,'yaw': -90.0,'z': 10.0},   # e_stop
    442:   {},               # carrot
    241:   {},               # traffic_light
    462:   {'z': 10.0},               # small_tree
    711:   {'x': -10.0, 'z': 10.0},     # hammer 로봇베이스 기준 안쪽은 x+ 
    4482:  {'x': -10.0},               # big_carrot
    8518:  {'z': 20.0,'yaw': -90.0},   # burger
    48132: {'z': 10.0},               # ice_cream
    46262: {'z': 20.0},      # big_tree 벅서 빅트리 회전제한 -90~90
}


def get_pick_offset(object_id):
    """object_id 의 파지 오프셋을 (없는 키는 0 으로 채워) 반환한다."""
    off = dict(PICK_OFFSET_DEFAULT)
    off.update(PICK_OFFSET.get(object_id, {}))
    return off

# --- UNLOAD Z 상수 (슬롯에서 물체 집을 때, 슬롯 2~6) ---
UNLOAD_Z_DOWN_MM = 55.0
UNLOAD_Z_UP_MM = -55.0

# --- UNLOAD X 상수 (슬롯 7/8 언로드 시 Tool X 방향 이동) ---
UNLOAD_X_DOWN_MM = 90.0
UNLOAD_X_UP_MM = -90.0

# 슬롯별 Tool X 이동 방향 부호 (+1.0 = X+, -1.0 = X-)
UNLOAD_SLOT_X_DIR = {
    7:  1.0,
    8: -1.0,
}

# --- DELIVERY Z 상수 (배달 위치에서 물체 내려놓을 때, 일반 재료 전용) ---
DELIVERY_Z_DOWN_MM = 115.0
DELIVERY_Z_UP_MM = -115.0


# --- 완성품(Products) 전용 delivery Z 상수 (층별 3단계) ---
# 완성품은 6번 포인트에서 손목이 꺾여 Tool z축이 수직이 아니므로,
# delivery 내려놓기는 Base 프레임(중력 방향) 기준으로 수행한다.
# 아래 값은 "내려가는 거리(양수=깊이)" 로 정의한다. (코드에서 Base z- 로 변환)
# TODO: 각 층 실제 높이에 맞게 값 조정.
PRODUCT_FLOOR_1_Z_DOWN_MM   = 100.0   # 1층
PRODUCT_FLOOR_1_Z_UP_MM     = -100.0
PRODUCT_FLOOR_2_5_Z_DOWN_MM = 90.0    # 2층반
PRODUCT_FLOOR_2_5_Z_UP_MM   = -90.0
PRODUCT_FLOOR_2_Z_DOWN_MM   = 80.0    # 2층
PRODUCT_FLOOR_2_Z_UP_MM     = -80.0


# --- 모션 속도/가속 (이 4개 숫자가 로봇팔 속도를 전부 결정한다) ---
# move_j : J_VEL=deg/s,  J_ACC=deg/s^2   (관절 이동: 슬롯/홈/배달 등 긴 이동)
# move_l : L_VEL=mm/s,   L_ACC=mm/s^2    (직선 이동: 비전 접근/하강/상승 등)
#
# 값이 클수록 빠르다. 아래는 "빠르게" 세팅이다.
# 만약 너무 거칠거나(덜컹/오버슈트) 물건을 놓치면 숫자를 낮추면 된다.
#   - 더 빠르게 : J 500/1200, L 800/2000   (관절은 로봇 물리 한계에서 멈춤)
#   - 빠르게    : J 400/1000, L 700/1500    <- 현재값
#   - 보통      : J 300/600,  L 500/1000
#   - 느리게    : J 200/400,  L 400/800
# (가속 J_ACC 가 짧은 이동 속도를 가장 크게 좌우한다. 더 빠르게 하려면 J_ACC 먼저 올릴 것)
J_VEL, J_ACC = 400, 1000
L_VEL, L_ACC = 700, 1500

# --- 조립(ASSEMBLE) 상수 ---
# ASSEMBLY_WAYPOINTS: target_slot별 조립 위치까지의 경유 조인트 목록
#   - 딕셔너리 키 = target_slot (7 또는 8)
#   - 마지막 요소가 실제 조립 위치 (= ASSEMBLY_JOINT[target_slot])
#   - sequence_assemble 에서 순서대로 move_j 로 이동
ASSEMBLY_Z_DOWN_MM = 90.0   # layer 0 기준 블록 내려놓기 하강 거리 (mm)
ASSEMBLY_Z_UP_MM   = -90.0  # layer 0 기준 블록 내려놓기 상승 거리 (mm)
BLOCK_H_MM         = 19.0   # 블록 1개 높이 (mm)

# --- BigTree (46262) 전용 조립 상수 ---
BIG_TREE_STACK_X_OFFSET_MM   = 32.0    # 슬롯B X3 오프셋: 4x2/상단 2x2 배치 위치 (mm)
BIG_TREE_ASSEMBLE_Z_BASE_MM  = 70.0   # 조립 Z 기준값; 각 단계 = base - BLOCK_H_MM * n

# UNLOAD 픽업용 슬롯 조인트 (direct move_j, 중간 웨이포인트 없음)
# 키: slot 번호 (슬롯 2~6)
UNLOAD_SLOT_JOINTS = {
    20: np.array([-266.24, -9.60, 56.23, -1.59, 109.94, 2.91]),  # ref: [-266.24, -9.60, 56.23, -1.59, 109.94, 2.91]
    21: np.array([-266.49, -5.88, 51.62, -1.50, 110.83, 2.69]),  # ref: [-266.49, -5.88, 51.62, -1.49, 110.83, 2.69]
    22: np.array([-266.72, -1.93, 46.49, -1.41, 112.00, 2.49]),  # ref: [-266.72, -1.93, 46.49, -1.41, 112.01, 2.49]
    23: np.array([-266.91, 2.35, 40.65, -1.35, 113.55, 2.30]),  # ref: [-266.91, 2.35, 40.65, -1.35, 113.55, 2.30]
    24: np.array([-267.08, 7.17, 33.75, -1.29, 115.63, 2.12]),  # ref: [-267.08, 7.17, 33.75, -1.29, 115.63, 2.12]
    30: np.array([-246.77, -6.53, 53.50, -9.69, 111.00, 18.00]),  # ref: [-246.77, -6.53, 53.50, -9.69, 111.00, 18.00]
    31: np.array([-248.19, -2.84, 48.70, -9.18, 111.93, 16.71]),  # ref: [-248.19, -2.84, 48.70, -9.18, 111.93, 16.71]
    32: np.array([-249.46, 1.14, 43.29, -8.75, 113.20, 15.51]),  # ref: [-249.46, 1.14, 43.30, -8.75, 113.20, 15.51]
    33: np.array([-250.59, 5.55, 37.04, -8.40, 114.91, 14.36]),  # ref: [-250.59, 5.55, 37.04, -8.40, 114.91, 14.36]
    34: np.array([-251.61, 10.68, 29.39, -8.14, 117.30, 13.22]),  # ref: [-251.61, 10.68, 29.39, -8.14, 117.30, 13.22]
    40: np.array([-231.15, 0.67, 46.72, -15.76, 113.13, 30.14]),  # ref: [-231.15, 0.67, 46.72, -15.77, 113.13, 30.14]
    41: np.array([-233.11, 4.40, 41.34, -15.21, 114.33, 28.16]),  # ref: [-233.11, 4.40, 41.34, -15.21, 114.33, 28.16]
    42: np.array([-234.91, 8.62, 35.05, -14.76, 116.00, 26.21]),  # ref: [-234.91, 8.62, 35.05, -14.76, 116.01, 26.21]
    43: np.array([-236.56, 13.65, 27.25, -14.45, 118.41, 24.22]),  # ref: [-236.56, 13.65, 27.25, -14.45, 118.41, 24.22]
    44: np.array([-238.08, 20.63, 15.83, -14.46, 122.46, 21.87]),  # ref: [-238.08, 20.63, 15.83, -14.46, 122.46, 21.87]
    50: np.array([-209.45, -14.62, 62.23, -23.19, 118.28, 46.92]),  # ref: [-209.45, -14.62, 62.23, -23.19, 118.28, 46.92]
    51: np.array([-213.31, -12.27, 59.05, -22.13, 117.95, 43.60]),  # ref: [-213.31, -12.27, 59.05, -22.13, 117.95, 43.60]
    52: np.array([-216.85, -9.70, 55.58, -21.13, 117.85, 40.53]),  # ref: [-216.85, -9.70, 55.58, -21.13, 117.85, 40.53]
    53: np.array([-220.08, -6.91, 51.78, -20.19, 117.99, 37.69]),  # ref: [-220.08, -6.91, 51.78, -20.19, 117.99, 37.69]
    54: np.array([-223.02, -3.87, 47.55, -19.33, 118.40, 35.04]),  # ref: [-223.02, -3.87, 47.56, -19.33, 118.40, 35.04]
    60: np.array([-226.84, -23.79, 67.91, -17.83, 117.14, 32.37]),  # ref: [-226.84, -23.79, 67.91, -17.83, 117.14, 32.37]
    61: np.array([-231.08, -21.03, 64.89, -16.24, 116.54, 29.12]),  # ref: [-231.08, -21.03, 64.89, -16.24, 116.54, 29.12]
    62: np.array([-234.65, -18.13, 61.62, -14.88, 116.23, 26.36]),  # ref: [-234.66, -18.13, 61.62, -14.88, 116.22, 26.36]
    63: np.array([-237.69, -15.06, 58.07, -13.72, 116.18, 23.98]),  # ref: [-237.69, -15.06, 58.07, -13.72, 116.18, 23.98]
    64: np.array([-240.28, -11.83, 54.19, -12.73, 116.40, 21.91]),  # ref: [-240.28, -11.83, 54.19, -12.73, 116.40, 21.90]
    # 20: np.array([-266.65,-10.78, 60.26, -1.40, 107.07, 2.67]),
    # 21: np.array([-266.87, -7.18, 55.88, -1.31, 107.85, 2.47]),
    # 22: np.array([-267.06, -3.39, 51.06, -1.24, 108.88, 2.3]),
    # 23: np.array([-267.23, 0.65, 45.68, -1.18, 110.22, 2.14]),
    # 24: np.array([-267.38, 5.07, 39.51, -1.13, 111.96, 1.98]),
    # 30: np.array([-246.79, -8.18, 58.19, -9.5, 107.99, 18.52]),
    # 31: np.array([-248.19, -4.65, 53.71, -9.0, 108.77, 17.25]),
    # 32: np.array([-249.43, -0.91, 48.75, -8.56, 109.83, 16.07]),
    # 33: np.array([-250.55, 3.14, 43.16, -8.18, 111.23, 14.97]),
    # 34: np.array([-251.55, 7.64, 36.67, -7.88, 113.10, 13.91]),
    # 40: np.array([-231.78, -1.0, 51.34, -15.23, 110.13, 30.48]),
    # 41: np.array([-233.70, 2.51, 46.39, -14.67, 111.15, 28.58]),
    # 42: np.array([-235.45, 6.37, 40.76, -14.18, 112.53, 26.75]),
    # 43: np.array([-237.06, 10.76, 34.14, -13.78, 114.42, 24.94]),
    # 44: np.array([-238.53, 16.09, 25.78, -13.53, 117.13, 23.05]),
    # 50: np.array([-210.75, -15.79, 65.59, -22.38, 115.86, 46.85]),
    # 51: np.array([-214.49, -13.46, 62.51, -21.35, 115.50, 43.62]),
    # 52: np.array([-217.91, -10.93, 59.16, -20.37, 115.36, 40.63]),
    # 53: np.array([-221.04, -8.21, 55.51, -19.45, 115.44, 37.87]),
    # 54: np.array([-223.89, -5.26, 51.50, -18.61, 115.76, 35.31]),
    # 60: np.array([-228.30, -25.67, 71.33, -16.99, 114.78, 31.97]),
    # 61: np.array([-232.32, -22.33, 68.38, -15.49, 114.16, 28.85]),
    # 62: np.array([-235.12, -19.44, 65.21, -14.21, 113.81, 26.18]),
    # 63: np.array([-238.61, -16.42, 61.78, -13.11, 113.71, 23.89]),
    # 64: np.array([-241.08, -13.25, 58.07, -12.17, 113.86, 21.89]),
}

MATERIAL_NAMES = {
    # --- Raw Materials ---
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
    # --- Products ---
    34: "battery",
    13: "magnet",
    81: "e_stop",
    442: "carrot",
    241: "traffic_light",
    462: "small_tree",
    711: "hammer",
    4482: "big_carrot",
    8518: "burger",
    48132: "ice_cream",
    46262: "big_tree",
}

# 완성품별 조립 재료 시퀀스.
# 리스트 인덱스 = layer_index (0부터 시작).
# 값 = 재료 object_id.
# layer 0: ASSEMBLY_Z_DOWN_MM 그대로 하강.
# layer N: ASSEMBLY_Z_DOWN_MM - (BLOCK_H_MM * N) 만큼 하강.
ASSEMBLY_SEQUENCE = {
    34:   [3, 4],        # battery:       2x2파랑 → 2x2노랑
    13:   [1, 3],        # magnet:        2x2빨강 → 2x2파랑
    81:   [8, 1],        # e_stop:        4x2노랑 → 2x2빨강
    442:  [4, 4, 2],     # carrot:        2x2노랑 → 2x2노랑 → 2x2초록
    241:  [2, 4, 1],     # traffic_light: 2x2초록 → 2x2노랑 → 2x2빨강
    462:  [4, 6, 2],     # small_tree:    2x2노랑 → 4x2초록 → 2x2초록
    711:  [1, 1, 3],     # hammer:        2x2빨강 → 2x2빨강 → 2x2파랑
    4482: [4, 4, 8, 2],  # big_carrot:    2x2노랑 → 2x2노랑 → 4x2노랑 → 2x2초록
    # dict 형식: {'id': 재료id, 'layer': 높이레이어, 'x': Tool X 오프셋(mm)}
    # 같은 layer 값이 여러 번 나오면 같은 높이에서 x 위치만 달리해 배치한다.
    48132: [  # ice_cream
        {'id': 4, 'layer': 0, 'x':  0.0},
        {'id': 8, 'layer': 1, 'x':  0.0},
        {'id': 1, 'layer': 2, 'x':  16.0},
        {'id': 3, 'layer': 2, 'x': -16.0},
        {'id': 2, 'layer': 3, 'x':  0.0},
    ],
    8518: [   # burger
        {'id': 8, 'layer': 0, 'x':  0.0},
        {'id': 1, 'layer': 1, 'x':  32.0},
        {'id': 5, 'layer': 1, 'x': -16.0},
        {'id': 8, 'layer': 2, 'x':  0.0},
    ],
}


class AmrRobotNode(Node):
    """load / unload 통합 오케스트레이터.

    /arm_command 서비스 하나로 LOAD / UNLOAD 를 모두 처리한다.
    request.action 이 'LOAD' 이면 적재 시퀀스, 'UNLOAD' 이면 출고 시퀀스를 돈다.
    로봇 연결·busy 락·서비스는 모두 단일 인스턴스로 공유한다.
    """

    def __init__(self):
        super().__init__('amr_robot_node')
        self.cbg = ReentrantCallbackGroup()

        self.robot = None
        self.rc = None
        self.robot_data = None
        self.robot_ready = False

        try:
            self.robot = rb.Cobot(ROBOT_IP)
            self.rc = rb.ResponseCollector()
            self.robot.set_operation_mode(self.rc, rb.OperationMode.Real)
            self.robot.set_speed_bar(self.rc, 1.0)
            self.robot.set_speed_multiplier(self.rc, 1.5)
            self.robot_ready = True
            self.get_logger().info('[AMR] robot connected')
        except Exception as e:
            self.robot = None
            self.rc = None
            self.robot_ready = False
            self.get_logger().error(f'[AMR] robot connection error: {e}')

        # 현재 조인트 각도 읽기용 데이터 채널 (HOME 도착 여부 판정에 사용)
        try:
            self.robot_data = rb.CobotData(ROBOT_IP)
            self.get_logger().info('[AMR] data channel connected')
        except Exception as e:
            self.robot_data = None
            self.get_logger().warn(f'[AMR] data channel connect failed: {e}')

        self.vision_client = self.create_client(
            GetTargetPose, '/get_target_pose', callback_group=self.cbg)
        self.gripper_open_client = self.create_client(
            Trigger, '/gripper/open', callback_group=self.cbg)
        self.gripper_grip_client = self.create_client(
            Trigger, '/gripper/grip', callback_group=self.cbg)
        self.cargo_client = self.create_client(
            Cargo, '/cargo', callback_group=self.cbg)
        self.srv = self.create_service(
            ArmCommand, '/amr_robot_command', self.arm_robot_command_cb, callback_group=self.cbg)

        self._busy_lock = threading.Lock()
        self._busy = False

        self.get_logger().info('[AMR] amr_robot_node started')

        if self.robot_ready:
            t = threading.Thread(target=self._startup_move, daemon=True)
            t.start()

    # --- 상태 확인 헬퍼 ---

    def is_robot_ready(self):
        if not self.robot_ready or self.robot is None or self.rc is None:
            self.get_logger().error('[AMR] robot is not connected')
            return False
        return True

    def is_at_home(self, tol_deg=1.0):
        """현재 측정 조인트 각도(jnt_ang)를 읽어 HOME과 비교한다.
        데이터 채널이 없거나 읽기 실패 시 False를 반환해, 안전하게 실제 이동으로 폴백한다."""
        if self.robot_data is None:
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
            return bool(np.all(np.abs(cur - HOME_JOINT_DEG) <= tol_deg))
        except Exception as e:
            self.get_logger().warn(f'[AMR] is_at_home read failed: {e}')
            return False

    def is_at_moving_pose(self, tol_deg=1.0):
        """현재 조인트가 MOVING_JOINT_DEG와 일치하는지 확인한다."""
        if self.robot_data is None:
            return False
        try:
            data = self.robot_data.request_data(1.0)
            cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
            return bool(np.all(np.abs(cur - MOVING_JOINT_DEG) <= tol_deg))
        except Exception as e:
            self.get_logger().warn(f'[AMR] is_at_moving_pose read failed: {e}')
            return False

    # --- 서비스 호출 헬퍼 ---

    def call_service(self, client, request, timeout=10.0):
        """Call a ROS2 service from inside callbacks without nested spinning.
        This node runs under MultiThreadedExecutor with a ReentrantCallbackGroup.
        The current callback thread waits on an Event, while another executor
        thread processes the service response.
        """
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f'[AMR] service unavailable: {client.srv_name}')
                return None

            future = client.call_async(request)
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())

            if not done_event.wait(timeout=timeout):
                self.get_logger().error(f'[AMR] service timeout: {client.srv_name}')
                return None

            return future.result()
        except Exception as e:
            self.get_logger().error(f'[AMR] service call failed: {client.srv_name}: {e}')
            return None

    def call_vision(self, target_color, retries=3):
        for i in range(retries):
            req = GetTargetPose.Request()
            req.target_color = target_color
            req.target_size = ""
            res = self.call_service(self.vision_client, req, timeout=30.0)
            if res and res.success:
                return res
            self.get_logger().warn(f'[AMR] vision retry {i + 1}/{retries}')
            time.sleep(0.5)
        return None

    def _scan_y_delta(self, dy_mm):
        delta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        delta[SCAN_Y_AXIS_INDEX] = float(dy_mm)
        return delta

    def return_scan_center(self, current_y_offset_mm):
        if abs(current_y_offset_mm) < 1e-6:
            return True
        return self.move_l_rel_checked(
            self._scan_y_delta(-current_y_offset_mm),
            label='scan return center',
        )

    def call_vision_with_y_scan(self, target_color):
        current_y_offset = 0.0

        for cycle in range(SCAN_MAX_CYCLES):
            self.get_logger().info(f'[AMR] vision scan cycle {cycle + 1}/{SCAN_MAX_CYCLES}')
            for target_y_offset in SCAN_Y_OFFSETS_MM:
                delta_y = target_y_offset - current_y_offset
                if abs(delta_y) > 1e-6:
                    if not self.move_l_rel_checked(
                        self._scan_y_delta(delta_y),
                        label=f'scan y offset {target_y_offset:.0f}mm',
                    ):
                        self.get_logger().error(
                            f'[AMR] scan move failed: y={target_y_offset:.0f}mm')
                        self.return_scan_center(current_y_offset)
                        return None
                    current_y_offset = target_y_offset

                time.sleep(SCAN_SETTLE_TIME_SEC)
                self.get_logger().info(
                    f'[AMR] vision scan at y_offset={current_y_offset:.0f}mm')

                res = self.call_vision(
                    target_color,
                    retries=SCAN_VISION_RETRIES_PER_POSE,
                )
                if res:
                    self.get_logger().info(
                        f'[AMR] vision success at y_offset={current_y_offset:.0f}mm')
                    return res

            self.get_logger().warn(f'[AMR] vision scan cycle {cycle + 1} failed')

        self.get_logger().warn('[AMR] vision scan failed at all cycles')
        if not self.return_scan_center(current_y_offset):
            self.get_logger().error('[AMR] failed to return scan center')
        return None

    def call_gripper(self, grip: bool):
        client = self.gripper_grip_client if grip else self.gripper_open_client
        req = Trigger.Request()
        res = self.call_service(client, req, timeout=6.0)
        action_name = 'grip' if grip else 'open'
        if res and res.success:
            self.get_logger().info(f'[GRIPPER] {action_name}')
            return True
        self.get_logger().error(f'[GRIPPER] {action_name} failed')
        return False

    def call_cargo(self, action, slot=0, object_id=0, station_id=0):
        req = Cargo.Request()
        req.action = action
        req.slot = slot
        req.object_id = object_id
        req.station_id = station_id
        return self.call_service(self.cargo_client, req)

    # --- 로봇 이동 헬퍼 ---

    def wait_move(self, timeout=10.0, label='move'):
        if not self.is_robot_ready():
            return False
        try:
            result = self.robot.wait_for_move_finished(self.rc, timeout=timeout)
            if result is False:
                self.get_logger().error(f'[AMR] {label} wait returned False')
                return False
            return True
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} wait failed: {e}')
            return False

    def move_j_checked(self, joints_deg, label='move_j', timeout=10.0):
        if not self.is_robot_ready():
            return False
        if self.robot_data is not None:
            try:
                data = self.robot_data.request_data(1.0)
                cur = np.array([data.sdata.jnt_ang[i] for i in range(6)], dtype=float)
                if np.all(np.abs(cur - joints_deg) <= 1.0):
                    self.get_logger().info(f'[AMR] {label} already at target, skip')
                    return True
            except Exception:
                pass
        try:
            self.robot.move_j(self.rc, joints_deg, J_VEL, J_ACC)
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        return self.wait_move(timeout=timeout, label=label)

    def move_l_rel_checked(self, delta, label='move_l_rel', timeout=10.0,
                           ref_frame=None):
        if not self.is_robot_ready():
            return False
        if ref_frame is None:
            ref_frame = rb.ReferenceFrame.Tool
        try:
            self.get_logger().info(f'[AMR] command start {label}: {delta} (frame={ref_frame})')
            self.robot.move_l_rel(
                self.rc,
                np.array(delta, dtype=float),
                L_VEL,
                L_ACC,
                ref_frame,
            )
        except Exception as e:
            self.get_logger().error(f'[AMR] {label} command failed: {e}')
            return False
        ok = self.wait_move(timeout=timeout, label=label)
        if ok:
            self.get_logger().info(f'[AMR] command done {label}')
        return ok

    def _startup_move(self):
        """노드 시작 직후 MOVING_JOINT_DEG 자세로 직접 이동한다."""
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True
        try:
            if self.is_at_moving_pose():
                self.get_logger().info('[AMR] already at moving pose, skip startup move')
                return
            self.go_moving_pose()
        finally:
            with self._busy_lock:
                self._busy = False

    def go_home(self):
        return self.move_j_checked(HOME_JOINT_DEG, label='go_home')

    def go_moving_pose(self):
        if not self.move_j_checked(MOVING_JOINT_DEG, label='go_moving_pose'):
            return False
        self.get_logger().info('[AMR] moving pose reached')
        return True

    # --- 웨이포인트 이동 (action별 테이블을 인자로 받음) ---

    def move_to_slot(self, slot, for_unload=False, layer_index=0):
        # 슬롯 7/8 언로드는 전용 웨이포인트 사용
        if for_unload and slot in UNLOAD_SLOT_WAYPOINTS:
            waypoints = UNLOAD_SLOT_WAYPOINTS[slot]
        else:
            waypoints = SLOT_WAYPOINTS.get(slot)

        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        move_waypoints = list(waypoints)

        # UNLOAD_SLOT_WAYPOINTS 사용 시 마지막 WP가 이미 픽업 위치이므로 교체 안 함
        if for_unload and slot not in UNLOAD_SLOT_WAYPOINTS:
            unload_joint = UNLOAD_SLOT_JOINTS.get(slot * 10 + layer_index)
            if unload_joint is not None:
                move_waypoints[-1] = unload_joint

        for idx, wp in enumerate(move_waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] slot={slot} reached')
        return True

    def return_from_slot(self, slot, skip_last=False, for_unload=False):
        # 슬롯 7/8 언로드 복귀는 전용 웨이포인트 역순 사용
        if for_unload and slot in UNLOAD_SLOT_WAYPOINTS:
            waypoints = UNLOAD_SLOT_WAYPOINTS[slot]
        else:
            waypoints = SLOT_WAYPOINTS.get(slot)

        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for slot={slot}')
            return False

        # 역방향 첫 번째는 방금 도착한 최종 자세라서 스킵
        return_waypoints = list(reversed(waypoints))[1:]

        if skip_last:
            return_waypoints = return_waypoints[:-1]

        for idx, wp in enumerate(return_waypoints, start=2):
            if not self.move_j_checked(wp, label=f'return_from_slot({slot}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] returned from slot={slot}')
        return True

    def product_delivery_z(self, object_id):
        """완성품 object_id가 속한 층 그룹의 delivery (z_down, z_up) 을 돌려준다.
        어느 그룹에도 없으면 기본 UNLOAD Z 로 안전하게 폴백한다."""
        if object_id in PRODUCT_FLOOR_1:
            return PRODUCT_FLOOR_1_Z_DOWN_MM, PRODUCT_FLOOR_1_Z_UP_MM
        if object_id in PRODUCT_FLOOR_2_5:
            return PRODUCT_FLOOR_2_5_Z_DOWN_MM, PRODUCT_FLOOR_2_5_Z_UP_MM
        if object_id in PRODUCT_FLOOR_2:
            return PRODUCT_FLOOR_2_Z_DOWN_MM, PRODUCT_FLOOR_2_Z_UP_MM
        return UNLOAD_Z_DOWN_MM, UNLOAD_Z_UP_MM

    def move_to_delivery(self, delivery_idx):
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        for idx, wp in enumerate(waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_delivery({delivery_idx}) wp{idx}'):
                return False

        self.get_logger().info(f'[AMR] delivery position {delivery_idx} reached')
        return True

    def return_from_delivery(self, delivery_idx):
        # waypoint가 1개뿐이면 역순 복귀 없이 그대로 반환.
        # (move_j_checked가 zero-distance를 자동 skip하므로 별도 처리 불필요)
        # waypoint가 여러 개이면 마지막 자세를 제외한 경유점만 역순으로 탄다.
        waypoints = DELIVERY_WAYPOINTS.get(delivery_idx)
        if waypoints is None:
            self.get_logger().error(f'[AMR] no waypoints for delivery_idx={delivery_idx}')
            return False

        if len(waypoints) > 1:
            return_waypoints = list(reversed(waypoints))[1:]
            for idx, wp in enumerate(return_waypoints, start=1):
                if not self.move_j_checked(wp, label=f'return_from_delivery({delivery_idx}) wp{idx}'):
                    return False

        self.get_logger().info(f'[AMR] returned from delivery position {delivery_idx}')
        return True

    def move_to_product_verify(self):
        waypoints = PRODUCT_VERIFY_WAYPOINTS.get(PRODUCT_DELIVERY_IDX)
        if waypoints is None:
            self.get_logger().error('[AMR] no product verification waypoints')
            return False

        for idx, wp in enumerate(waypoints, start=1):
            if not self.move_j_checked(wp, label=f'move_to_product_verify wp{idx}'):
                return False

        self.get_logger().info('[AMR] product verification position reached')
        return True

    def verify_product_unload(self, object_id):
        if not self.move_to_product_verify():
            return False

        time.sleep(PRODUCT_VERIFY_SETTLE_TIME_SEC)
        res = self.call_vision(str(object_id), retries=PRODUCT_VERIFY_VISION_RETRIES)

        if res and res.success:
            self.get_logger().error(
                f'[UNLOAD VERIFY] object_id={object_id} still detected; treat as fail'
            )
            return False

        self.get_logger().info(
            f'[UNLOAD VERIFY] object_id={object_id} not detected; treat as success'
        )
        return True

    def pick_from_floor_by_vision(self, object_id, label_prefix='pick'):
        vision_target = str(object_id)

        if not self.call_gripper(False):
            return False

        if not self.go_home():
            return False

        p = self.call_vision_with_y_scan(vision_target)
        if not p:
            self.get_logger().error(f'[AMR] vision failed during {label_prefix}')
            self.go_home()
            return False

        off = get_pick_offset(object_id)
        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET
        yaw = p.yaw + off['yaw']

        tool_x = dy + off['x']
        tool_y = dx + off['y']
        tool_z = (z_move - Z_MARGIN) + off['z']

        if any(off[k] != 0.0 for k in ('x', 'y', 'z', 'yaw')):
            self.get_logger().info(
                f'[AMR] {label_prefix} offset applied: object_id={object_id}, '
                f'off={off}, vision_yaw={p.yaw:.2f} -> yaw={yaw:.2f}'
            )

        if not self.move_l_rel_checked(
            [tool_x, tool_y, tool_z, 0.0, 0.0, yaw],
            label=f'{label_prefix} yaw+xy+z approach',
        ):
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label=f'{label_prefix} z final approach',
        ):
            self.go_home()
            return False
        time.sleep(GRIP_SETTLE_TIME_SEC)

        if not self.call_gripper(True):
            self.get_logger().error(f'[AMR] {label_prefix} grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                label=f'{label_prefix} retreat after grip failure',
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label=f'{label_prefix} lift after grip',
        ):
            self.go_home()
            return False

        return True

    def place_at_delivery(self, object_id, delivery_idx, is_product):
        if not self.move_to_delivery(delivery_idx):
            self.go_home()
            return False

        if is_product:
            z_down, z_up = self.product_delivery_z(object_id)
            delivery_ref = rb.ReferenceFrame.Base
            delivery_down = [0.0, 0.0, -z_down, 0.0, 0.0, 0.0]
            delivery_up = [0.0, 0.0, -z_up, 0.0, 0.0, 0.0]
        else:
            delivery_ref = rb.ReferenceFrame.Tool
            delivery_down = [0.0, 0.0, DELIVERY_Z_DOWN_MM, 0.0, 0.0, 0.0]
            delivery_up = [0.0, 0.0, DELIVERY_Z_UP_MM, 0.0, 0.0, 0.0]

        if not self.move_l_rel_checked(
            delivery_down,
            label='delivery z down',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                delivery_up,
                label='retreat after delivery open failure',
                ref_frame=delivery_ref,
            )
            self.go_home()
            return False

        if not self.move_l_rel_checked(
            delivery_up,
            label='delivery z up',
            ref_frame=delivery_ref,
        ):
            self.go_home()
            return False

        return True

    def retry_product_unload_recovery(self, object_id):
        self.get_logger().warn(
            f'[UNLOAD VERIFY] recovery start: object_id={object_id}'
        )

        # 1. 검증 위치에서 바로 비전 탐색 후 파지 (yaw 먼저 돌리는 복구 전용 픽업)
        if not self.recovery_pick_by_vision(object_id, label_prefix='recovery'):
            return False

        # 2. 바로 배송 위치로 이동 및 하역
        if not self.place_at_delivery(
            object_id,
            PRODUCT_DELIVERY_IDX,
            is_product=True,
        ):
            return False

        return True
    
    def recovery_pick_by_vision(self, object_id, label_prefix='recovery'):

        vision_target = str(object_id)

        # 1. 그리퍼 열기
        if not self.call_gripper(False):
            return False

        # 2. 비전 탐색
        p = self.call_vision_with_y_scan(vision_target)
        if not p:
            self.get_logger().error(f'[AMR] vision failed during {label_prefix}')
            return False

        # 3. 좌표 및 오프셋 계산
        off = get_pick_offset(object_id)
        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET
        yaw = p.yaw

        tool_x = dy + off['x']
        tool_y = dx + off['y']
        tool_z = (z_move - Z_MARGIN) + off['z']

        if any(off[k] != 0.0 for k in ('x', 'y', 'z')):
            self.get_logger().info(
                f'[AMR] {label_prefix} offset applied: object_id={object_id}, '
                f'off={off}, vision_yaw(raw)={p.yaw:.2f}'
            )

        # 4. yaw 회전 선행
        if not self.move_l_rel_checked(
            [0.0, 0.0, 0.0, 0.0, 0.0, yaw],
            label=f'{label_prefix} yaw approach first',
        ):
            return False

        # Tool 좌표계가 yaw만큼 돌아갔으므로, 기존 카메라 기준의 x, y 이동량도 회전 변환 적용
        yaw_rad = np.radians(yaw)
        adj_tool_x = tool_x * np.cos(yaw_rad) + tool_y * np.sin(yaw_rad)
        adj_tool_y = -tool_x * np.sin(yaw_rad) + tool_y * np.cos(yaw_rad)

        # 5. 보정된 x, y, z(안전 마진까지) 대각선 하강
        if not self.move_l_rel_checked(
            [adj_tool_x, adj_tool_y, tool_z, 0.0, 0.0, 0.0],
            label=f'{label_prefix} xy+z approach',
        ):
            return False
        # 6. 최종 수직 하강
        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label=f'{label_prefix} z final approach',
        ):
            return False
        time.sleep(GRIP_SETTLE_TIME_SEC)

        # 7. 그리퍼 닫기 (파지)
        if not self.call_gripper(True):
            self.get_logger().error(f'[AMR] {label_prefix} grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                label=f'{label_prefix} retreat after grip failure',
            )
            return False

        # 8. 위로 들어 올리기 (Lift)
        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label=f'{label_prefix} lift after grip',
        ):
            return False

        return True

    # --- 서비스 콜백 (LOAD / UNLOAD 분기) ---

    def arm_robot_command_cb(self, request, response):
        response.slots = []
        response.object_ids = []

        action = request.action.upper()
        if action not in ('LOAD', 'UNLOAD', 'ASSEMBLE'):
            response.success = False
            response.message = f'unknown action: {request.action}'
            return response

        if not self.is_robot_ready():
            response.success = False
            response.message = 'robot not connected'
            return response

        with self._busy_lock:
            if self._busy:
                response.success = False
                response.message = 'busy'
                return response
            self._busy = True

        try:
            if action == 'LOAD':
                results = self.sequence_load_multi(list(request.object_ids))
            elif action == 'UNLOAD':
                results = self.sequence_unload_multi(
                    list(request.object_ids), request.station_id)
            else:
                results = self.sequence_assemble_multi(
                    list(request.object_ids), request.station_id)

            success_all = bool(results) and all(r['success'] for r in results)
            response.success = success_all
            response.slots = [r['slot'] for r in results]
            response.object_ids = [r['object_id'] for r in results]
            response.message = ', '.join(r['message'] for r in results)
        except Exception as e:
            self.get_logger().error(f'[AMR] exception: {e}')
            response.success = False
            response.slots = []
            response.object_ids = []
            response.message = str(e)
        finally:
            with self._busy_lock:
                self._busy = False

        return response

    # --- LOAD 시퀀스 ---

    def sequence_load_multi(self, object_ids):
        results = []
        last_idx = len(object_ids) - 1
        for idx, object_id in enumerate(object_ids):
            is_last = (idx == last_idx)
            result = self.sequence_load(object_id, is_last=is_last)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] load failed at object_id={object_id}, stopping')
                break

        self.go_moving_pose()
        return results

    def sequence_load(self, object_id, is_last=False):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        target_color = MATERIAL_NAMES.get(object_id)
        if not target_color:
            self.get_logger().error(f'[AMR] unknown object_id: {object_id}')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'unknown object_id={object_id}',
            }

        vision_target = str(object_id)

        self.get_logger().info(f'[LOAD START] object_id={object_id}, target={target_color}')

        # 1. 빈 슬롯 확인
        res = self.call_cargo('FIND_EMPTY', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] no empty slot')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'no empty slot',
            }
        slot = res.slot
        self.get_logger().info(f'[CARGO] empty slot: {slot}')

        # 2. 초기화
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        if not self.move_j_checked(VISION_LOAD_JOINT_DEG, label='vision load pose'):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'vision load pose failed',
            }

        # 3. 비전 자세 기준 center -> left -> right 순서로 탐색
        p = self.call_vision_with_y_scan(vision_target)
        if not p:
            self.get_logger().error('[AMR] vision failed')
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'OBJECT_NOT_FOUND',
            }

        # 4. YAW + XY + Z접근 동시 이동
        #    물체 바로 위(Z_MARGIN)까지 대각선으로 내려가고,
        #    최종 접근은 5번에서 수직으로 따로 한다. (대각선 최종접근은 파지 안정성 저하)
        off = get_pick_offset(object_id)
        dx = -(p.x * 1000.0) + CAM_Y_OFF
        dy = (p.y * 1000.0) + CAM_X_OFF
        z_move = (p.z * 1000.0) + Z_OFFSET

        yaw = p.yaw + off['yaw']

        tool_x = dy + off['x']
        tool_y = dx + off['y']
        tool_z = (z_move - Z_MARGIN) + off['z']

        if any(off[k] != 0.0 for k in ('x', 'y', 'z', 'yaw')):
            self.get_logger().info(
                f'[LOAD] pick offset applied: object_id={object_id}, '
                f'off={off}, vision_yaw={p.yaw:.2f} -> yaw={yaw:.2f}')

        if not self.move_l_rel_checked(
            [tool_x, tool_y, tool_z, 0.0, 0.0, yaw],
            label='yaw+xy+z approach',
        ):

            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'yaw+xy+z approach failed',
            }

        # 5. 수직 최종 접근 (yaw 회전 후에도 tool Z축은 수직 유지)
        if not self.move_l_rel_checked(
            [0.0, 0.0, Z_MARGIN, 0.0, 0.0, 0.0],
            label='z final approach',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'z final approach failed',
            }
        time.sleep(GRIP_SETTLE_TIME_SEC)

        # 6. 그리퍼 grip
        if not self.call_gripper(True):
            self.get_logger().error('[AMR] grip failed')
            self.move_l_rel_checked(
                [0.0, 0.0, -Z_MARGIN, 0.0, 0.0, 0.0],
                label='retreat after grip failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'grip failed',
            }

        # 7. Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
            label='lift after grip',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'lift after grip failed',
            }

        # 8. 웨이포인트 순서대로 슬롯으로 이동
        if not self.move_to_slot(slot):
            self.get_logger().error('[AMR] move to slot failed')
            self.go_home()
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 9. Z 하강 -> open -> Z 상승
        if not self.move_l_rel_checked(
            [0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
            label='place z down',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z down failed',
            }

        if not self.call_gripper(False):
            self.get_logger().error('[AMR] final gripper open failed')
            self.move_l_rel_checked(
                [0.0, 0.0, LOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                label='retreat after open failure',
            )
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'final gripper open failed',
            }

        if not self.move_l_rel_checked(
            [0.0, 0.0, LOAD_Z_UP_MM, 0.0, 0.0, 0.0],
            label='place z up',
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'place z up failed',
            }

        # 10. 웨이포인트 역순으로 복귀
        #     마지막 물체면 복귀 경로의 끝점(SLOT_COMMON_WPS[0])을 생략하고 바로 이동 포즈로 간다.
        if not self.return_from_slot(slot, skip_last=is_last):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'return from slot failed',
            }

        # 11. 카고 기록
        res = self.call_cargo('SET', slot=slot, object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo SET failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'loaded physically but cargo SET failed',
            }

        self.get_logger().info(f'[LOAD DONE] object_id={object_id}, slot={slot}')
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'load success',
        }

    # --- UNLOAD 시퀀스 ---

    def sequence_unload_multi(self, object_ids, station_id=0):
        results = []
        for idx, object_id in enumerate(object_ids):
            result = self.sequence_unload(object_id, idx, station_id=station_id)
            results.append(result)
            if not result['success']:
                self.get_logger().error(f'[AMR] unload failed at object_id={object_id}, stopping')
                break

        self.go_moving_pose()
        return results

    def sequence_unload(self, object_id, delivery_idx, station_id=0):
        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'robot not connected',
            }

        self.get_logger().info(f'[UNLOAD START] object_id={object_id}, delivery_idx={delivery_idx}')

        # 완성품 여부 판별: 완성품이면 delivery 단계에서 전용 포인트/전용 Z를 쓴다.
        is_product = object_id in FINISHED_PRODUCTS
        if is_product:
            self.get_logger().info(f'[UNLOAD] object_id={object_id} is a finished product')

        # 1. 슬롯 확인
        res = self.call_cargo('FIND_OBJECT', object_id=object_id)
        if not res or not res.success:
            self.get_logger().error(f'[AMR] object_id={object_id} not found in cargo')
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': f'object not found: {object_id}',
            }
        slot = res.slot
        layer_index = res.layer_index
        self.get_logger().info(f'[CARGO] object found: slot={slot}, layer_index={layer_index}')

        # 2. 초기화
        if not self.call_gripper(False):
            return {
                'success': False,
                'slot': -1,
                'object_id': object_id,
                'message': 'initial gripper open failed',
            }

        # 3. 웨이포인트 순서대로 슬롯으로 이동 (레이어별 UNLOAD 위치 사용)
        if not self.move_to_slot(slot, for_unload=True, layer_index=layer_index):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'move to slot failed',
            }

        # 4-6. 픽업 벡터 결정
        #   슬롯 7/8: Tool X 방향으로 이동
        #   슬롯 2~6: Tool Z 방향으로 이동 (완성품은 ASSEMBLY_Z, 재료는 UNLOAD_Z)
        if slot in UNLOAD_SLOT_WAYPOINTS:
            x_dir = UNLOAD_SLOT_X_DIR.get(slot, 1.0)
            pick_down = [UNLOAD_X_DOWN_MM * x_dir, 0.0, 0.0, 0.0, 0.0, 0.0]
            pick_up   = [UNLOAD_X_UP_MM   * x_dir, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            pickup_z_down = ASSEMBLY_Z_DOWN_MM if is_product else UNLOAD_Z_DOWN_MM
            pickup_z_up   = ASSEMBLY_Z_UP_MM   if is_product else UNLOAD_Z_UP_MM
            pick_down = [0.0, 0.0, pickup_z_down, 0.0, 0.0, 0.0]
            pick_up   = [0.0, 0.0, pickup_z_up,   0.0, 0.0, 0.0]

        # 4. 하강
        self.get_logger().info('[AMR] start slot pick down')
        if not self.move_l_rel_checked(
            pick_down,
            label='slot pick down',
            ref_frame=rb.ReferenceFrame.Tool,
        ):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot pick down failed',
            }

        # 5. 그리퍼 grip
        if not self.call_gripper(True):
            self.get_logger().error('[AMR] grip failed')
            self.move_l_rel_checked(
                pick_up,
                label='retreat after grip failure',
                ref_frame=rb.ReferenceFrame.Tool,
            )
            self.return_from_slot(slot, for_unload=True)
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'grip failed',
            }

        # 6. 상승
        self.get_logger().info('[AMR] start slot pick up')
        if not self.move_l_rel_checked(
            pick_up,
            label='slot pick up',
            ref_frame=rb.ReferenceFrame.Tool,
        ):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'slot pick up failed',
            }

        # 7. 슬롯에서 물체를 들어 올렸으므로 cargo 상태를 먼저 비운다.
        # 이후 복귀 실패가 나도 cargo_manager의 슬롯 상태는 실제 물리 상태와 맞는다.
        res = self.call_cargo('CLEAR', slot=slot, object_id=object_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo CLEAR failed')
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'object picked physically but cargo CLEAR failed',
            }

        # 8. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_slot(slot, for_unload=True):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'cargo CLEAR done, but return from slot failed',
            }

        # 9. 배달 위치로 이동
        #    슬롯 1(완제품 보관)은 항상 6번 고정, 슬롯 7/8(조립 완성품)은 delivery_idx 사용
        target_delivery_idx = PRODUCT_DELIVERY_IDX if slot == PRODUCT_SLOT else delivery_idx
        if not self.place_at_delivery(object_id, target_delivery_idx, is_product):
            self.go_home()
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'delivery placement failed',
            }

        if is_product and station_id in WORKBENCH_STATION_IDS and not self.verify_product_unload(object_id):
            if not self.retry_product_unload_recovery(object_id):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'product unload recovery failed',
                }

            if not self.verify_product_unload(object_id):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': object_id,
                    'message': 'product unload verification failed after recovery',
                }

        # 11. 웨이포인트 역순으로 홈 복귀
        if not self.return_from_delivery(target_delivery_idx):
            return {
                'success': False,
                'slot': slot,
                'object_id': object_id,
                'message': 'return from delivery failed',
            }

        self.get_logger().info(
            f'[UNLOAD DONE] object_id={object_id}, slot={slot}, delivery_idx={delivery_idx}'
        )
        return {
            'success': True,
            'slot': slot,
            'object_id': object_id,
            'message': 'unload success',
        }


    # --- ASSEMBLE 시퀀스 ---

    def sequence_assemble_multi(self, object_ids, target_slot=8):
        results = []
        for product_id in object_ids:
            result = self.sequence_assemble(product_id, target_slot=target_slot)
            results.append(result)
            if not result['success']:
                self.get_logger().error(
                    f'[AMR] assemble failed at product_id={product_id}, stopping')
                break

        self.return_from_slot(target_slot)
        self.go_moving_pose()
        return results

    def sequence_assemble(self, product_id, target_slot=8):
        if product_id == 46262:
            return self.sequence_assemble_big_tree(target_slot=target_slot)

        if not self.is_robot_ready():
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': 'robot not connected',
            }

        assembly_wps = SLOT_WAYPOINTS.get(target_slot)
        if assembly_wps is None:
            self.get_logger().error(
                f'[AMR] no slot waypoints for target_slot={target_slot}')
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': f'no slot waypoints for target_slot={target_slot}',
            }

        assembly_joint = assembly_wps[-1]  # SLOT_WAYPOINTS 마지막 WP = 조립 위치

        sequence = ASSEMBLY_SEQUENCE.get(product_id)
        if sequence is None:
            self.get_logger().error(
                f'[AMR] no assembly sequence for product_id={product_id}')
            return {
                'success': False,
                'slot': -1,
                'object_id': product_id,
                'message': f'no assembly sequence for product_id={product_id}',
            }

        self.get_logger().info(
            f'[ASSEMBLE START] product_id={product_id}, target_slot={target_slot}, steps={len(sequence)}')

        for enum_idx, step in enumerate(sequence):
            # int 형식: material_id만, layer=enumerate index, x_offset=0
            # dict 형식: {'id', 'layer', 'x'} — 같은 layer에 여러 블록 배치 가능
            if isinstance(step, dict):
                material_id = step['id']
                place_layer  = step['layer']
                x_offset     = step.get('x', 0.0)
            else:
                material_id = step
                place_layer  = enum_idx
                x_offset     = 0.0

            self.get_logger().info(
                f'[ASSEMBLE] enum={enum_idx}, material_id={material_id}, '
                f'place_layer={place_layer}, x_offset={x_offset}')

            # 1. 카고에서 재료 슬롯 확인
            res = self.call_cargo('FIND_OBJECT', object_id=material_id)
            if not res or not res.success:
                self.get_logger().error(
                    f'[AMR] material {material_id} not found in cargo')
                self.go_home()
                return {
                    'success': False,
                    'slot': -1,
                    'object_id': product_id,
                    'message': f'material {material_id} not found in cargo at enum={enum_idx}',
                }
            slot = res.slot
            cargo_layer = res.layer_index
            self.get_logger().info(
                f'[ASSEMBLE] material_id={material_id} -> slot={slot}, cargo_layer={cargo_layer}')

            # 2. 그리퍼 열기
            if not self.call_gripper(False):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'gripper open failed at enum={enum_idx}',
                }

            # 3. 재료 슬롯으로 이동
            if enum_idx == 0:
                # 첫 번째 재료: 이동 포즈에서 웨이포인트 경유
                if not self.move_to_slot(slot, for_unload=True, layer_index=cargo_layer):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'move to slot={slot} failed at enum={enum_idx}',
                    }
            else:
                # 이후 재료: 조립위치에서 직접 이동 (웨이포인트 없음)
                slot_joint = UNLOAD_SLOT_JOINTS.get(slot * 10 + cargo_layer)
                if slot_joint is None:
                    self.get_logger().error(
                        f'[AMR] no unload slot joint for slot={slot} cargo_layer={cargo_layer}')
                    self.go_home()
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'no assembly slot joint for slot={slot} cargo_layer={cargo_layer}',
                    }
                if not self.move_j_checked(
                    slot_joint, label=f'assemble to slot={slot} cargo_layer={cargo_layer}'
                ):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'move to slot={slot} failed at enum={enum_idx}',
                    }

            # 5. Z 하강
            if not self.move_l_rel_checked(
                [0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                label=f'assemble slot={slot} z down',
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'slot z down failed at enum={enum_idx}',
                }

            # 6. 그리퍼 grip
            if not self.call_gripper(True):
                self.get_logger().error(
                    f'[AMR] assemble grip failed at slot={slot}')
                self.move_l_rel_checked(
                    [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                    label='retreat after grip failure',
                )
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'grip failed at enum={enum_idx}',
                }

            # 7. Z 상승
            if not self.move_l_rel_checked(
                [0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                label=f'assemble slot={slot} z up',
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'slot z up failed at enum={enum_idx}',
                }

            # 8. 카고 슬롯 비우기
            res = self.call_cargo('CLEAR', slot=slot, object_id=material_id)
            if not res or not res.success:
                self.get_logger().error('[AMR] cargo CLEAR failed')
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'cargo CLEAR failed at enum={enum_idx}',
                }

            # 9. 조립 위치로 직접 이동 (경유 없이 assembly_joint 로)
            if not self.move_j_checked(
                assembly_joint,
                label=f'assemble return to assembly position enum={enum_idx}',
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'return to assembly position failed at enum={enum_idx}',
                }

            # 10. place_layer 기준 z 하강 거리 계산 (높은 층일수록 덜 내려감)
            z_down = ASSEMBLY_Z_DOWN_MM - (BLOCK_H_MM * place_layer)

            # 10a. X 오프셋 이동 (dict 형식에서 같은 layer 내 위치 분리)
            if abs(x_offset) > 1e-6:
                if not self.move_l_rel_checked(
                    [x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                    label=f'assemble x offset={x_offset} enum={enum_idx}',
                    ref_frame=rb.ReferenceFrame.Tool,
                ):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'assembly x offset failed at enum={enum_idx}',
                    }

            # 10b. Z 하강 (Tool 기준)
            if not self.move_l_rel_checked(
                [0.0, 0.0, z_down, 0.0, 0.0, 0.0],
                label=f'assemble place z down place_layer={place_layer}',
                ref_frame=rb.ReferenceFrame.Tool,
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly z down failed at enum={enum_idx}',
                }

            # 11. 그리퍼 열기 (블록 내려놓기)
            if not self.call_gripper(False):
                self.get_logger().error('[AMR] assembly gripper open failed')
                self.move_l_rel_checked(
                    [0.0, 0.0, -z_down, 0.0, 0.0, 0.0],
                    label='retreat z after assembly open failure',
                    ref_frame=rb.ReferenceFrame.Tool,
                )
                if abs(x_offset) > 1e-6:
                    self.move_l_rel_checked(
                        [-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                        label='retreat x after assembly open failure',
                        ref_frame=rb.ReferenceFrame.Tool,
                    )
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly gripper open failed at enum={enum_idx}',
                }

            # 12. Z 상승 (Tool 기준)
            if not self.move_l_rel_checked(
                [0.0, 0.0, -z_down, 0.0, 0.0, 0.0],
                label=f'assemble place z up place_layer={place_layer}',
                ref_frame=rb.ReferenceFrame.Tool,
            ):
                self.go_home()
                return {
                    'success': False,
                    'slot': slot,
                    'object_id': product_id,
                    'message': f'assembly z up failed at enum={enum_idx}',
                }

            # 12a. X 오프셋 복귀 (assembly_joint 기준 위치로 되돌림)
            if abs(x_offset) > 1e-6:
                if not self.move_l_rel_checked(
                    [-x_offset, 0.0, 0.0, 0.0, 0.0, 0.0],
                    label=f'assemble x return enum={enum_idx}',
                    ref_frame=rb.ReferenceFrame.Tool,
                ):
                    self.go_home()
                    return {
                        'success': False,
                        'slot': slot,
                        'object_id': product_id,
                        'message': f'assembly x return failed at enum={enum_idx}',
                    }

        # 13. cargo 등록
        res = self.call_cargo('SET', slot=target_slot, object_id=product_id)
        if not res or not res.success:
            self.get_logger().error('[AMR] cargo SET for assembled product failed')
            return {
                'success': False,
                'slot': target_slot,
                'object_id': product_id,
                'message': 'assembled physically but cargo SET failed',
            }

        self.get_logger().info(f'[ASSEMBLE DONE] product_id={product_id}, slot={target_slot}')
        return {
            'success': True,
            'slot': target_slot,
            'object_id': product_id,
            'message': 'assemble success',
        }

    def sequence_assemble_big_tree(self, target_slot=8):
        """46262 BigTree 전용 조립 시퀀스.

        프리어셈블  2x2_green(2A) → slot-B LOAD 위치 배치
        스텝 1      4x2_green(6B) → slot-B X3 배치 (layer 1)
        스텝 2      2x2_green(2C) → slot-B X3 하강 → 재파지 → 전체 스택 리프트
        스텝 3      RZ 90° → assembly_joint z 하강 → 릴리즈 → 30mm 상승
        """
        product_id = 46262

        if not self.is_robot_ready():
            return {'success': False, 'slot': -1, 'object_id': product_id, 'message': 'robot not connected'}

        assembly_wps = SLOT_WAYPOINTS.get(target_slot)
        if assembly_wps is None:
            self.get_logger().error(f'[BIG_TREE] no slot waypoints for target_slot={target_slot}')
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'no slot waypoints for target_slot={target_slot}'}

        assembly_joint = assembly_wps[-1]

        self.get_logger().info(f'[BIG_TREE START] product_id={product_id}, target_slot={target_slot}')

        # ── 프리어셈블: 2x2_green(2) 파지 → slot-B LOAD 위치에 배치 ─────────
        res = self.call_cargo('FIND_OBJECT', object_id=2)
        if not res or not res.success:
            self.get_logger().error('[BIG_TREE] pre_assemble: 2x2_green not found')
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: 2x2_green not found'}
        slot_2a, layer_2a = res.slot, res.layer_index

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: gripper open failed'}

        if not self.move_to_slot(slot_2a, for_unload=True, layer_index=layer_2a):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: move to 2A slot failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='pre_assemble 2A z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: 2A z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, -UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                     label='pre_assemble 2A retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: 2A grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='pre_assemble 2A z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: 2A z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_2a, object_id=2)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_2a, 'object_id': product_id,
                    'message': 'pre_assemble: cargo CLEAR 2A failed'}

        if not self.move_to_slot(target_slot):  # for_unload=False → LOAD 경로
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: move to slot-B load pos failed'}

        if not self.move_l_rel_checked([0.0, 0.0, LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='pre_assemble slot-B z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: slot-B place z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                     label='pre_assemble slot-B retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: slot-B release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -LOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='pre_assemble slot-B z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'pre_assemble: slot-B place z up failed'}

        # ── 스텝 1: 4x2_green(6) 파지 → slot-B X3 위치에 배치 (layer 1) ──────
        res = self.call_cargo('FIND_OBJECT', object_id=6)
        if not res or not res.success:
            self.get_logger().error('[BIG_TREE] step1: 4x2_green not found')
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: 4x2_green not found'}
        slot_6b, layer_6b = res.slot, res.layer_index

        slot_joint_6b = UNLOAD_SLOT_JOINTS.get(slot_6b * 10 + layer_6b)
        if slot_joint_6b is None:
            self.get_logger().error(f'[BIG_TREE] step1: no unload joint for slot={slot_6b} layer={layer_6b}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'step1: no unload joint for slot={slot_6b} layer={layer_6b}'}

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: gripper open failed'}

        if not self.move_j_checked(slot_joint_6b, label=f'step1 to slot={slot_6b}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'step1: move to slot={slot_6b} failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='step1 6B cargo z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: 6B z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                     label='step1 6B retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: 6B grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                        label='step1 6B cargo z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: 6B z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_6b, object_id=6)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_6b, 'object_id': product_id,
                    'message': 'step1: cargo CLEAR 6B failed'}

        if not self.move_j_checked(assembly_joint, label='step1 to assembly_joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: move to assembly_joint failed'}

        if not self.move_l_rel_checked([BIG_TREE_STACK_X_OFFSET_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        label='step1 X3 offset',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: X3 offset failed'}

        z1 = BIG_TREE_ASSEMBLE_Z_BASE_MM - BLOCK_H_MM * 1  # 100 - 19 = 81mm
        if not self.move_l_rel_checked([0.0, 0.0, z1, 0.0, 0.0, 0.0],
                                        label='step1 z down',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: z down failed'}

        if not self.call_gripper(False):
            self.move_l_rel_checked([0.0, 0.0, -z1, 0.0, 0.0, 0.0],
                                     label='step1 retreat', ref_frame=rb.ReferenceFrame.Tool)
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z1, 0.0, 0.0, 0.0],
                                        label='step1 z up',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: z up failed'}

        if not self.move_l_rel_checked([-BIG_TREE_STACK_X_OFFSET_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        label='step1 X3 return',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step1: X3 return failed'}

        # ── 스텝 2: 2x2_green(2) 파지 → slot-B X3 하강 → 재파지 → 전체 스택 리프트
        res = self.call_cargo('FIND_OBJECT', object_id=2)
        if not res or not res.success:
            self.get_logger().error('[BIG_TREE] step2: 2x2_green not found')
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: 2x2_green not found'}
        slot_2c, layer_2c = res.slot, res.layer_index

        slot_joint_2c = UNLOAD_SLOT_JOINTS.get(slot_2c * 10 + layer_2c)
        if slot_joint_2c is None:
            self.get_logger().error(f'[BIG_TREE] step2: no unload joint for slot={slot_2c} layer={layer_2c}')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'step2: no unload joint for slot={slot_2c} layer={layer_2c}'}

        if not self.call_gripper(False):
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: gripper open failed'}

        if not self.move_j_checked(slot_joint_2c, label=f'step2 to slot={slot_2c}'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': f'step2: move to slot={slot_2c} failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_DOWN_MM, 0.0, 0.0, 0.0],
                                        label='step2 2C cargo z down'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: 2C z down failed'}

        if not self.call_gripper(True):
            self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                     label='step2 2C retreat')
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: 2C grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, UNLOAD_Z_UP_MM, 0.0, 0.0, 0.0],
                                        label='step2 2C cargo z up'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: 2C z up failed'}

        res = self.call_cargo('CLEAR', slot=slot_2c, object_id=2)
        if not res or not res.success:
            self.go_home()
            return {'success': False, 'slot': slot_2c, 'object_id': product_id,
                    'message': 'step2: cargo CLEAR 2C failed'}

        if not self.move_j_checked(assembly_joint, label='step2 to assembly_joint'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: move to assembly_joint failed'}

        if not self.move_l_rel_checked([BIG_TREE_STACK_X_OFFSET_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        label='step2 X3 offset',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: X3 offset failed'}

        z2 = BIG_TREE_ASSEMBLE_Z_BASE_MM - BLOCK_H_MM * 2  # 100 - 38 = 62mm
        if not self.move_l_rel_checked([0.0, 0.0, z2, 0.0, 0.0, 0.0],
                                        label='step2 2C z down to stack',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: 2C z down to stack failed'}

        # 재파지: 2C가 4x2에 연결된 전체 스택(2x2+4x2+2x2)을 함께 파지
        if not self.call_gripper(True):
            self.get_logger().error('[BIG_TREE] step2: re-grip failed')
            self.move_l_rel_checked([0.0, 0.0, -z2, 0.0, 0.0, 0.0],
                                     label='step2 re-grip retreat', ref_frame=rb.ReferenceFrame.Tool)
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: re-grip failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z2, 0.0, 0.0, 0.0],
                                        label='step2 stack lift',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: stack lift failed'}

        if not self.move_l_rel_checked([-BIG_TREE_STACK_X_OFFSET_MM, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        label='step2 X3 return',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step2: X3 return failed'}

        # ── 스텝 3: RZ 90° → 블록4 위치 z 하강 → 릴리즈 → 30mm 상승 ──────────
        if not self.move_l_rel_checked([0.0, 0.0, 0.0, 0.0, 0.0, 90.0],
                                        label='step3 rz 90'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step3: rz rotation failed'}

        if not self.move_j_checked(assembly_joint, label='step3 to block4 pos'):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step3: move to block4 pos failed'}

        z3 = BIG_TREE_ASSEMBLE_Z_BASE_MM - BLOCK_H_MM * 3  # 100 - 57 = 43mm
        if not self.move_l_rel_checked([0.0, 0.0, z3, 0.0, 0.0, 0.0],
                                        label='step3 z down',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step3: z down failed'}

        if not self.call_gripper(False):
            self.get_logger().error('[BIG_TREE] step3: release failed')
            self.move_l_rel_checked([0.0, 0.0, -z3, 0.0, 0.0, 0.0],
                                     label='step3 retreat', ref_frame=rb.ReferenceFrame.Tool)
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step3: release failed'}

        if not self.move_l_rel_checked([0.0, 0.0, -z3, 0.0, 0.0, 0.0],
                                        label='step3 z up',
                                        ref_frame=rb.ReferenceFrame.Tool):
            self.go_home()
            return {'success': False, 'slot': -1, 'object_id': product_id,
                    'message': 'step3: 30mm up failed'}

        # ── cargo 등록 ────────────────────────────────────────────────────────
        res = self.call_cargo('SET', slot=target_slot, object_id=product_id)
        if not res or not res.success:
            self.get_logger().error('[BIG_TREE] cargo SET failed')
            return {'success': False, 'slot': target_slot, 'object_id': product_id,
                    'message': 'assembled but cargo SET failed'}

        self.get_logger().info(f'[ASSEMBLE DONE] big_tree product_id={product_id}, slot={target_slot}')
        return {'success': True, 'slot': target_slot, 'object_id': product_id, 'message': 'assemble success'}


def main(args=None):
    rclpy.init(args=args)
    node = AmrRobotNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()