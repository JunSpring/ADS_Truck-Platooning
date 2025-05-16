import math
import numpy as np

def convert_pixel_to_world(u, v, cx=320, cy=240, fov_x=90.0, x=2.3, y=0.0, z=3.5, pitch=-45.0):
    # ───── 기본 파라미터 ─────
    # cx, cy   = 320, 240           # principal point
    # fov_x    = 90.0               # [deg]
    fx = fy = cx / math.tan(math.radians(fov_x / 2))

    cam_pos  = np.array([x, y, z])   # [m]  (0,0,3.5)

    pitch = math.radians(pitch)            # ↓ 45°
    Rx = np.array([[1, 0, 0],
                [0,  math.cos(pitch), -math.sin(pitch)],
                [0,  math.sin(pitch),  math.cos(pitch)]])

    align = np.array([[ 0,  0,  1],
                    [-1,  0,  0],
                    [ 0, -1,  0]])

    R_c2w = align @ Rx                     # 카메라 → 월드

    def pixel_to_world(u, v):
        # 1) 픽셀 → 정규화 시선벡터(카메라)
        d_c = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_c /= np.linalg.norm(d_c)

        # 2) 월드로 회전
        d_w = R_c2w @ d_c

        # 3) 지면(z=0)과 교차
        t = -cam_pos[2] / d_w[2]
        return cam_pos + t * d_w           # (X, Y, Z)

    coordinate = (tuple(pixel_to_world(u, v)[0:2]))
    return coordinate
