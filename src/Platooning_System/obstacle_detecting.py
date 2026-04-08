import cv2
import numpy as np

def detect_obstacle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # 🔥 근거리 ROI: 아래 30%, 중앙 20%
    x1 = int(w * 0.40)
    x2 = int(w * 0.60)
    y1 = int(h * 0.55)
    y2 = int(h * 0.80)

    roi = gray[y1:y2, x1:x2]

    # 정상 영역 (도로=밝음, 차선=어두움)
    white_mask = roi > 170
    black_mask = roi < 70
    normal_mask = np.logical_or(white_mask, black_mask)

    abnormal_ratio = (roi.size - normal_mask.sum()) / roi.size

    # 🔥 승현님이 보낸 박스 크기 기준 → 약 20% 이상일 때만 True
    if abnormal_ratio > 0.20:
        return True

    return False
