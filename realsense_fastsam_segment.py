import pyrealsense2 as rs
import numpy as np
import cv2
import time
import torch
from ultralytics import YOLO, FastSAM

MODEL_PATH = "/home/user/Sudo_finalproject-main/proto.pt"
DEVICE = 0 if torch.cuda.is_available() else "cpu"
WINDOW_NAME = "proto.pt (known=green / unknown=colors)"

PALETTE = [
    (255,  60,  60),
    ( 60,  60, 255),
    (255, 160,   0),
    (160,   0, 200),
    (  0, 200, 200),
    (200, 200,   0),
    (  0, 200,  80),
    (200,  80, 160),
]

# ===== D455F (848x480@60 color 미지원 → 640x480@30) =====
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# ===== 모델 =====
# proto.pt: known 분류 / FastSAM: 프레임 전체 세그 → unknown 후보
model = YOLO(MODEL_PATH)
fastsam = FastSAM("FastSAM-s.pt")

YOLO_KW = dict(imgsz=640, conf=0.15, half=DEVICE == 0, device=DEVICE, verbose=False)
SAM_KW = dict(imgsz=640, conf=0.5, iou=0.7, half=DEVICE == 0,
              device=DEVICE, retina_masks=True, verbose=False)

# ROI: 화면 중앙에서 오른쪽 74px, 아래 10px, 크기 360x240
ROI_W, ROI_H = 360, 240
ROI_SHIFT_X = 74
ROI_SHIFT_Y = 10


def roi_rect(frame_w: int, frame_h: int) -> tuple[int, int, int, int]:
    cx = frame_w // 2 + ROI_SHIFT_X
    cy = frame_h // 2 + ROI_SHIFT_Y
    x1 = cx - ROI_W // 2
    y1 = cy - ROI_H // 2
    return x1, y1, x1 + ROI_W, y1 + ROI_H


# 물체 크기 필터 (ROI 360x240 = 86,400px 기준)
MIN_AREA =  500 # 줄일수록 작은거 잘 잡힘
MAX_AREA = 10_000  # 줄일 수 록 큰거 잘 안잡힘


def mask_iou(seg: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> float:
    """FastSAM 마스크(bool) 와 YOLO bbox 의 IoU."""
    bbox_mask = np.zeros(seg.shape, np.uint8)
    bbox_mask[y1:y2, x1:x2] = 1
    seg_bin = (seg > 0).astype(np.uint8)
    inter = (seg_bin & bbox_mask).sum()
    union = (seg_bin | bbox_mask).sum()
    return float(inter) / float(union) if union > 0 else 0.0


class UnknownTracker:
    """
    프레임 간 unknown 물체 ID를 유지하는 centroid 트래커.
    이전 프레임 중심점과 가장 가까운 새 마스크를 같은 물체로 판단.
    """
    def __init__(self, max_dist: int = 80, max_age: int = 12):
        self.max_dist = max_dist  # 같은 물체로 볼 최대 중심점 거리 (px)
        self.max_age  = max_age   # 감지 안 돼도 ID 유지할 프레임 수
        self._tracks  = {}        # id → {cx, cy, age, color_idx}
        self._next_id = 1

    def update(self, segs: list) -> list:
        """
        segs: unknown bool 마스크 리스트
        반환: (id, color_idx) 리스트 (segs 와 순서 동일)
        """
        # 새 마스크 중심점 계산
        centroids = []
        for seg in segs:
            ys, xs = np.where(seg)
            centroids.append((int(xs.mean()), int(ys.mean())))

        # 기존 트랙 age 증가 / 만료 삭제
        for tid in list(self._tracks):
            self._tracks[tid]['age'] += 1
            if self._tracks[tid]['age'] > self.max_age:
                del self._tracks[tid]

        # 탐욕 매칭: 각 centroid 에 가장 가까운 기존 트랙 할당
        used = set()
        assignments = []
        for cx, cy in centroids:
            best_tid, best_dist = None, self.max_dist
            for tid, tr in self._tracks.items():
                if tid in used:
                    continue
                dist = ((cx - tr['cx']) ** 2 + (cy - tr['cy']) ** 2) ** 0.5
                if dist < best_dist:
                    best_dist, best_tid = dist, tid

            if best_tid is not None:
                self._tracks[best_tid].update(cx=cx, cy=cy, age=0)
                used.add(best_tid)
                assignments.append((best_tid, self._tracks[best_tid]['color_idx']))
            else:
                new_id = self._next_id
                self._next_id += 1
                self._tracks[new_id] = dict(cx=cx, cy=cy, age=0,
                                            color_idx=(new_id - 1) % len(PALETTE))
                used.add(new_id)
                assignments.append((new_id, self._tracks[new_id]['color_idx']))

        return assignments


tracker = UnknownTracker(max_dist=80, max_age=12)
fps_val, t_fps, n_frames = 0.0, time.perf_counter(), 0
fullscreen = False

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

try:
    while True:
        frames      = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        color = np.asarray(color_frame.get_data())
        H, W = color.shape[:2]
        rx1, ry1, rx2, ry2 = roi_rect(W, H)
        roi = color[ry1:ry2, rx1:rx2]
        RH, RW = roi.shape[:2]

        # ── YOLO (proto.pt): known bbox (ROI 내부만) ─────────────────────────
        yolo_boxes = []
        r = model(roi, **YOLO_KW)[0]
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_name = model.names[int(box.cls[0])]
            yolo_boxes.append((x1, y1, x2, y2, float(box.conf[0]), cls_name))

        # ── FastSAM: ROI 세그 (unknown 후보) ─────────────────────────────────
        sam_res = fastsam(roi, **SAM_KW)[0]
        sam_masks = []
        if sam_res.masks is not None:
            for m in sam_res.masks.data.cpu().numpy():
                if m.shape[:2] != (RH, RW):
                    m = cv2.resize(m, (RW, RH), interpolation=cv2.INTER_NEAREST)
                sam_masks.append(m > 0.5)

        # ── YOLO bbox 와 매칭된 FastSAM → known, 나머지 → unknown ───────────
        known_segs = []   # (mask, conf, cls_name)
        matched_sam = set()

        for x1, y1, x2, y2, conf, cls_name in yolo_boxes:
            best_iou, best_idx = 0.0, -1
            for i, seg in enumerate(sam_masks):
                iou = mask_iou(seg, x1, y1, x2, y2)
                if iou > best_iou:
                    best_iou, best_idx = iou, i

            if best_iou >= 0.15 and best_idx >= 0:
                known_segs.append((sam_masks[best_idx], conf, cls_name))
                matched_sam.add(best_idx)
            else:
                bbox_mask = np.zeros((RH, RW), bool)
                bbox_mask[y1:y2, x1:x2] = True
                known_segs.append((bbox_mask, conf, cls_name))

        unknown_segs = []
        for i, seg in enumerate(sam_masks):
            if i in matched_sam:
                continue
            area = seg.sum()
            if area < MIN_AREA or area > MAX_AREA:
                continue
            unknown_segs.append(seg)
    
        # ── 트래킹: unknown ID 유지 ───────────────────────────────────────
        track_ids = tracker.update(unknown_segs)  # [(id, color_idx), ...]

        # ── 시각화 (ROI 내부만 오버레이, 바깥은 어둡게) ─────────────────────
        vis = (color.astype(np.float32) * 0.35).astype(np.uint8)
        roi_base = color[ry1:ry2, rx1:rx2].copy()
        roi_overlay = roi_base.copy()

        for seg, (tid, cidx) in zip(unknown_segs, track_ids):
            roi_overlay[seg] = PALETTE[cidx]

        for seg, _, __ in known_segs:
            roi_overlay[seg] = (0, 220, 0)

        roi_vis = cv2.addWeighted(roi_base, 0.45, roi_overlay, 0.55, 0)

        for seg, (tid, cidx) in zip(unknown_segs, track_ids):
            contours, _ = cv2.findContours(seg.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(roi_vis, contours, -1, PALETTE[cidx], 2)
            if contours:
                x, y, w, h = cv2.boundingRect(contours[0])
                cv2.putText(roi_vis, f"unknown{tid}", (x, max(y - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            PALETTE[cidx], 1, cv2.LINE_AA)

        for seg, conf, cls_name in known_segs:
            contours, _ = cv2.findContours(seg.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(roi_vis, contours, -1, (0, 220, 0), 2)
            if contours:
                x, y, w, h = cv2.boundingRect(contours[0])
                cv2.putText(roi_vis, f"{cls_name} {conf:.2f}", (x, max(y - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1, cv2.LINE_AA)

        vis[ry1:ry2, rx1:rx2] = roi_vis
        cv2.rectangle(vis, (rx1, ry1), (rx2 - 1, ry2 - 1), (255, 255, 0), 2)

        # FPS / 카운트
        n_frames += 1
        t_now = time.perf_counter()
        if t_now - t_fps >= 1.0:
            fps_val  = n_frames / (t_now - t_fps)
            n_frames = 0
            t_fps    = t_now
        hud = (f"FPS {fps_val:.1f}  known:{len(known_segs)}  unknown:{len(unknown_segs)}"
               f"  ROI({rx1},{ry1})-({rx2},{ry2})")
        if fullscreen:
            hud += "  [F]window  [ESC]quit"
        else:
            hud += "  [F]fullscreen  [ESC]quit"
        cv2.putText(vis, hud, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(WINDOW_NAME, vis)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key in (ord("f"), ord("F")):
            fullscreen = not fullscreen
            mode = cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, mode)
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
