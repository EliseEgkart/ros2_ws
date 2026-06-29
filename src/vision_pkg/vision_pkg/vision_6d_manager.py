import os
import time
from dataclasses import dataclass

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


WORKSPACE_DIR = os.environ.get("ROS2_WS", "/home/st02/ros2_ws")
DET_MODEL_PATH = os.path.join(WORKSPACE_DIR, "best.pt")
SEG_MODEL_PATH = os.path.join(WORKSPACE_DIR, "best_old.pt")
COMP_MODEL_PATH = os.path.join(WORKSPACE_DIR, "best_comp.pt")

BRICK_IDS = {1, 2, 3, 4, 5, 6, 7, 8}
COMPONENT_IDS = {13, 34, 81, 241, 442, 462, 711, 4482, 8518, 46262, 48132}


ID_TO_CLASS = {
    1: "2x2_red",
    2: "2x2_green",
    3: "2x2_blue",
    4: "2x2_yellow",
    5: "4x2_red",
    6: "4x2_green",
    7: "4x2_blue",
    8: "4x2_yellow",
    999: "assembly",
    888: "assembly_fine",
    13: "Magnet",
    34: "Battery",
    81: "Estop",
    241: "Trafficlight",
    442: "carrot",
    462: "small tree",
    711: "hammer",
    4482: "bigcarrot",
    8518: "burger",
    46262: "bigtree",
    48132: "icecream",
}


@dataclass
class PoseResult:
    success: bool
    target_id: int | None = None
    class_name: str | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    yaw_deg: float | None = None
    layer: int | None = None
    reason: str | None = None


class Vision6DPoseManager:
    """Ensemble detector based on vision_6Dpose_node.py.

    best.pt is used for stable detection coordinates. best_old.pt is used for
    segmentation masks and yaw extraction. The public output is kept compatible
    with arm_interfaces/srv/GetTargetPose.
    """

    def __init__(
        self,
        logger=None,
        det_model_path=DET_MODEL_PATH,
        seg_model_path=SEG_MODEL_PATH,
        comp_model_path=COMP_MODEL_PATH,
        sample_sec=1.2,
        min_samples=5,
        match_distance_px=40.0,
        visualize=False,
        visualize_window="6D Pose (Ensemble Mode)",
        visualize_scale=1.0,
    ):
        self.logger = logger
        self.det_model_path = det_model_path
        self.seg_model_path = seg_model_path
        self.comp_model_path = comp_model_path
        self.sample_sec = float(sample_sec)
        self.min_samples = int(min_samples)
        self.match_distance_px = float(match_distance_px)
        self.visualize = bool(visualize)
        self.visualize_window = str(visualize_window)
        self.visualize_scale = max(0.1, float(visualize_scale))

        self._check_model_file(self.det_model_path)
        self._check_model_file(self.seg_model_path)
        self._check_model_file(self.comp_model_path)

        self.model_det = YOLO(self.det_model_path)
        self.model_seg = YOLO(self.seg_model_path)
        self.model_comp = YOLO(self.comp_model_path)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )

        self._log_info(
            f"6D ensemble loaded: det={self.det_model_path}, seg={self.seg_model_path}, "
            f"comp={self.comp_model_path}, "
            f"det_task={self.model_det.task}, seg_task={self.model_seg.task}, "
            f"comp_task={self.model_comp.task}, "
            f"visualize={self.visualize}, "
            f"visualize_scale={self.visualize_scale}"
        )

    def shutdown(self):
        try:
            self.pipeline.stop()
        except Exception as exc:
            self._log_warn(f"RealSense pipeline stop failed: {exc}")
        if self.visualize:
            try:
                cv2.destroyWindow(self.visualize_window)
            except Exception:
                pass

    def run_pipeline_by_id(self, target_id):
        try:
            target_id = int(target_id)
        except Exception:
            return PoseResult(False, reason=f"invalid target id: {target_id}")

        class_name = ID_TO_CLASS.get(target_id)
        if class_name is None:
            return PoseResult(
                False,
                target_id=target_id,
                reason=f"unknown target id: {target_id}",
            )

        return self.run_pipeline_by_class(target_id, class_name)

    def run_pipeline_by_class(self, target_id, class_name):
        model_det, model_seg, pipeline_name = self._select_models(target_id)
        target_key = self._normalize_class_name(class_name)
        samples = []
        start_time = time.time()

        self._log_info(
            f"6D ensemble search start: id={target_id}, class={class_name}, "
            f"pipeline={pipeline_name}"
        )

        while time.time() - start_time < self.sample_sec:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                aligned = self.align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not color_frame or not depth_frame:
                    continue

                image = np.asanyarray(color_frame.get_data())
                det_result = model_det(image, verbose=False)[0]
                seg_result = model_seg(image, verbose=False)[0]
                if det_result.boxes is None:
                    continue

                all_z_values = []
                frame_targets = []
                detections_for_vis = []

                for box in det_result.boxes:
                    cls_name = det_result.names[int(box.cls[0])]
                    cls_key = self._normalize_class_name(cls_name)

                    xyxy = box.xyxy[0].cpu().numpy()
                    u = int((xyxy[0] + xyxy[2]) / 2)
                    v = int((xyxy[1] + xyxy[3]) / 2)

                    is_target = self._target_matches(target_key, cls_key)

                    # ------------------------------------------------------------
                    # [NEW] 화면 테두리에 걸친 객체 제거
                    # ------------------------------------------------------------
                    if self.is_border_cut_object(
                        xyxy=xyxy,
                        image_shape=image.shape,
                        seg_result=seg_result,
                        target_u=u,
                        target_v=v,
                        match_distance_px=self.match_distance_px,
                        margin_px=12,
                    ):
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": 0.0,
                                "yaw": 0.0,
                                "class_name": f"{cls_name}_edge_cut",
                                "is_target": False,
                            }
                        )
                        continue

                    z = self.get_valid_depth(depth_frame, u, v)
                    yaw = 0.0
                    is_target = self._target_matches(target_key, cls_key)
                    if z <= 0.0:
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": z,
                                "yaw": yaw,
                                "class_name": str(cls_name),
                                "is_target": is_target,
                            }
                        )
                        continue

                    all_z_values.append(z)
                    if not is_target:
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": z,
                                "yaw": yaw,
                                "class_name": str(cls_name),
                                "is_target": False,
                            }
                        )
                        continue

                    yaw = self.find_yaw_from_segmentation(seg_result, u, v)
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "class_name": str(cls_name),
                            "is_target": True,
                        }
                    )
                    frame_targets.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "detected_class": str(cls_name),
                        }
                    )

                current_best = None
                if frame_targets and all_z_values:
                    floor_z = max(all_z_values)
                    best = min(frame_targets, key=lambda item: item["z"])
                    current_best = best
                    layer = int(round((floor_z - best["z"]) / 0.016)) + 1
                    x_m, y_m, z_m = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics,
                        [best["u"], best["v"]],
                        best["z"],
                    )
                    samples.append(
                        [
                            float(x_m),
                            float(y_m),
                            float(z_m),
                            float(best["yaw"]),
                            float(layer),
                            best["detected_class"],
                        ]
                    )

                if self.visualize:
                    self.show_visualization(
                        det_result=det_result,
                        detections=detections_for_vis,
                        target_class=class_name,
                        best=current_best,
                    )

                time.sleep(0.01)
            except Exception as exc:
                self._log_warn(f"6D ensemble frame skipped: {exc}")

        if len(samples) < self.min_samples:
            return PoseResult(
                False,
                target_id=target_id,
                class_name=class_name,
                reason=f"not enough samples: {len(samples)}/{self.min_samples}",
            )

        numeric_samples = np.array([sample[:5] for sample in samples], dtype=float)
        median_pose = np.median(numeric_samples, axis=0)
        detected_class = self._majority_class([sample[5] for sample in samples])

        result = PoseResult(
            True,
            target_id=target_id,
            class_name=detected_class or class_name,
            x_m=float(median_pose[0]),
            y_m=float(median_pose[1]),
            z_m=float(median_pose[2]),
            yaw_deg=float(median_pose[3]),
            layer=int(round(float(median_pose[4]))),
        )

        self._log_info(
            "6D ensemble target fixed: "
            f"id={target_id}, class={result.class_name}, "
            f"x={result.x_m * 1000.0:.1f}mm, "
            f"y={result.y_m * 1000.0:.1f}mm, "
            f"z={result.z_m * 1000.0:.1f}mm, "
            f"yaw={result.yaw_deg:.1f}deg, layer={result.layer}"
        )
        return result

    def show_live_frame(self, target_id=0):
        """Show one annotated RealSense frame without waiting for a service call."""
        try:
            target_id = int(target_id)
        except Exception:
            target_id = 0

        target_class = ID_TO_CLASS.get(target_id)
        target_key = self._normalize_class_name(target_class) if target_class else None
        model_det, model_seg, pipeline_name = self._select_models(target_id)

        frames = self.pipeline.wait_for_frames(timeout_ms=500)
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not color_frame or not depth_frame:
            return False

        image = np.asanyarray(color_frame.get_data())
        det_result = model_det(image, verbose=False)[0]
        seg_result = model_seg(image, verbose=False)[0]

        detections_for_vis = []
        best = None
        best_z = float("inf")

        if det_result.boxes is not None:
            for box in det_result.boxes:
                cls_name = det_result.names[int(box.cls[0])]
                cls_key = self._normalize_class_name(cls_name)

                xyxy = box.xyxy[0].cpu().numpy()
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)

                is_target = True
                if target_key is not None:
                    is_target = self._target_matches(target_key, cls_key)

                if self.is_border_cut_object(
                    xyxy=xyxy,
                    image_shape=image.shape,
                    seg_result=seg_result,
                    target_u=u,
                    target_v=v,
                    match_distance_px=self.match_distance_px,
                    margin_px=12,
                ):
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": 0.0,
                            "yaw": 0.0,
                            "class_name": f"{cls_name}_edge_cut",
                            "is_target": False,
                        }
                    )
                    continue

                z = self.get_valid_depth(depth_frame, u, v)
                yaw = self.find_yaw_from_segmentation(seg_result, u, v) if z > 0.0 else 0.0
                detections_for_vis.append(
                    {
                        "u": u,
                        "v": v,
                        "z": z,
                        "yaw": yaw,
                        "class_name": str(cls_name),
                        "is_target": is_target,
                    }
                )

                if is_target and 0.0 < z < best_z:
                    best_z = z
                    best = {
                        "u": u,
                        "v": v,
                        "z": z,
                        "yaw": yaw,
                        "detected_class": str(cls_name),
                    }

        label = target_class if target_class else f"all ({pipeline_name})"
        self.show_visualization(
            det_result=det_result,
            detections=detections_for_vis,
            target_class=label,
            best=best,
        )
        return True

    def _select_models(self, target_id):
        if target_id in COMPONENT_IDS:
            return self.model_comp, self.model_comp, "component"
        if target_id in BRICK_IDS:
            return self.model_det, self.model_seg, "brick"
        return self.model_det, self.model_seg, "default"

    def show_visualization(self, det_result, detections, target_class, best=None):
        image = det_result.plot()
        height, width = image.shape[:2]
        cv2.circle(image, (width // 2, height // 2), 5, (0, 0, 255), -1)
        cv2.putText(
            image,
            f"target: {target_class}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        best_u = best["u"] if best is not None else None
        best_v = best["v"] if best is not None else None

        for det in detections:
            u = det["u"]
            v = det["v"]
            color = (0, 255, 255) if det["is_target"] else (180, 180, 180)
            radius = 7 if det["is_target"] else 4
            if best_u == u and best_v == v:
                color = (0, 0, 255)
                radius = 9

            cv2.circle(image, (u, v), radius, color, -1)
            if det["z"] > 0.0:
                label = (
                    f"{det['class_name']} "
                    f"Z:{det['z'] * 1000.0:.0f} "
                    f"Yaw:{det['yaw']:.1f}"
                )
            else:
                label = f"{det['class_name']} Z:invalid"

            cv2.putText(
                image,
                label,
                (max(0, u - 90), min(height - 10, v + 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        if self.visualize_scale != 1.0:
            image = cv2.resize(
                image,
                None,
                fx=self.visualize_scale,
                fy=self.visualize_scale,
                interpolation=cv2.INTER_LINEAR,
            )

        cv2.imshow(self.visualize_window, image)
        cv2.waitKey(1)

    def find_yaw_from_segmentation(self, seg_result, target_u, target_v):
        if seg_result.masks is None or seg_result.boxes is None:
            return 0.0

        min_dist = float("inf")
        best_mask_pts = None

        for idx, seg_box in enumerate(seg_result.boxes):
            xyxy = seg_box.xyxy[0].cpu().numpy()
            seg_u = int((xyxy[0] + xyxy[2]) / 2)
            seg_v = int((xyxy[1] + xyxy[3]) / 2)
            dist = ((target_u - seg_u) ** 2 + (target_v - seg_v) ** 2) ** 0.5

            if dist < self.match_distance_px and dist < min_dist:
                min_dist = dist
                if len(seg_result.masks.xy) > idx:
                    best_mask_pts = np.int32(seg_result.masks.xy[idx])

        if best_mask_pts is None or len(best_mask_pts) < 3:
            return 0.0

        moments = cv2.moments(best_mask_pts)
        if moments["m00"] == 0:
            return 0.0

        rect = cv2.minAreaRect(best_mask_pts)
        return self.calculate_refined_yaw(rect)

    @staticmethod
    def calculate_refined_yaw(rect):
        (_, _), (width, height), angle = rect
        if width < height:
            yaw = angle
        else:
            yaw = angle + 90.0

        if yaw > 90.0:
            yaw -= 180.0
        if yaw < -90.0:
            yaw += 180.0
        return float(yaw)

    @staticmethod
    def is_border_cut_object(
        xyxy,
        image_shape,
        seg_result=None,
        target_u=None,
        target_v=None,
        match_distance_px=40.0,
        margin_px=12,
    ):
        """
        화면 테두리에 걸쳐 잘린 객체인지 판단.

        판단 기준:
        1. YOLO detection bbox가 이미지 테두리에 margin_px 이내로 닿으면 True
        2. segmentation mask polygon이 이미지 테두리에 margin_px 이내로 닿으면 True

        Args:
            xyxy: YOLO bbox 좌표 [x1, y1, x2, y2]
            image_shape: image.shape, 보통 (H, W, C)
            seg_result: YOLO segmentation 결과. 없으면 bbox 기준만 사용
            target_u, target_v: detection bbox 중심 픽셀
            match_distance_px: detection bbox와 segmentation bbox 매칭 거리
            margin_px: 테두리로 판단할 픽셀 여유값

        Returns:
            True  -> 화면 테두리에 걸친 잘린 객체, 비활성화 권장
            False -> 정상 객체
        """
        h, w = image_shape[:2]

        x1, y1, x2, y2 = map(float, xyxy)

        # ------------------------------------------------------------
        # 1) Detection bbox가 화면 테두리에 닿는지 확인
        # ------------------------------------------------------------
        bbox_touches_border = (
            x1 <= margin_px or
            y1 <= margin_px or
            x2 >= (w - 1 - margin_px) or
            y2 >= (h - 1 - margin_px)
        )

        if bbox_touches_border:
            return True

        # ------------------------------------------------------------
        # 2) Segmentation mask가 있으면 mask polygon도 확인
        #    bbox는 안 닿았는데 mask만 경계에 걸치는 경우 방어
        # ------------------------------------------------------------
        if seg_result is None:
            return False

        if seg_result.masks is None or seg_result.boxes is None:
            return False

        if target_u is None or target_v is None:
            return False

        min_dist = float("inf")
        best_mask_pts = None

        for idx, seg_box in enumerate(seg_result.boxes):
            seg_xyxy = seg_box.xyxy[0].cpu().numpy()
            seg_u = int((seg_xyxy[0] + seg_xyxy[2]) / 2)
            seg_v = int((seg_xyxy[1] + seg_xyxy[3]) / 2)

            dist = ((target_u - seg_u) ** 2 + (target_v - seg_v) ** 2) ** 0.5

            if dist < match_distance_px and dist < min_dist:
                min_dist = dist
                if len(seg_result.masks.xy) > idx:
                    best_mask_pts = np.asarray(seg_result.masks.xy[idx], dtype=np.float32)

        if best_mask_pts is None or len(best_mask_pts) < 3:
            return False

        xs = best_mask_pts[:, 0]
        ys = best_mask_pts[:, 1]

        mask_touches_border = (
            np.any(xs <= margin_px) or
            np.any(ys <= margin_px) or
            np.any(xs >= (w - 1 - margin_px)) or
            np.any(ys >= (h - 1 - margin_px))
        )

        return bool(mask_touches_border)

    @staticmethod
    def get_valid_depth(depth_frame, u, v, search_radius=10):
        z = depth_frame.get_distance(u, v)
        if z > 0.0:
            return float(z)

        for radius in range(1, search_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    nu = u + dx
                    nv = v + dy
                    if 0 <= nu < 640 and 0 <= nv < 480:
                        z = depth_frame.get_distance(nu, nv)
                        if z > 0.0:
                            return float(z)
        return 0.0

    @staticmethod
    def _normalize_class_name(name):
        return str(name).lower().replace(" ", "").replace("-", "_")

    @staticmethod
    def _target_matches(target_key, detected_key):
        if target_key == detected_key:
            return True
        return target_key in detected_key or detected_key in target_key

    @staticmethod
    def _majority_class(names):
        counts = {}
        for name in names:
            counts[name] = counts.get(name, 0) + 1
        if not counts:
            return None
        return max(counts.items(), key=lambda item: item[1])[0]

    @staticmethod
    def _check_model_file(path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"YOLO model not found: {path}")

    def _log_info(self, message):
        if self.logger is not None:
            self.logger.info(message)

    def _log_warn(self, message):
        if self.logger is not None:
            self.logger.warn(message)