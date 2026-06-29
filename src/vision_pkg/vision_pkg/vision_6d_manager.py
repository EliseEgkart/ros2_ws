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
        use_shape_ratio_filter=True,
        shape_ratio_threshold=1.5,
        edge_contact_max_px=10,
        edge_contact_margin_px=2,
        use_depth_median_filter=True,
        depth_median_margin_m=0.030,
        depth_median_min_samples=2,
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
        self.use_shape_ratio_filter = bool(use_shape_ratio_filter)
        self.shape_ratio_threshold = float(shape_ratio_threshold)
        self.edge_contact_max_px = int(edge_contact_max_px)
        self.edge_contact_margin_px = int(edge_contact_margin_px)
        self.use_depth_median_filter = bool(use_depth_median_filter)
        self.depth_median_margin_m = float(depth_median_margin_m)
        self.depth_median_min_samples = int(depth_median_min_samples)
        self.stop_requested = False
        self._pipeline_started = False

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
        self._pipeline_started = True
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
            f"visualize_scale={self.visualize_scale}, "
            f"shape_filter={self.use_shape_ratio_filter}, "
            f"shape_ratio_threshold={self.shape_ratio_threshold}, "
            f"edge_contact_max_px={self.edge_contact_max_px}, "
            f"edge_contact_margin_px={self.edge_contact_margin_px}, "
            f"depth_median_filter={self.use_depth_median_filter}, "
            f"depth_median_margin_m={self.depth_median_margin_m}, "
            f"depth_median_min_samples={self.depth_median_min_samples}"
        )

    def shutdown(self):
        """Stop camera and OpenCV GUI safely.

        This is intentionally tolerant because shutdown can be triggered by
        Ctrl+C, q/ESC in an OpenCV window, or ROS2 node destruction.
        """
        self.stop_requested = True

        if self._pipeline_started:
            try:
                self.pipeline.stop()
            except Exception as exc:
                self._log_warn(f"RealSense pipeline stop failed: {exc}")
            finally:
                self._pipeline_started = False

        try:
            cv2.destroyAllWindows()
            # Let the HighGUI event queue flush. This reduces Qt/GTK window
            # errors when the node is stopped from a terminal.
            for _ in range(5):
                cv2.waitKey(1)
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

        while (time.time() - start_time < self.sample_sec) and not self.stop_requested:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                aligned = self.align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not color_frame or not depth_frame:
                    continue

                image = np.asanyarray(color_frame.get_data())
                det_result, seg_result = self.run_yolo_pair(model_det, model_seg, image)
                if det_result.boxes is None:
                    continue

                all_z_values = []
                frame_targets = []
                detections_for_vis = []
                pre_candidates = []

                # ------------------------------------------------------------
                # 1차 필터:
                # - segmentation edge contact 길이 검사
                # - 2x2 / 4x2 mask 형상비 검사
                # - YOLO bbox 중심 depth 획득
                # 여기서 통과한 객체들의 중심 depth median을 기준 depth로 사용한다.
                # ------------------------------------------------------------
                for det_idx, box in enumerate(det_result.boxes):
                    cls_name = det_result.names[int(box.cls[0])]
                    cls_key = self._normalize_class_name(cls_name)

                    xyxy = box.xyxy[0].cpu().numpy()
                    u = int((xyxy[0] + xyxy[2]) / 2)
                    v = int((xyxy[1] + xyxy[3]) / 2)

                    is_target = self._target_matches(target_key, cls_key)
                    mask_pts = self.get_matching_mask_points(
                        det_result=det_result,
                        seg_result=seg_result,
                        det_idx=det_idx,
                        target_u=u,
                        target_v=v,
                    )

                    edge_contact_px = self.get_mask_border_contact_px(
                        mask_pts=mask_pts,
                        image_shape=image.shape,
                        margin_px=self.edge_contact_margin_px,
                    )
                    if edge_contact_px > self.edge_contact_max_px:
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": 0.0,
                                "yaw": 0.0,
                                "ratio": None,
                                "class_name": f"{cls_name}_edge{edge_contact_px}px",
                                "is_target": False,
                            }
                        )
                        continue

                    ratio = self.calculate_mask_aspect_ratio(mask_pts)
                    if not self.brick_shape_ratio_pass(cls_key, ratio):
                        ratio_text = "unknown" if ratio is None else f"{ratio:.2f}"
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": 0.0,
                                "yaw": 0.0,
                                "ratio": ratio,
                                "class_name": f"{cls_name}_shape_r{ratio_text}",
                                "is_target": False,
                            }
                        )
                        continue

                    z = self.get_valid_depth(depth_frame, u, v)
                    if z <= 0.0:
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": z,
                                "yaw": 0.0,
                                "ratio": ratio,
                                "class_name": str(cls_name),
                                "is_target": is_target,
                            }
                        )
                        continue

                    pre_candidates.append(
                        {
                            "u": u,
                            "v": v,
                            "z": float(z),
                            "ratio": ratio,
                            "mask_pts": mask_pts,
                            "class_name": str(cls_name),
                            "cls_key": cls_key,
                            "is_target": is_target,
                        }
                    )

                depth_ref_m = self.get_depth_median_reference(
                    [item["z"] for item in pre_candidates]
                )

                # ------------------------------------------------------------
                # 2차 필터:
                # 같은 프레임에서 살아남은 YOLO 후보들의 중심 depth median과
                # 크게 다른 객체는 바닥 아래/뒤쪽을 본 것으로 간주하고 제외한다.
                # RANSAC 없이 CPU에서 가볍게 동작하는 상대 depth 필터이다.
                # ------------------------------------------------------------
                for item in pre_candidates:
                    z = item["z"]
                    ratio = item["ratio"]
                    cls_name = item["class_name"]
                    u = item["u"]
                    v = item["v"]
                    is_target = item["is_target"]

                    if not self.depth_median_filter_pass(z, depth_ref_m):
                        diff_mm = 0.0 if depth_ref_m is None else abs(z - depth_ref_m) * 1000.0
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": z,
                                "yaw": 0.0,
                                "ratio": ratio,
                                "class_name": f"{cls_name}_depth{diff_mm:.0f}mm",
                                "is_target": False,
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
                                "yaw": 0.0,
                                "ratio": ratio,
                                "class_name": cls_name,
                                "is_target": False,
                            }
                        )
                        continue

                    yaw = self.find_yaw_from_mask_points(item["mask_pts"])
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "ratio": ratio,
                            "class_name": cls_name,
                            "is_target": True,
                        }
                    )
                    frame_targets.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "detected_class": cls_name,
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

        if self.stop_requested:
            return False

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
        det_result, seg_result = self.run_yolo_pair(model_det, model_seg, image)

        detections_for_vis = []
        best = None
        best_z = float("inf")
        pre_candidates = []

        if det_result.boxes is not None:
            for det_idx, box in enumerate(det_result.boxes):
                cls_name = det_result.names[int(box.cls[0])]
                cls_key = self._normalize_class_name(cls_name)

                xyxy = box.xyxy[0].cpu().numpy()
                u = int((xyxy[0] + xyxy[2]) / 2)
                v = int((xyxy[1] + xyxy[3]) / 2)

                is_target = True
                if target_key is not None:
                    is_target = self._target_matches(target_key, cls_key)

                mask_pts = self.get_matching_mask_points(
                    det_result=det_result,
                    seg_result=seg_result,
                    det_idx=det_idx,
                    target_u=u,
                    target_v=v,
                )

                edge_contact_px = self.get_mask_border_contact_px(
                    mask_pts=mask_pts,
                    image_shape=image.shape,
                    margin_px=self.edge_contact_margin_px,
                )
                if edge_contact_px > self.edge_contact_max_px:
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": 0.0,
                            "yaw": 0.0,
                            "ratio": None,
                            "class_name": f"{cls_name}_edge{edge_contact_px}px",
                            "is_target": False,
                        }
                    )
                    continue

                ratio = self.calculate_mask_aspect_ratio(mask_pts)
                if not self.brick_shape_ratio_pass(cls_key, ratio):
                    ratio_text = "unknown" if ratio is None else f"{ratio:.2f}"
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": 0.0,
                            "yaw": 0.0,
                            "ratio": ratio,
                            "class_name": f"{cls_name}_shape_r{ratio_text}",
                            "is_target": False,
                        }
                    )
                    continue

                z = self.get_valid_depth(depth_frame, u, v)
                if z <= 0.0:
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": 0.0,
                            "ratio": ratio,
                            "class_name": str(cls_name),
                            "is_target": is_target,
                        }
                    )
                    continue

                pre_candidates.append(
                    {
                        "u": u,
                        "v": v,
                        "z": float(z),
                        "ratio": ratio,
                        "mask_pts": mask_pts,
                        "class_name": str(cls_name),
                        "is_target": is_target,
                    }
                )

            depth_ref_m = self.get_depth_median_reference(
                [item["z"] for item in pre_candidates]
            )

            for item in pre_candidates:
                z = item["z"]
                ratio = item["ratio"]
                cls_name = item["class_name"]
                u = item["u"]
                v = item["v"]
                is_target = item["is_target"]

                if not self.depth_median_filter_pass(z, depth_ref_m):
                    diff_mm = 0.0 if depth_ref_m is None else abs(z - depth_ref_m) * 1000.0
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": 0.0,
                            "ratio": ratio,
                            "class_name": f"{cls_name}_depth{diff_mm:.0f}mm",
                            "is_target": False,
                        }
                    )
                    continue

                yaw = self.find_yaw_from_mask_points(item["mask_pts"])
                detections_for_vis.append(
                    {
                        "u": u,
                        "v": v,
                        "z": z,
                        "yaw": yaw,
                        "ratio": ratio,
                        "class_name": cls_name,
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
                        "detected_class": cls_name,
                    }

        label = target_class if target_class else f"all ({pipeline_name})"
        self.show_visualization(
            det_result=det_result,
            detections=detections_for_vis,
            target_class=label,
            best=best,
        )
        return True

    def get_depth_median_reference(self, z_values):
        """Return median center depth from current YOLO candidates.

        The filter is intentionally light-weight: it does not build a plane or a
        depth mask. It only compares valid center depths of detections that have
        already passed edge and shape-ratio filters.
        """
        if not self.use_depth_median_filter:
            return None

        valid = [float(z) for z in z_values if z is not None and np.isfinite(z) and z > 0.0]
        if len(valid) < self.depth_median_min_samples:
            return None

        return float(np.median(np.asarray(valid, dtype=np.float32)))

    def depth_median_filter_pass(self, z, depth_ref_m):
        """Check whether one detection center depth is close to frame median."""
        if not self.use_depth_median_filter:
            return True
        if depth_ref_m is None:
            return True
        if z is None or not np.isfinite(z) or z <= 0.0:
            return False

        return abs(float(z) - float(depth_ref_m)) <= self.depth_median_margin_m

    @staticmethod
    def run_yolo_pair(model_det, model_seg, image):
        """Run detection and segmentation models.

        If both references point to the same YOLO object, inference is executed
        only once. This avoids duplicated best_comp.pt inference in component
        mode while preserving the original det/seg manager structure.
        """
        det_result = model_det(image, verbose=False)[0]
        if model_seg is model_det:
            seg_result = det_result
        else:
            seg_result = model_seg(image, verbose=False)[0]
        return det_result, seg_result

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

            if det.get("ratio") is not None:
                label += f" R:{det['ratio']:.2f}"

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

        try:
            cv2.imshow(self.visualize_window, image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                self.stop_requested = True
                self._log_info("OpenCV q/ESC pressed. Stop requested.")
        except Exception as exc:
            self.stop_requested = True
            self._log_warn(f"OpenCV visualization failed: {exc}")

    def find_yaw_from_segmentation(self, seg_result, target_u, target_v):
        """Backward-compatible yaw helper using only the segmentation result."""
        mask_pts = self.get_nearest_mask_points(
            result=seg_result,
            target_u=target_u,
            target_v=target_v,
            match_distance_px=self.match_distance_px,
        )
        return self.find_yaw_from_mask_points(mask_pts)

    @staticmethod
    def find_yaw_from_mask_points(mask_pts):
        if mask_pts is None or len(mask_pts) < 3:
            return 0.0

        mask_pts = np.asarray(mask_pts, dtype=np.int32)
        moments = cv2.moments(mask_pts)
        if moments["m00"] == 0:
            return 0.0

        rect = cv2.minAreaRect(mask_pts)
        return Vision6DPoseManager.calculate_refined_yaw(rect)

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

    def get_matching_mask_points(
        self,
        det_result,
        seg_result,
        det_idx,
        target_u,
        target_v,
    ):
        """Return the mask polygon corresponding to a detection.

        Priority:
        1. det_result mask at the same detection index, if best.pt is a seg model
        2. nearest mask from seg_result, normally best_old.pt
        """
        if (
            det_result is not None
            and det_result.masks is not None
            and hasattr(det_result.masks, "xy")
            and len(det_result.masks.xy) > det_idx
        ):
            pts = np.asarray(det_result.masks.xy[det_idx], dtype=np.float32)
            if len(pts) >= 3:
                return pts

        return self.get_nearest_mask_points(
            result=seg_result,
            target_u=target_u,
            target_v=target_v,
            match_distance_px=self.match_distance_px,
        )

    @staticmethod
    def get_nearest_mask_points(result, target_u, target_v, match_distance_px=40.0):
        if result is None or result.masks is None or result.boxes is None:
            return None

        min_dist = float("inf")
        best_mask_pts = None

        for idx, seg_box in enumerate(result.boxes):
            xyxy = seg_box.xyxy[0].cpu().numpy()
            seg_u = int((xyxy[0] + xyxy[2]) / 2)
            seg_v = int((xyxy[1] + xyxy[3]) / 2)
            dist = ((target_u - seg_u) ** 2 + (target_v - seg_v) ** 2) ** 0.5

            if dist < match_distance_px and dist < min_dist:
                min_dist = dist
                if len(result.masks.xy) > idx:
                    pts = np.asarray(result.masks.xy[idx], dtype=np.float32)
                    if len(pts) >= 3:
                        best_mask_pts = pts

        return best_mask_pts

    def brick_shape_ratio_pass(self, cls_key, ratio):
        """Check 2x2/4x2 brick shape by segmentation minAreaRect ratio.

        - 2x2 classes pass when long_side / short_side <= threshold.
        - 4x2 classes pass when long_side / short_side >= threshold.
        - Non-brick classes or objects without mask ratio pass unchanged.
        """
        if not self.use_shape_ratio_filter:
            return True
        if ratio is None:
            return True

        if cls_key.startswith("2x2"):
            return ratio <= self.shape_ratio_threshold
        if cls_key.startswith("4x2"):
            return ratio >= self.shape_ratio_threshold
        return True

    @staticmethod
    def calculate_mask_aspect_ratio(mask_pts):
        if mask_pts is None or len(mask_pts) < 3:
            return None

        pts = np.asarray(mask_pts, dtype=np.float32)
        rect = cv2.minAreaRect(pts)
        (_, _), (width, height), _ = rect
        width = float(width)
        height = float(height)

        if width <= 1e-6 or height <= 1e-6:
            return None

        return max(width, height) / min(width, height)

    @staticmethod
    def get_mask_border_contact_px(mask_pts, image_shape, margin_px=2):
        """Return maximum contact length between a mask and image border in pixels.

        This replaces the older behavior where any bbox/mask border touch disabled
        the object. A small corner touch is allowed. Only a long contact span over
        edge_contact_max_px is filtered by the caller.
        """
        if mask_pts is None or len(mask_pts) < 3:
            return 0

        h, w = image_shape[:2]
        margin_px = max(0, int(margin_px))

        pts = np.asarray(mask_pts, dtype=np.int32)
        pts[:, 0] = np.clip(pts[:, 0], 0, w - 1)
        pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], 1)

        max_contact = 0

        # Left / right: measure vertical span of contact rows.
        left_cols = mask[:, : margin_px + 1]
        right_cols = mask[:, max(0, w - 1 - margin_px) : w]
        for strip in (left_cols, right_cols):
            ys = np.where(np.any(strip > 0, axis=1))[0]
            if ys.size > 0:
                max_contact = max(max_contact, int(ys.max() - ys.min() + 1))

        # Top / bottom: measure horizontal span of contact columns.
        top_rows = mask[: margin_px + 1, :]
        bottom_rows = mask[max(0, h - 1 - margin_px) : h, :]
        for strip in (top_rows, bottom_rows):
            xs = np.where(np.any(strip > 0, axis=0))[0]
            if xs.size > 0:
                max_contact = max(max_contact, int(xs.max() - xs.min() + 1))

        return int(max_contact)

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