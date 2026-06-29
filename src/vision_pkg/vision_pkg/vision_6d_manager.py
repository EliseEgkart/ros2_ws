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
        use_depth_volume_filter=True,
        visualize_depth_mask=False,
        depth_min_height_m=0.015,
        depth_max_height_m=0.120,
        depth_min_size_x_m=0.025,
        depth_min_size_y_m=0.025,
        depth_min_area_px=80,
        depth_min_overlap_ratio=0.06,
        depth_border_margin_px=8,
        depth_max_border_contact_m=0.008,
        use_depth_mask_as_yolo_input=False,
        use_shape_ratio_filter=True,
        shape_ratio_2x2_max=1.45,
        shape_ratio_4x2_min=1.55,
        shape_ratio_min_mask_points=8,
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

        # ------------------------------------------------------------
        # Depth/RANSAC volume filter
        # ------------------------------------------------------------
        self.use_depth_volume_filter = bool(use_depth_volume_filter)
        self.visualize_depth_mask = bool(visualize_depth_mask)
        self.depth_min_height_m = float(depth_min_height_m)
        self.depth_max_height_m = float(depth_max_height_m)
        self.depth_min_size_x_m = float(depth_min_size_x_m)
        self.depth_min_size_y_m = float(depth_min_size_y_m)
        self.depth_min_area_px = int(depth_min_area_px)
        self.depth_min_overlap_ratio = float(depth_min_overlap_ratio)
        self.depth_border_margin_px = int(depth_border_margin_px)
        self.depth_max_border_contact_m = float(depth_max_border_contact_m)
        self.use_depth_mask_as_yolo_input = bool(use_depth_mask_as_yolo_input)

        # ------------------------------------------------------------
        # YOLO segmentation shape-ratio filter
        # ------------------------------------------------------------
        self.use_shape_ratio_filter = bool(use_shape_ratio_filter)
        self.shape_ratio_2x2_max = float(shape_ratio_2x2_max)
        self.shape_ratio_4x2_min = float(shape_ratio_4x2_min)
        self.shape_ratio_min_mask_points = int(shape_ratio_min_mask_points)

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
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(depth_sensor.get_depth_scale())

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
            f"depth_scale={self.depth_scale}, "
            f"use_depth_volume_filter={self.use_depth_volume_filter}, "
            f"depth_min_height_m={self.depth_min_height_m}, "
            f"depth_min_size=({self.depth_min_size_x_m}, {self.depth_min_size_y_m}), "
            f"depth_max_border_contact_m={self.depth_max_border_contact_m}, "
            f"use_shape_ratio_filter={self.use_shape_ratio_filter}, "
            f"shape_ratio_2x2_max={self.shape_ratio_2x2_max}, "
            f"shape_ratio_4x2_min={self.shape_ratio_4x2_min}"
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

                depth_object_mask = None
                depth_components = []
                rejected_depth_components = []
                if self.use_depth_volume_filter:
                    (
                        depth_object_mask,
                        depth_components,
                        rejected_depth_components,
                        _,
                    ) = self.build_depth_volume_mask(depth_frame, image.shape)

                yolo_image = image
                if (
                    self.use_depth_mask_as_yolo_input
                    and depth_object_mask is not None
                ):
                    # 학습 분포가 바뀔 수 있으므로 기본값은 False.
                    # 필요할 때만 depth mask를 YOLO 입력 이미지에 직접 적용.
                    yolo_image = image.copy()
                    yolo_image[~depth_object_mask] = (0, 0, 0)

                det_result = model_det(yolo_image, verbose=False)[0]
                seg_result = model_seg(yolo_image, verbose=False)[0]
                if det_result.boxes is None:
                    continue

                all_z_values = []
                frame_targets = []
                detections_for_vis = []

                for det_idx, box in enumerate(det_result.boxes):
                    cls_name = det_result.names[int(box.cls[0])]
                    cls_key = self._normalize_class_name(cls_name)

                    xyxy = box.xyxy[0].cpu().numpy()
                    u = int((xyxy[0] + xyxy[2]) / 2)
                    v = int((xyxy[1] + xyxy[3]) / 2)

                    is_target = self._target_matches(target_key, cls_key)

                    # ------------------------------------------------------------
                    # [NEW] RANSAC plane 기반 3D volume mask 검증
                    # - YOLO는 보이지만 실제로 바닥 위 물체 부피가 없으면 제거
                    # - depth component가 카메라 사이드에 길게 잘린 경우 제거
                    # ------------------------------------------------------------
                    if self.use_depth_volume_filter:
                        if depth_object_mask is None:
                            detections_for_vis.append(
                                {
                                    "u": u,
                                    "v": v,
                                    "z": 0.0,
                                    "yaw": 0.0,
                                    "class_name": f"{cls_name}_depth_filter_fail",
                                    "is_target": False,
                                }
                            )
                            continue

                        if not self.detection_overlaps_depth_mask(
                            xyxy=xyxy,
                            depth_object_mask=depth_object_mask,
                            min_overlap_ratio=self.depth_min_overlap_ratio,
                        ):
                            detections_for_vis.append(
                                {
                                    "u": u,
                                    "v": v,
                                    "z": 0.0,
                                    "yaw": 0.0,
                                    "class_name": f"{cls_name}_no_depth_volume",
                                    "is_target": False,
                                }
                            )
                            continue
                    else:
                        # depth volume filter를 끈 경우에만 기존의 단순 border cut 사용
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

                    mask_geom = self.get_mask_geometry_for_detection(
                        det_result=det_result,
                        det_index=det_idx,
                        fallback_seg_result=seg_result,
                        target_u=u,
                        target_v=v,
                    )

                    shape_ok, shape_reason = self.validate_brick_shape_ratio(
                        class_name=cls_name,
                        mask_geom=mask_geom,
                    )
                    if not shape_ok:
                        shape_ratio = None if mask_geom is None else mask_geom.get("ratio")
                        ratio_suffix = "" if shape_ratio is None else f"_r{shape_ratio:.2f}"
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": z,
                                "yaw": 0.0 if mask_geom is None else mask_geom.get("yaw", 0.0),
                                "class_name": f"{cls_name}_{shape_reason}{ratio_suffix}",
                                "is_target": False,
                                "shape_ratio": shape_ratio,
                            }
                        )
                        continue

                    yaw = self.find_yaw_from_mask_geometry(mask_geom)
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "class_name": str(cls_name),
                            "is_target": True,
                            "shape_ratio": None if mask_geom is None else mask_geom.get("ratio"),
                            "mask_source": None if mask_geom is None else mask_geom.get("source"),
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
                    self.show_depth_volume_debug(
                        image=image,
                        depth_object_mask=depth_object_mask,
                        depth_components=depth_components,
                        rejected_components=rejected_depth_components,
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

        depth_object_mask = None
        depth_components = []
        rejected_depth_components = []
        if self.use_depth_volume_filter:
            (
                depth_object_mask,
                depth_components,
                rejected_depth_components,
                _,
            ) = self.build_depth_volume_mask(depth_frame, image.shape)

        yolo_image = image
        if (
            self.use_depth_mask_as_yolo_input
            and depth_object_mask is not None
        ):
            yolo_image = image.copy()
            yolo_image[~depth_object_mask] = (0, 0, 0)

        det_result = model_det(yolo_image, verbose=False)[0]
        seg_result = model_seg(yolo_image, verbose=False)[0]

        detections_for_vis = []
        best = None
        best_z = float("inf")

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

                if self.use_depth_volume_filter:
                    if depth_object_mask is None:
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": 0.0,
                                "yaw": 0.0,
                                "class_name": f"{cls_name}_depth_filter_fail",
                                "is_target": False,
                            }
                        )
                        continue

                    if not self.detection_overlaps_depth_mask(
                        xyxy=xyxy,
                        depth_object_mask=depth_object_mask,
                        min_overlap_ratio=self.depth_min_overlap_ratio,
                    ):
                        detections_for_vis.append(
                            {
                                "u": u,
                                "v": v,
                                "z": 0.0,
                                "yaw": 0.0,
                                "class_name": f"{cls_name}_no_depth_volume",
                                "is_target": False,
                            }
                        )
                        continue
                else:
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
                mask_geom = None
                yaw = 0.0
                if z > 0.0:
                    mask_geom = self.get_mask_geometry_for_detection(
                        det_result=det_result,
                        det_index=det_idx,
                        fallback_seg_result=seg_result,
                        target_u=u,
                        target_v=v,
                    )
                    yaw = self.find_yaw_from_mask_geometry(mask_geom)

                shape_ok, shape_reason = self.validate_brick_shape_ratio(
                    class_name=cls_name,
                    mask_geom=mask_geom,
                )
                if not shape_ok:
                    shape_ratio = None if mask_geom is None else mask_geom.get("ratio")
                    ratio_suffix = "" if shape_ratio is None else f"_r{shape_ratio:.2f}"
                    detections_for_vis.append(
                        {
                            "u": u,
                            "v": v,
                            "z": z,
                            "yaw": yaw,
                            "class_name": f"{cls_name}_{shape_reason}{ratio_suffix}",
                            "is_target": False,
                            "shape_ratio": shape_ratio,
                        }
                    )
                    continue

                detections_for_vis.append(
                    {
                        "u": u,
                        "v": v,
                        "z": z,
                        "yaw": yaw,
                        "class_name": str(cls_name),
                        "is_target": is_target,
                        "shape_ratio": None if mask_geom is None else mask_geom.get("ratio"),
                        "mask_source": None if mask_geom is None else mask_geom.get("source"),
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
        self.show_depth_volume_debug(
            image=image,
            depth_object_mask=depth_object_mask,
            depth_components=depth_components,
            rejected_components=rejected_depth_components,
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
            ratio_text = ""
            if det.get("shape_ratio") is not None:
                ratio_text = f" R:{float(det['shape_ratio']):.2f}"

            source_text = ""
            if det.get("mask_source") is not None:
                source_text = f" {det['mask_source']}"

            if det["z"] > 0.0:
                label = (
                    f"{det['class_name']} "
                    f"Z:{det['z'] * 1000.0:.0f} "
                    f"Yaw:{det['yaw']:.1f}"
                    f"{ratio_text}{source_text}"
                )
            else:
                label = f"{det['class_name']} Z:invalid{ratio_text}{source_text}"

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

    def show_depth_volume_debug(
        self,
        image,
        depth_object_mask,
        depth_components=None,
        rejected_components=None,
    ):
        """Depth/RANSAC volume mask 디버그 창."""
        if not self.visualize_depth_mask:
            return
        if image is None or depth_object_mask is None:
            return

        vis = image.copy()
        vis[~depth_object_mask] = (0, 0, 0)

        for comp in depth_components or []:
            x1, y1, x2, y2 = comp["bbox_px"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
            label = (
                f"ok {comp['size_x_m']*1000:.0f}x"
                f"{comp['size_y_m']*1000:.0f} "
                f"h{comp['height_m']*1000:.0f} "
                f"edge{comp['border_contact_m']*1000:.0f}"
            )
            cv2.putText(
                vis,
                label,
                (x1, max(14, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        for comp in rejected_components or []:
            x1, y1, x2, y2 = comp["bbox_px"]
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 1)
            label = str(comp.get("reason", "reject"))
            cv2.putText(
                vis,
                label,
                (x1, min(vis.shape[0] - 8, y2 + 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )

        if self.visualize_scale != 1.0:
            vis = cv2.resize(
                vis,
                None,
                fx=self.visualize_scale,
                fy=self.visualize_scale,
                interpolation=cv2.INTER_LINEAR,
            )

        cv2.imshow("RANSAC Depth Volume Mask", vis)
        cv2.waitKey(1)

    def get_mask_geometry_for_detection(
        self,
        det_result,
        det_index,
        fallback_seg_result,
        target_u,
        target_v,
    ):
        """
        현재 detection에 대응되는 segmentation mask geometry를 가져온다.

        우선순위:
          1. det_result 자체의 mask. best.pt가 segmentation 모델이면 이 mask가 사용됨.
          2. fallback_seg_result의 가장 가까운 mask. 보통 best_old.pt mask가 사용됨.
        """
        geom = self.extract_mask_geometry_from_result(
            result=det_result,
            target_u=target_u,
            target_v=target_v,
            preferred_index=det_index,
            source="det",
        )
        if geom is not None:
            return geom

        return self.extract_mask_geometry_from_result(
            result=fallback_seg_result,
            target_u=target_u,
            target_v=target_v,
            preferred_index=None,
            source="seg_fallback",
        )

    def extract_mask_geometry_from_result(
        self,
        result,
        target_u,
        target_v,
        preferred_index=None,
        source="seg",
    ):
        """
        YOLO result에서 target 중심점에 대응되는 mask를 찾아 minAreaRect geometry를 계산.

        Returns:
            {
                "points": Nx2 float32,
                "rect": cv2.minAreaRect result,
                "yaw": float,
                "ratio": long_side / short_side,
                "long_side_px": float,
                "short_side_px": float,
                "source": str,
                "mask_index": int,
            }
            또는 None
        """
        if result is None:
            return None
        if result.masks is None or result.boxes is None:
            return None
        if result.masks.xy is None:
            return None

        mask_count = len(result.masks.xy)
        if mask_count <= 0:
            return None

        candidate_indices = []

        if preferred_index is not None:
            try:
                preferred_index = int(preferred_index)
            except Exception:
                preferred_index = None

        if preferred_index is not None and 0 <= preferred_index < mask_count:
            candidate_indices.append(preferred_index)
        else:
            min_dist = float("inf")
            best_idx = None
            for idx, seg_box in enumerate(result.boxes):
                if idx >= mask_count:
                    break
                xyxy = seg_box.xyxy[0].cpu().numpy()
                seg_u = int((xyxy[0] + xyxy[2]) / 2)
                seg_v = int((xyxy[1] + xyxy[3]) / 2)
                dist = ((target_u - seg_u) ** 2 + (target_v - seg_v) ** 2) ** 0.5
                if dist < self.match_distance_px and dist < min_dist:
                    min_dist = dist
                    best_idx = idx
            if best_idx is not None:
                candidate_indices.append(best_idx)

        for idx in candidate_indices:
            if idx < 0 or idx >= mask_count:
                continue

            pts = np.asarray(result.masks.xy[idx], dtype=np.float32)
            if pts is None or len(pts) < self.shape_ratio_min_mask_points:
                continue

            pts_i32 = np.int32(pts)
            rect = cv2.minAreaRect(pts_i32)
            (_, _), (width, height), _ = rect
            width = float(width)
            height = float(height)
            short_side = min(width, height)
            long_side = max(width, height)

            if short_side <= 1e-6 or long_side <= 1e-6:
                continue

            yaw = self.calculate_refined_yaw(rect)
            ratio = float(long_side / short_side)

            return {
                "points": pts,
                "rect": rect,
                "yaw": float(yaw),
                "ratio": ratio,
                "long_side_px": float(long_side),
                "short_side_px": float(short_side),
                "source": str(source),
                "mask_index": int(idx),
            }

        return None

    def validate_brick_shape_ratio(self, class_name, mask_geom):
        """
        2x2 / 4x2 브릭 class를 YOLO segmentation mask 형상비로 검증.

        - 2x2는 long/short가 1에 가까워야 함.
        - 4x2는 long/short가 2에 가까워야 함.
        - depth mask는 실제 물체 여부 검증용이고, 형상비는 YOLO mask 기준으로 판단함.
        """
        if not self.use_shape_ratio_filter:
            return True, "ok"

        class_key = self._normalize_class_name(class_name)
        is_2x2 = class_key.startswith("2x2")
        is_4x2 = class_key.startswith("4x2")

        if not is_2x2 and not is_4x2:
            return True, "ok"

        if mask_geom is None:
            return False, "no_shape_mask"

        ratio = float(mask_geom.get("ratio", 0.0))
        if ratio <= 0.0:
            return False, "bad_shape_ratio"

        if is_2x2 and ratio > self.shape_ratio_2x2_max:
            return False, "reject_2x2_shape"

        if is_4x2 and ratio < self.shape_ratio_4x2_min:
            return False, "reject_4x2_shape"

        return True, "ok"

    @staticmethod
    def find_yaw_from_mask_geometry(mask_geom):
        if mask_geom is None:
            return 0.0
        return float(mask_geom.get("yaw", 0.0))

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

    def build_depth_volume_mask(self, depth_frame, image_shape=None):
        """
        RANSAC으로 현재 선반/바닥 plane을 잡고,
        plane보다 카메라 쪽으로 튀어나온 3D 부피 객체만 남기는 mask 생성.

        조건:
          - plane 기준 높이 >= depth_min_height_m
          - 3D x/y 크기 >= depth_min_size_x_m / depth_min_size_y_m
          - 카메라 사이드 접촉 길이 <= depth_max_border_contact_m

        Returns:
            final_mask: bool, shape=(H, W)
            components: 통과한 component 정보
            rejected_components: 제거한 component 정보
            plane: [a,b,c,d] or None
        """
        depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        depth_m = depth_raw * self.depth_scale
        h, w = depth_m.shape[:2]

        valid = np.isfinite(depth_m) & (depth_m > 0.05) & (depth_m < 2.0)
        if np.count_nonzero(valid) < 500:
            return None, [], [], None

        xyz_map = self.depth_to_xyz_map(depth_m, self.intrinsics)
        points = xyz_map[valid]

        plane = self.fit_plane_ransac_numpy(
            points=points,
            num_iter=160,
            distance_threshold=0.006,
            max_points=9000,
            random_seed=0,
            min_abs_c=0.08,
        )
        if plane is None:
            return None, [], [], None

        a, b, c, d = plane
        if abs(c) < 1e-6:
            return None, [], [], plane

        x_map = xyz_map[:, :, 0]
        y_map = xyz_map[:, :, 1]
        z_plane = -(a * x_map + b * y_map + d) / c

        # 카메라 optical axis 기준 protrusion.
        # 바닥보다 카메라에 가까우면 depth_m이 작으므로 z_plane - depth_m이 양수.
        height_m = z_plane - depth_m

        protrusion_mask = (
            valid
            & np.isfinite(height_m)
            & (height_m >= self.depth_min_height_m)
            & (height_m <= self.depth_max_height_m)
        )

        mask_u8 = protrusion_mask.astype(np.uint8) * 255
        kernel3 = np.ones((3, 3), np.uint8)
        kernel5 = np.ones((5, 5), np.uint8)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel3, iterations=1)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel5, iterations=2)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask_u8,
            connectivity=8,
        )

        final_mask = np.zeros((h, w), dtype=bool)
        components = []
        rejected_components = []

        for label_id in range(1, num_labels):
            area_px = int(stats[label_id, cv2.CC_STAT_AREA])
            x0 = int(stats[label_id, cv2.CC_STAT_LEFT])
            y0 = int(stats[label_id, cv2.CC_STAT_TOP])
            bw = int(stats[label_id, cv2.CC_STAT_WIDTH])
            bh = int(stats[label_id, cv2.CC_STAT_HEIGHT])
            x1 = x0 + bw
            y1 = y0 + bh

            base_info = {
                "label_id": int(label_id),
                "area_px": area_px,
                "bbox_px": [x0, y0, x1, y1],
                "centroid_px": [
                    float(centroids[label_id][0]),
                    float(centroids[label_id][1]),
                ],
            }

            if area_px < self.depth_min_area_px:
                base_info["reason"] = "small_area"
                rejected_components.append(base_info)
                continue

            comp_mask = labels == label_id
            ys, xs = np.where(comp_mask)
            if len(xs) < self.depth_min_area_px:
                base_info["reason"] = "small_points"
                rejected_components.append(base_info)
                continue

            comp_xyz = xyz_map[ys, xs]
            comp_h = height_m[ys, xs]

            finite_xyz = np.isfinite(comp_xyz).all(axis=1)
            finite_h = np.isfinite(comp_h)
            keep = finite_xyz & finite_h
            comp_xyz = comp_xyz[keep]
            comp_h = comp_h[keep]
            xs_keep = xs[keep]
            ys_keep = ys[keep]

            if comp_xyz.shape[0] < self.depth_min_area_px:
                base_info["reason"] = "few_valid_depth"
                rejected_components.append(base_info)
                continue

            x_min, x_max = np.percentile(comp_xyz[:, 0], [5, 95])
            y_min, y_max = np.percentile(comp_xyz[:, 1], [5, 95])
            h95 = np.percentile(comp_h, 95)

            size_x_m = float(x_max - x_min)
            size_y_m = float(y_max - y_min)
            height_obj_m = float(h95)

            border_contact_m = self.estimate_border_contact_m(
                comp_mask=comp_mask,
                depth_m=depth_m,
                intrinsics=self.intrinsics,
                margin_px=self.depth_border_margin_px,
            )

            info = dict(base_info)
            info.update(
                {
                    "size_x_m": size_x_m,
                    "size_y_m": size_y_m,
                    "height_m": height_obj_m,
                    "border_contact_m": float(border_contact_m),
                }
            )

            # 2x2 브릭보다 살짝 작은 기준: 25mm x 25mm x 15mm
            if size_x_m < self.depth_min_size_x_m:
                info["reason"] = "small_x"
                rejected_components.append(info)
                continue

            if size_y_m < self.depth_min_size_y_m:
                info["reason"] = "small_y"
                rejected_components.append(info)
                continue

            if height_obj_m < self.depth_min_height_m:
                info["reason"] = "small_height"
                rejected_components.append(info)
                continue

            # 카메라 사이드에 닿은 선이 물리적으로 길면 반쯤 잘린 객체로 판단.
            # 작은 모서리 접촉은 border_contact_m이 작으므로 통과.
            if border_contact_m > self.depth_max_border_contact_m:
                info["reason"] = "large_edge_contact"
                rejected_components.append(info)
                continue

            final_mask[comp_mask] = True
            components.append(info)

        return final_mask, components, rejected_components, plane

    @staticmethod
    def depth_to_xyz_map(depth_m, intrinsics):
        """depth image[m] -> camera XYZ map[m]."""
        h, w = depth_m.shape[:2]
        u_grid, v_grid = np.meshgrid(
            np.arange(w, dtype=np.float32),
            np.arange(h, dtype=np.float32),
        )

        z = depth_m.astype(np.float32)
        x = (u_grid - intrinsics.ppx) / intrinsics.fx * z
        y = (v_grid - intrinsics.ppy) / intrinsics.fy * z

        return np.dstack((x, y, z)).astype(np.float32)

    @staticmethod
    def fit_plane_ransac_numpy(
        points,
        num_iter=160,
        distance_threshold=0.006,
        max_points=9000,
        random_seed=0,
        min_abs_c=0.08,
    ):
        """
        points: Nx3, meter
        plane: ax + by + cz + d = 0

        min_abs_c는 z = f(x,y) 형태로 plane을 쓸 수 있는지 확인하는 최소값.
        너무 작으면 optical axis 기준 height 계산이 불안정함.
        """
        if points is None or points.shape[0] < 3:
            return None

        rng = np.random.default_rng(random_seed)

        if points.shape[0] > max_points:
            idx = rng.choice(points.shape[0], size=max_points, replace=False)
            sample_points = points[idx]
        else:
            sample_points = points

        n_points = sample_points.shape[0]
        if n_points < 3:
            return None

        best_plane = None
        best_inlier_mask = None
        best_inlier_count = -1

        for _ in range(num_iter):
            ids = rng.choice(n_points, size=3, replace=False)
            p1, p2, p3 = sample_points[ids]

            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)

            if norm < 1e-9:
                continue

            normal = normal / norm
            if abs(float(normal[2])) < min_abs_c:
                continue

            d = -float(np.dot(normal, p1))

            distances = np.abs(sample_points @ normal + d)
            inlier_mask = distances < distance_threshold
            inlier_count = int(np.count_nonzero(inlier_mask))

            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_plane = np.array(
                    [normal[0], normal[1], normal[2], d],
                    dtype=np.float32,
                )
                best_inlier_mask = inlier_mask

        if best_plane is None or best_inlier_mask is None:
            return None

        inliers = sample_points[best_inlier_mask]
        if inliers.shape[0] < 3:
            return best_plane

        centroid = np.mean(inliers, axis=0)
        _, _, vh = np.linalg.svd(inliers - centroid)
        normal = vh[-1]
        normal = normal / (np.linalg.norm(normal) + 1e-9)
        if abs(float(normal[2])) < min_abs_c:
            return best_plane

        d = -float(np.dot(normal, centroid))
        return np.array([normal[0], normal[1], normal[2], d], dtype=np.float32)

    @staticmethod
    def estimate_border_contact_m(comp_mask, depth_m, intrinsics=None, margin_px=8):
        """
        component가 화면 경계와 닿은 길이를 meter로 환산.
        작은 모서리만 닿으면 길이가 작고, 반쯤 잘리면 길이가 커진다.
        """
        h, w = comp_mask.shape[:2]
        contacts = []

        side_specs = [
            ("left", comp_mask[:, :margin_px], "vertical", 0),
            ("right", comp_mask[:, max(0, w - margin_px):], "vertical", max(0, w - margin_px)),
            ("top", comp_mask[:margin_px, :], "horizontal", 0),
            ("bottom", comp_mask[max(0, h - margin_px):, :], "horizontal", max(0, h - margin_px)),
        ]

        for _, strip, direction, offset in side_specs:
            if strip.size == 0 or not np.any(strip):
                continue

            local_ys, local_xs = np.where(strip)
            if direction == "vertical":
                span_px = int(local_ys.max() - local_ys.min() + 1)
                global_ys = local_ys
                if offset == 0:
                    global_xs = local_xs
                else:
                    global_xs = local_xs + offset
            else:
                span_px = int(local_xs.max() - local_xs.min() + 1)
                global_xs = local_xs
                if offset == 0:
                    global_ys = local_ys
                else:
                    global_ys = local_ys + offset

            z_vals = depth_m[global_ys, global_xs]
            z_vals = z_vals[np.isfinite(z_vals) & (z_vals > 0.0)]
            if z_vals.size == 0:
                continue

            z_med = float(np.median(z_vals))

            if intrinsics is not None:
                focal = intrinsics.fy if direction == "vertical" else intrinsics.fx
                contact_m = span_px * z_med / float(focal)
            else:
                # fallback: D435 640x480에서 fx/fy가 대략 600 근처.
                contact_m = span_px * z_med / 600.0

            contacts.append(float(contact_m))

        return max(contacts) if contacts else 0.0

    @staticmethod
    def detection_overlaps_depth_mask(
        xyxy,
        depth_object_mask,
        min_overlap_ratio=0.06,
    ):
        """YOLO bbox 안에 depth volume mask가 충분히 있는지 검사."""
        if depth_object_mask is None:
            return True

        h, w = depth_object_mask.shape[:2]
        x1, y1, x2, y2 = map(int, xyxy)

        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h, y2))

        if x2 <= x1 or y2 <= y1:
            return False

        crop = depth_object_mask[y1:y2, x1:x2]
        if crop.size <= 0:
            return False

        overlap_ratio = np.count_nonzero(crop) / float(crop.size)
        return bool(overlap_ratio >= min_overlap_ratio)

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