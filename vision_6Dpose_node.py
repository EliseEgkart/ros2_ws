# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import cv2
# import time

# class VisionNode(Node):
#     def __init__(self):
#         super().__init__('vision_node')
#         self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
#         self.model = YOLO("/home/da/dis_duplo_ws/best.pt")

#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         profile = self.pipeline.start(config)
#         self.align = rs.align(rs.stream.color)
#         self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#         self.latest_color = None
#         self.latest_depth = None
#         self.latest_results = None

#         self.create_timer(0.033, self.visualize_callback)
#         self.get_logger().info("✅ Vision Node: 층수(Layer) 자동 계산 및 최고층 우선 파지 모드 가동")

#     def calculate_refined_yaw(self, rect):
#         (cx, cy), (w, h), angle = rect
#         if w < h:
#             yaw = angle
#         else:
#             yaw = angle + 90.0

#         if yaw > 90: yaw -= 180
#         if yaw < -90: yaw += 180
#         return yaw

#     def visualize_callback(self):
#         try:
#             frames = self.pipeline.wait_for_frames(timeout_ms=1000)
#             aligned = self.align.process(frames)
#             self.latest_depth = aligned.get_depth_frame()
#             color_frame = aligned.get_color_frame()
            
#             if not color_frame or not self.latest_depth: return

#             self.latest_color = np.asanyarray(color_frame.get_data())
#             self.latest_results = self.model(self.latest_color, verbose=False)[0]

#             display_img = self.latest_results.plot()
#             cv2.circle(display_img, (320, 240), 5, (0, 0, 255), -1)

#             if self.latest_results.boxes is not None:
#                 for i, box in enumerate(self.latest_results.boxes):
#                     xyxy = box.xyxy[0].cpu().numpy()
#                     u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
#                     yaw = 0.0

#                     if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
#                         pts = np.int32([self.latest_results.masks.xy[i]])
#                         M = cv2.moments(pts)
#                         if M["m00"] != 0:
#                             u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
#                             rect = cv2.minAreaRect(pts)
#                             yaw = self.calculate_refined_yaw(rect)
                    
#                     z_val = self.latest_depth.get_distance(u, v)
#                     if z_val > 0:
#                         x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
#                         cv2.putText(display_img, f"X:{x_r*1000:.1f} Y:{y_r*1000:.1f}", (u - 60, v + 25), 
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                         cv2.putText(display_img, f"Yaw:{yaw:.1f}", (u - 60, v + 45), 
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

#             cv2.imshow("6D Pose (Refined Yaw)", display_img)
#             cv2.waitKey(1)
#         except Exception:
#             pass

#     def get_valid_depth(self, depth_frame, u, v, search_radius=10):
#         z = depth_frame.get_distance(u, v)
#         if z > 0: return z
#         for r in range(1, search_radius + 1):
#             for dx in range(-r, r + 1):
#                 for dy in range(-r, r + 1):
#                     nu, nv = u + dx, v + dy
#                     if 0 <= nu < 640 and 0 <= nv < 480:
#                         z = depth_frame.get_distance(nu, nv)
#                         if z > 0: return z
#         return 0.0

#     def get_pose_cb(self, request, response):
#         target = request.target_color.lower()

#         # [개수 파악 로직 유지]
#         if target.startswith("count_"):
#             search_color = target.replace("count_", "")
#             start_time = time.time()
#             max_count = 0
#             while time.time() - start_time < 0.5:
#                 try:
#                     frames = self.pipeline.wait_for_frames(timeout_ms=500)
#                     aligned = self.align.process(frames)
#                     color_f = aligned.get_color_frame()
#                     if not color_f: continue

#                     img = np.asanyarray(color_f.get_data())
#                     results = self.model(img, verbose=False)[0]
                    
#                     if results.boxes is not None:
#                         current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
#                         max_count = max(max_count, current_count)
#                 except Exception:
#                     pass
#             response.success = True
#             response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
#             # 카운트 시에는 layer 0 반환
#             response.layer = 0 
#             return response

#         # -------------------------------------------------------------
#         self.get_logger().info(f"🔍 '{target}' 정밀 측정 (모든 층 탐색 및 Layer 계산 중...)")
        
#         samples = []
#         start_time = time.time()
        
#         while time.time() - start_time < 1.2:
#             try:
#                 frames = self.pipeline.wait_for_frames(timeout_ms=500)
#                 aligned = self.align.process(frames)
#                 depth_f = aligned.get_depth_frame()
#                 color_f = aligned.get_color_frame()
                
#                 if not color_f or not depth_f: continue

#                 img = np.asanyarray(color_f.get_data())
#                 results = self.model(img, verbose=False)[0]
#                 if results.boxes is None: continue

#                 all_z_values = [] # 🌟 화면에 보이는 모든 블록의 Z값을 모아 바닥을 찾기 위함
#                 frame_targets = []
                
#                 for i, box in enumerate(results.boxes):
#                     cls_name = results.names[int(box.cls[0])].lower()
                    
#                     u, v, yaw = 0, 0, 0.0
#                     if results.masks is not None and len(results.masks.xy) > i:
#                         pts = np.int32([results.masks.xy[i]])
#                         M = cv2.moments(pts)
#                         if M["m00"] != 0:
#                             u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
#                             rect = cv2.minAreaRect(pts)
#                             yaw = self.calculate_refined_yaw(rect)
#                     else:
#                         xyxy = box.xyxy[0].cpu().numpy()
#                         u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)

#                     z = self.get_valid_depth(depth_f, u, v)
                    
#                     if z > 0:
#                         all_z_values.append(z) # 색상 상관없이 Z값 수집
                        
#                         # 우리가 찾는 타겟 색상인 경우 따로 저장
#                         if target in cls_name:
#                             frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw})

#                 if frame_targets and all_z_values:
#                     # 🌟 1. 진짜 바닥 찾기 (모든 블록 중 가장 먼 Z값)
#                     floor_z = max(all_z_values)
                    
#                     # 🌟 2. 분해 최우선 타겟 선택 (해당 색상 중 가장 Z가 작은 = 카메라에 가까운 = 제일 높은 놈)
#                     best = min(frame_targets, key=lambda t: t['z'])
                    
#                     # 🌟 3. 층수 계산 (바닥 Z - 타겟 Z) / 블록 높이 16mm
#                     # 결과: 바닥이면 1, 한 칸 위면 2, 두 칸 위면 3...
#                     calculated_layer = int(round((floor_z - best['z']) / 0.016)) + 1
                    
#                     x, y, z_real = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best['u'], best['v']], best['z'])
                    
#                     # x, y, z, yaw, layer 순으로 저장
#                     samples.append([x, y, z_real, best['yaw'], calculated_layer])
                
#                 time.sleep(0.01)
#             except Exception:
#                 continue

#         if len(samples) < 5:
#             self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
#             response.success = False
#             return response

#         samples = np.array(samples)
#         median_pose = np.median(samples, axis=0)
        
#         response.success = True
#         response.x = float(median_pose[0])
#         response.y = float(median_pose[1])
#         response.z = float(median_pose[2])
#         response.yaw = float(median_pose[3])
#         # 🌟 새롭게 추가된 층수 데이터 응답!
#         response.layer = int(median_pose[4]) 
        
#         self.get_logger().info(f"🎯 타겟 확정: X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f} | 🏢 판정 층수: {response.layer}층")
#         return response

# def main():
#     rclpy.init()
#     node = VisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import cv2
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        
        # 🌟 앙상블 모드: 두 개의 모델을 동시에 로드합니다.
        self.model_det = YOLO("/home/han/duplo_ws/best.pt")      # 좌표 안정화용 새 모델 (detect)
        self.model_seg = YOLO("/home/han/duplo_ws/best_old.pt")  # Yaw 추출용 구형 모델 (segment)

        self.get_logger().info(f"✅ YOLO 새 모델 Task: {self.model_det.task}")
        self.get_logger().info(f"✅ YOLO 예전 모델 Task: {self.model_seg.task}")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_color = None
        self.latest_depth = None

        self.create_timer(0.033, self.visualize_callback)
        self.get_logger().info("🚀 Vision Node: 앙상블 + 층수(Layer) 자동 계산 마스터 모드 가동")

    def calculate_refined_yaw(self, rect):
        (cx, cy), (w, h), angle = rect
        if w < h:
            yaw = angle
        else:
            yaw = angle + 90.0

        if yaw > 90: yaw -= 180
        if yaw < -90: yaw += 180
        return yaw

    def visualize_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            self.latest_depth = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            
            if not color_frame or not self.latest_depth: return

            self.latest_color = np.asanyarray(color_frame.get_data())
            
            # 두 모델로 각각 추론을 돌립니다.
            res_det = self.model_det(self.latest_color, verbose=False)[0]
            res_seg = self.model_seg(self.latest_color, verbose=False)[0]

            # 시각화 화면은 좌표가 안정적인 새 모델(res_det)의 박스를 기준으로 그립니다.
            display_img = res_det.plot()
            cv2.circle(display_img, (320, 240), 5, (0, 0, 255), -1)

            if res_det.boxes is not None:
                for i, box in enumerate(res_det.boxes):
                    xyxy = box.xyxy[0].cpu().numpy()
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    yaw = 0.0

                    # 앙상블 매칭 알고리즘: 새 모델의 중심점과 가장 가까운 옛날 모델의 마스크를 찾음
                    min_dist = float('inf')
                    best_mask_pts = None

                    if res_seg.masks is not None and res_seg.boxes is not None:
                        for j, seg_box in enumerate(res_seg.boxes):
                            s_xyxy = seg_box.xyxy[0].cpu().numpy()
                            s_u, s_v = int((s_xyxy[0] + s_xyxy[2]) / 2), int((s_xyxy[1] + s_xyxy[3]) / 2)
                            
                            dist = ((u - s_u) ** 2 + (v - s_v) ** 2) ** 0.5
                            if dist < 40 and dist < min_dist:
                                min_dist = dist
                                if len(res_seg.masks.xy) > j:
                                    best_mask_pts = np.int32(res_seg.masks.xy[j])

                    if best_mask_pts is not None and len(best_mask_pts) >= 3:
                        M = cv2.moments(best_mask_pts)
                        if M["m00"] != 0:
                            rect = cv2.minAreaRect(best_mask_pts)
                            yaw = self.calculate_refined_yaw(rect)
                    
                    z_val = self.latest_depth.get_distance(u, v)
                    if z_val > 0:
                        x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        cv2.putText(display_img, f"X:{x_r*1000:.1f} Y:{y_r*1000:.1f}", (u - 60, v + 25), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(display_img, f"Yaw:{yaw:.1f}", (u - 60, v + 45), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.imshow("6D Pose (Ensemble Mode)", display_img)
            cv2.waitKey(1)
        except Exception:
            pass

    def get_valid_depth(self, depth_frame, u, v, search_radius=10):
        z = depth_frame.get_distance(u, v)
        if z > 0: return z
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nu, nv = u + dx, v + dy
                    if 0 <= nu < 640 and 0 <= nv < 480:
                        z = depth_frame.get_distance(nu, nv)
                        if z > 0: return z
        return 0.0

    def get_pose_cb(self, request, response):
        target = request.target_color.lower()

        # [개수 파악 로직] - 새 모델(model_det) 기준 카운트
        if target.startswith("count_"):
            search_color = target.replace("count_", "")
            start_time = time.time()
            max_count = 0
            while time.time() - start_time < 0.5:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=500)
                    aligned = self.align.process(frames)
                    color_f = aligned.get_color_frame()
                    if not color_f: continue

                    img = np.asanyarray(color_f.get_data())
                    results = self.model_det(img, verbose=False)[0]
                    
                    if results.boxes is not None:
                        current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
                        max_count = max(max_count, current_count)
                except Exception:
                    pass
            response.success = True
            response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
            response.layer = 0
            return response

        # -------------------------------------------------------------
        # 🌟 앙상블 매칭 + 모든 층 스캔 및 실시간 층수 역산 로직 통합
        self.get_logger().info(f"🔍 '{target}' 앙상블 정밀 측정 및 층수 분석 중...")
        
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < 1.2:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                aligned = self.align.process(frames)
                depth_f = aligned.get_depth_frame()
                color_f = aligned.get_color_frame()
                
                if not color_f or not depth_f: continue

                img = np.asanyarray(color_f.get_data())
                res_det = self.model_det(img, verbose=False)[0]
                res_seg = self.model_seg(img, verbose=False)[0]
                
                if res_det.boxes is None: continue

                all_z_values = []  # 화면 안의 모든 오브젝트 Z값을 모아 진짜 바닥을 추정하기 위함
                frame_targets = []
                
                for i, box in enumerate(res_det.boxes):
                    cls_name = res_det.names[int(box.cls[0])].lower()
                    
                    # 1. 새 모델의 안정적인 바운딩 박스 중심점 추출
                    xyxy = box.xyxy[0].cpu().numpy()
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    yaw = 0.0

                    z = self.get_valid_depth(depth_f, u, v)
                    if z > 0:
                        all_z_values.append(z) # 색상 구분 없이 모든 깊이 정보 수집

                        # 우리가 찾는 목표 색상 블록인 경우
                        if target in cls_name:
                            # 2. 옛날 세그멘테이션 모델과 중심점 픽셀 매칭 (Yaw 추출용)
                            min_dist = float('inf')
                            best_mask_pts = None
                            if res_seg.masks is not None and res_seg.boxes is not None:
                                for j, seg_box in enumerate(res_seg.boxes):
                                    s_xyxy = seg_box.xyxy[0].cpu().numpy()
                                    s_u, s_v = int((s_xyxy[0] + s_xyxy[2]) / 2), int((s_xyxy[1] + s_xyxy[3]) / 2)
                                    
                                    dist = ((u - s_u) ** 2 + (v - s_v) ** 2) ** 0.5
                                    if dist < 40 and dist < min_dist:
                                        min_dist = dist
                                        if len(res_seg.masks.xy) > j:
                                            best_mask_pts = np.int32(res_seg.masks.xy[j])

                            # 3. 매칭 성공 시 구형 모델 데이터로 정확한 Yaw 보정 계산
                            if best_mask_pts is not None and len(best_mask_pts) >= 3:
                                M = cv2.moments(best_mask_pts)
                                if M["m00"] != 0:
                                    rect = cv2.minAreaRect(best_mask_pts)
                                    yaw = self.calculate_refined_yaw(rect)

                            frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw})

                if frame_targets and all_z_values:
                    # 🌟 1) 진짜 바닥 기준점 탐색 (화면 내 모든 감지 블록 중 가장 먼 Z값)
                    floor_z = max(all_z_values)
                    
                    # 🌟 2) 분해 타겟 선정 (해당 색상 블록 중 가장 카메라와 가까운 = Z가 가장 작은 최고층 블록)
                    best = min(frame_targets, key=lambda t: t['z'])
                    
                    # 🌟 3) 층수 역산 (바닥 기준점 - 타겟 높이) / 블록 높이 16mm
                    calculated_layer = int(round((floor_z - best['z']) / 0.016)) + 1
                    
                    x, y, z_real = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best['u'], best['v']], best['z'])
                    samples.append([x, y, z_real, best['yaw'], calculated_layer])
                
                time.sleep(0.01)
            except Exception:
                continue

        if len(samples) < 5:
            self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
            response.success = False
            return response

        samples = np.array(samples)
        median_pose = np.median(samples, axis=0)
        
        response.success = True
        response.x = float(median_pose[0])
        response.y = float(median_pose[1])
        response.z = float(median_pose[2])
        response.yaw = float(median_pose[3])
        # 🌟 계산된 층수 데이터를 서비스 응답에 포함
        response.layer = int(median_pose[4]) 
        
        self.get_logger().info(f"🎯 타겟 확정(앙상블+레이어): X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f} | 🏢 판정 층수: {response.layer}층")
        return response

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()