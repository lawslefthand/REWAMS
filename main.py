#!/usr/bin/env python3

import cv2
import numpy as np
import onnxruntime as ort
import argparse
import time
import json
import math
import os
from datetime import datetime
from typing import List, Dict, Tuple
import sys
from collections import deque

try:
    import gpsd
    GPS_AVAILABLE = True
except ImportError:
    GPS_AVAILABLE = False
    print("⚠️  GPS module not available. Using manual coordinates.")


class HighPrecisionDroneDetectionSystem:
    def __init__(self,
                 model_path: str,
                 altitude: float = 8.0,
                 camera_fov: float = 84.0,
                 confidence_threshold: float = 0.3,
                 nms_threshold: float = 0.3,
                 desktop_path: str = "C:\\Users\\Namit\\Desktop\\rew_ncnn"):
        self.model_path = model_path
        self.altitude = altitude
        self.camera_fov = camera_fov
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.desktop_path = desktop_path

        os.makedirs(desktop_path, exist_ok=True)

        print("🔄 Loading YOLO model with RPi 5 optimizations...")
        providers = ['CPUExecutionProvider']
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = 4
        session_options.inter_op_num_threads = 2
        session_options.execution_mode = ort.ExecutionMode.ORT_PARALLEL
        session_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=providers
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        print("✅ YOLO model loaded with performance optimizations")

        self.EARTH_RADIUS_EQUATORIAL = 6378137.0
        self.EARTH_RADIUS_POLAR = 6356752.314245
        self.FLATTENING = 1 / 298.257223563

        self.video_cap = None
        self.all_detections = []
        self.frame_results = {}

        self.drone_gps = (0.0, 0.0)
        self.setup_gps()

        self.processing_times = deque(maxlen=100)
        self.fps_history = deque(maxlen=30)
        self.target_fps = 20.0
        self.frame_skip_adaptive = False

        self.distance_corrections = []
        self.gps_precision_history = []

    def setup_gps(self):
        if GPS_AVAILABLE:
            try:
                gpsd.connect()
                print("✅ GPS connected")
                return True
            except Exception as e:
                print(f"❌ GPS connection failed: {e}")
                return False
        return False

    def get_gps_coordinates_precise(self) -> Tuple[float, float, float]:
        if GPS_AVAILABLE:
            try:
                packet = gpsd.get_current()
                if packet.lat != 0 and packet.lon != 0:
                    accuracy = getattr(packet, 'error', {}).get('x', 5.0)
                    return (packet.lat, packet.lon, accuracy)
            except Exception as e:
                print(f"⚠️  GPS read error: {e}")

        return (*self.drone_gps, 3.0)

    def calculate_earth_radius_at_latitude(self, latitude: float) -> float:
        lat_rad = math.radians(latitude)
        cos_lat = math.cos(lat_rad)
        sin_lat = math.sin(lat_rad)

        a_cos_lat_sq = self.EARTH_RADIUS_EQUATORIAL * cos_lat
        b_sin_lat_sq = self.EARTH_RADIUS_POLAR * sin_lat

        numerator = (self.EARTH_RADIUS_EQUATORIAL * a_cos_lat_sq) ** 2 + (self.EARTH_RADIUS_POLAR * b_sin_lat_sq) ** 2
        denominator = (a_cos_lat_sq) ** 2 + (b_sin_lat_sq) ** 2

        return math.sqrt(numerator / denominator)

    def setup_video_optimized(self, video_path: str):
        print(f"🎬 Loading video with RPi 5 optimizations: {video_path}")

        if not os.path.exists(video_path):
            raise Exception(f"❌ Video file not found: {video_path}")

        self.video_cap = cv2.VideoCapture(video_path)

        self.video_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.video_cap.isOpened():
            raise Exception(f"❌ Failed to open video: {video_path}")

        self.total_frames = int(self.video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.video_cap.get(cv2.CAP_PROP_FPS)
        self.frame_width = int(self.video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.duration = self.total_frames / self.fps if self.fps > 0 else 0

        print(f"✅ Video loaded - {self.frame_width}x{self.frame_height} @ {self.fps:.2f}FPS")
        print(f"📊 {self.total_frames} frames, {self.duration:.2f}s duration")

    def preprocess_frame_optimized(self, frame: np.ndarray) -> np.ndarray:
        img_resized = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_LINEAR)

        img_normalized = img_resized.astype(np.float32) * (1.0 / 255.0)

        img_rgb = cv2.cvtColor(img_normalized, cv2.COLOR_BGR2RGB)
        img_transposed = np.transpose(img_rgb, (2, 0, 1))

        return np.expand_dims(img_transposed, axis=0)

    def calculate_precise_gps_coordinates(self, detections: List[Dict], frame_shape: Tuple[int, int]) -> List[Dict]:
        if not detections:
            return []

        H, W = frame_shape[:2]
        frame_center_x, frame_center_y = W // 2, H // 2

        drone_lat, drone_lon, gps_accuracy = self.get_gps_coordinates_precise()

        earth_radius = self.calculate_earth_radius_at_latitude(drone_lat)

        fov_rad = math.radians(self.camera_fov)

        distortion_factor = 1.0 - (0.1 * (self.camera_fov - 60) / 60) if self.camera_fov > 60 else 1.0
        effective_fov = fov_rad * distortion_factor

        ground_width = 2 * self.altitude * math.tan(effective_fov / 2)
        ground_height = ground_width * (H / W)

        meters_per_pixel_x = ground_width / W
        meters_per_pixel_y = ground_height / H

        pixel_precision_x = meters_per_pixel_x
        pixel_precision_y = meters_per_pixel_y

        results = []
        for i, detection in enumerate(detections):
            center_x = detection['center']['x']
            center_y = detection['center']['y']

            offset_x = center_x - frame_center_x
            offset_y = frame_center_y - center_y

            offset_meters_x = offset_x * meters_per_pixel_x
            offset_meters_y = offset_y * meters_per_pixel_y

            distance_from_center = math.sqrt(offset_meters_x ** 2 + offset_meters_y ** 2)

            lat_rad = math.radians(drone_lat)

            M = earth_radius * (1 - self.FLATTENING) ** 2 / (
                    (1 - (2 * self.FLATTENING - self.FLATTENING ** 2) * math.sin(lat_rad) ** 2) ** (3 / 2))

            N = earth_radius / math.sqrt(1 - (2 * self.FLATTENING - self.FLATTENING ** 2) * math.sin(lat_rad) ** 2)

            lat_offset = offset_meters_y / M * (180 / math.pi)
            lon_offset = offset_meters_x / (N * math.cos(lat_rad)) * (180 / math.pi)

            human_lat = drone_lat + lat_offset
            human_lon = drone_lon + lon_offset

            pixel_accuracy = math.sqrt(pixel_precision_x ** 2 + pixel_precision_y ** 2)
            total_accuracy = math.sqrt(gps_accuracy ** 2 + pixel_accuracy ** 2)

            result = {
                'id': i + 1,
                'latitude': human_lat,
                'longitude': human_lon,
                'distance_from_center_m': distance_from_center,
                'accuracy_estimate_m': total_accuracy,
                'pixel_coordinates': {'x': center_x, 'y': center_y},
                'offset_meters': {'x': offset_meters_x, 'y': offset_meters_y},
                'confidence': detection['confidence'],
                'bbox': detection['bbox'],
                'precision_metrics': {
                    'meters_per_pixel_x': pixel_precision_x,
                    'meters_per_pixel_y': pixel_precision_y,
                    'gps_accuracy': gps_accuracy,
                    'earth_radius_used': earth_radius
                }
            }
            results.append(result)

        return results

    def postprocess_detections_optimized(self, outputs: np.ndarray, frame_shape: Tuple[int, int]) -> List[Dict]:
        H, W = frame_shape[:2]

        if len(outputs[0].shape) == 3:
            output_data = outputs[0][0].T
        else:
            output_data = outputs[0].squeeze().T

        valid_detections = output_data[output_data[:, 4] >= self.confidence_threshold]

        if len(valid_detections) == 0:
            return []

        boxes = []
        confidences = []

        x_scale, y_scale = W / 640, H / 640

        for detection in valid_detections:
            x_center, y_center, w, h = detection[0:4]
            confidence = detection[4]

            if len(detection) > 5:
                class_scores = detection[5:]
                class_id = np.argmax(class_scores)
                confidence = confidence * class_scores[class_id]

                if class_id != 0:
                    continue

            if confidence >= self.confidence_threshold:
                x1 = max(0, min(int((x_center - w / 2) * x_scale), W - 1))
                y1 = max(0, min(int((y_center - h / 2) * y_scale), H - 1))
                box_w = min(int(w * x_scale), W - x1)
                box_h = min(int(h * y_scale), H - y1)

                boxes.append([x1, y1, box_w, box_h])
                confidences.append(float(confidence))

        detections = []
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, confidences,
                                       self.confidence_threshold,
                                       self.nms_threshold)

            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    center_x = x + w // 2
                    center_y = y + h // 2

                    detections.append({
                        'bbox': {'x': x, 'y': y, 'width': w, 'height': h},
                        'center': {'x': center_x, 'y': center_y},
                        'confidence': confidences[i],
                        'class_id': 0
                    })

        return detections

    def draw_enhanced_detections(self, frame: np.ndarray, results: List[Dict],
                                 frame_number: int = 0, timestamp: float = 0.0,
                                 current_fps: float = 0.0) -> np.ndarray:
        H, W = frame.shape[:2]
        frame_center_x, frame_center_y = W // 2, H // 2

        cv2.line(frame, (frame_center_x - 20, frame_center_y), (frame_center_x + 20, frame_center_y), (0, 255, 255), 3)
        cv2.line(frame, (frame_center_x, frame_center_y - 20), (frame_center_x, frame_center_y + 20), (0, 255, 255), 3)
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 255, 255), -1)

        cv2.putText(frame, "DRONE CENTER", (frame_center_x + 25, frame_center_y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        for result in results:
            x, y = result['pixel_coordinates']['x'], result['pixel_coordinates']['y']
            bbox = result['bbox']
            distance = result['distance_from_center_m']
            accuracy = result['accuracy_estimate_m']

            if distance < 5.0:
                color = (0, 255, 0)
            elif distance < 15.0:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)

            cv2.rectangle(frame, (bbox['x'], bbox['y']),
                          (bbox['x'] + bbox['width'], bbox['y'] + bbox['height']),
                          color, 3)

            cv2.circle(frame, (x, y), 12, color, -1)
            cv2.circle(frame, (x, y), 15, (255, 255, 255), 2)

            cv2.line(frame, (frame_center_x, frame_center_y), (x, y), color, 3)

            if distance > 5.0:
                steps = int(distance / 5.0)
                for step in range(1, steps + 1):
                    marker_x = int(frame_center_x + (x - frame_center_x) * (step * 5.0 / distance))
                    marker_y = int(frame_center_y + (y - frame_center_y) * (step * 5.0 / distance))
                    cv2.circle(frame, (marker_x, marker_y), 4, (255, 255, 255), -1)

            info_lines = [
                f"HUMAN {result['id']}",
                f"DIST: {distance:.1f}m (±{accuracy:.1f}m)",
                f"GPS: {result['latitude']:.6f}",
                f"     {result['longitude']:.6f}",
                f"CONF: {result['confidence']:.3f}"
            ]

            for i, line in enumerate(info_lines):
                y_offset = y - 60 + (i * 20)
                text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                cv2.rectangle(frame, (x + 15, y_offset - 15),
                              (x + 20 + text_size[0], y_offset + 5), (0, 0, 0), -1)
                cv2.putText(frame, line, (x + 18, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        status_bg_height = 180
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (500, status_bg_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        status_info = [
            f"FLOOD RESCUE DETECTION SYSTEM",
            f"Frame: {frame_number}/{self.total_frames} | Time: {timestamp:.1f}s",
            f"FPS: {current_fps:.1f} | Humans: {len(results)}",
            f"Altitude: {self.altitude}m | FOV: {self.camera_fov}°",
            f"GPS: {self.get_gps_coordinates_precise()[0]:.6f}, {self.get_gps_coordinates_precise()[1]:.6f}",
            f"Confidence: {self.confidence_threshold} | NMS: {self.nms_threshold}",
            f"Target FPS: {self.target_fps} | Processing Optimized for RPi 5"
        ]

        for i, text in enumerate(status_info):
            y_pos = 25 + (i * 22)
            font_scale = 0.8 if i == 0 else 0.6
            color = (0, 255, 255) if i == 0 else (255, 255, 255)
            thickness = 2 if i == 0 else 1
            cv2.putText(frame, text, (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

        return frame

    def process_frame_optimized(self, frame: np.ndarray) -> List[Dict]:
        input_tensor = self.preprocess_frame_optimized(frame)

        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})

        detections = self.postprocess_detections_optimized(outputs, frame.shape)

        results = self.calculate_precise_gps_coordinates(detections, frame.shape)

        return results

    def print_inference_results(self, results: List[Dict], frame_number: int,
                                timestamp: float, processing_time: float, current_fps: float):
        print(f"\n{'=' * 80}")
        print(f"🚁 FRAME {frame_number} | TIME: {timestamp:.2f}s | FPS: {current_fps:.1f}")
        print(f"⚡ Processing: {processing_time * 1000:.1f}ms | Target FPS: {self.target_fps}")

        if results:
            print(f"🎯 DETECTED {len(results)} HUMAN(S) - FLOOD RESCUE COORDINATES:")
            print(
                f"{'ID':<3} {'DISTANCE':<12} {'GPS LATITUDE':<12} {'GPS LONGITUDE':<13} {'ACCURACY':<10} {'CONFIDENCE':<10}")
            print(f"{'-' * 3} {'-' * 12} {'-' * 12} {'-' * 13} {'-' * 10} {'-' * 10}")

            for result in results:
                id_val = result['id']
                distance_str = f"{result['distance_from_center_m']:.1f}m"
                lat_str = f"{result['latitude']:.6f}"
                lon_str = f"{result['longitude']:.6f}"
                accuracy_str = f"±{result['accuracy_estimate_m']:.1f}m"
                confidence_str = f"{result['confidence']:.3f}"

                print(f"{id_val:<3} {distance_str:<12} {lat_str:<12} {lon_str:<13} {accuracy_str:<10} {confidence_str:<10}")

            distances = [r['distance_from_center_m'] for r in results]
            accuracies = [r['accuracy_estimate_m'] for r in results]
            confidences = [r['confidence'] for r in results]

            print(f"\n📊 SUMMARY:")
            print(f"   Closest human: {min(distances):.1f}m")
            print(f"   Average distance: {sum(distances) / len(distances):.1f}m")
            print(f"   Average accuracy: ±{sum(accuracies) / len(accuracies):.1f}m")
            print(f"   Average confidence: {sum(confidences) / len(confidences):.3f}")
        else:
            print("🔍 NO HUMANS DETECTED IN CURRENT FRAME")

        drone_lat, drone_lon, gps_acc = self.get_gps_coordinates_precise()
        print(f"📍 Drone GPS: {drone_lat:.6f}, {drone_lon:.6f} (±{gps_acc:.1f}m)")

    def process_video_high_performance(self,
                                       video_path: str,
                                       save_results: bool = False,
                                       save_video: bool = False,
                                       display: bool = True,
                                       debug: bool = False) -> List[Dict]:
        print(f"🎬 HIGH-PERFORMANCE PROCESSING: {video_path}")
        print(f"🎯 Target FPS: {self.target_fps} | Optimized for Raspberry Pi 5")

        self.setup_video_optimized(video_path)

        video_writer = None
        if save_video:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_name = os.path.splitext(os.path.basename(video_path))[0]
            output_video_path = os.path.join(self.desktop_path, f"flood_rescue_{video_name}_{timestamp}.mp4")

            fourcc = cv2.VideoWriter_fourcc(*'H264')
            video_writer = cv2.VideoWriter(output_video_path, fourcc, self.fps,
                                           (self.frame_width, self.frame_height))

            if not video_writer.isOpened():
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(output_video_path, fourcc, self.fps,
                                               (self.frame_width, self.frame_height))

            print(f"💾 Output video will be saved to: {output_video_path}")

        frame_number = 0
        start_time = time.time()
        last_fps_update = start_time

        try:
            while True:
                ret, frame = self.video_cap.read()
                if not ret:
                    break

                frame_number += 1
                timestamp = (frame_number - 1) / self.fps if self.fps > 0 else 0

                frame_start_time = time.time()
                results = self.process_frame_optimized(frame)
                processing_time = time.time() - frame_start_time

                self.processing_times.append(processing_time)
                current_fps = 1.0 / processing_time if processing_time > 0 else 0
                self.fps_history.append(current_fps)

                frame_data = {
                    'frame_number': frame_number,
                    'timestamp': timestamp,
                    'detections': results,
                    'processing_time': processing_time,
                    'fps': current_fps
                }
                self.frame_results[frame_number] = frame_data
                if results:
                    self.all_detections.extend(results)

                if debug or frame_number % 10 == 0:
                    self.print_inference_results(results, frame_number, timestamp,
                                                 processing_time, current_fps)

                if time.time() - last_fps_update > 2.0:
                    if self.fps_history:
                        avg_fps = sum(self.fps_history) / len(self.fps_history)
                        if avg_fps < self.target_fps * 0.8:
                            print(f"⚠️  Performance warning: {avg_fps:.1f} FPS (target: {self.target_fps})")
                    last_fps_update = time.time()

                if display or save_video:
                    annotated_frame = self.draw_enhanced_detections(
                        frame.copy(), results, frame_number, timestamp, current_fps
                    )

                    if display:
                        display_frame = annotated_frame
                        if self.frame_width > 1280:
                            scale = 1280 / self.frame_width
                            new_size = (int(self.frame_width * scale), int(self.frame_height * scale))
                            display_frame = cv2.resize(annotated_frame, new_size,
                                                       interpolation=cv2.INTER_LINEAR)

                        cv2.imshow('FLOOD RESCUE - Human Detection System', display_frame)

                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            print("\n🛑 Stopping video processing...")
                            break
                        elif key == ord('s'):
                            self.save_current_detections(frame_number, results)

                    if save_video and video_writer:
                        video_writer.write(annotated_frame)

        except KeyboardInterrupt:
            print("\n🛑 Emergency stop - Video processing interrupted")
        finally:
            self.cleanup_video(video_writer)

        total_time = time.time() - start_time
        self.print_performance_summary(total_time, frame_number)

        if save_results:
            self.save_comprehensive_results(video_path)

        return self.all_detections

    def save_current_detections(self, frame_number: int, results: List[Dict]):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.desktop_path, f"emergency_detections_frame_{frame_number}_{timestamp}.json")

        save_data = {
            'emergency_save': True,
            'frame_number': frame_number,
            'timestamp': timestamp,
            'detections': results,
            'drone_info': {
                'gps': self.get_gps_coordinates_precise(),
                'altitude': self.altitude,
                'camera_fov': self.camera_fov
            }
        }

        with open(filename, 'w') as f:
            json.dump(save_data, f, indent=2)

        print(f"🚨 EMERGENCY SAVE: Frame {frame_number} detections saved to desktop")

    def save_comprehensive_results(self, video_path: str):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_name = os.path.splitext(os.path.basename(video_path))[0]

        results_filename = os.path.join(self.desktop_path, f"flood_rescue_analysis_{video_name}_{timestamp}.json")

        total_detections = len(self.all_detections)
        frames_with_humans = len([f for f in self.frame_results.values() if f['detections']])
        avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
        avg_fps = sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0

        if self.all_detections:
            lats = [d['latitude'] for d in self.all_detections]
            lons = [d['longitude'] for d in self.all_detections]
            distances = [d['distance_from_center_m'] for d in self.all_detections]
            accuracies = [d['accuracy_estimate_m'] for d in self.all_detections]

            gps_bounds = {
                'north': max(lats),
                'south': min(lats),
                'east': max(lons),
                'west': min(lons),
                'center': {
                    'lat': sum(lats) / len(lats),
                    'lon': sum(lons) / len(lons)
                }
            }

            distance_stats = {
                'closest': min(distances),
                'farthest': max(distances),
                'average': sum(distances) / len(distances),
                'within_10m': len([d for d in distances if d <= 10.0]),
                'within_20m': len([d for d in distances if d <= 20.0])
            }

            accuracy_stats = {
                'best_accuracy': min(accuracies),
                'worst_accuracy': max(accuracies),
                'average_accuracy': sum(accuracies) / len(accuracies)
            }
        else:
            gps_bounds = None
            distance_stats = None
            accuracy_stats = None

        comprehensive_data = {
            'flood_rescue_mission': {
                'video_source': video_path,
                'analysis_timestamp': timestamp,
                'drone_operator': 'Flood Rescue Team',
                'mission_status': 'COMPLETED' if total_detections > 0 else 'NO_HUMANS_FOUND'
            },
            'video_info': {
                'total_frames': self.total_frames,
                'processed_frames': len(self.frame_results),
                'fps': self.fps,
                'duration': self.duration,
                'resolution': f"{self.frame_width}x{self.frame_height}"
            },
            'detection_settings': {
                'model_path': self.model_path,
                'altitude': self.altitude,
                'camera_fov': self.camera_fov,
                'confidence_threshold': self.confidence_threshold,
                'nms_threshold': self.nms_threshold
            },
            'performance_metrics': {
                'target_fps': self.target_fps,
                'average_fps_achieved': avg_fps,
                'average_processing_time_ms': avg_processing_time * 1000,
                'performance_rating': 'EXCELLENT' if avg_fps >= self.target_fps else 'GOOD' if avg_fps >= self.target_fps * 0.8 else 'NEEDS_OPTIMIZATION'
            },
            'rescue_statistics': {
                'total_humans_detected': total_detections,
                'frames_with_detections': frames_with_humans,
                'detection_rate': frames_with_humans / len(self.frame_results) if self.frame_results else 0,
                'gps_coverage_area': gps_bounds,
                'distance_analysis': distance_stats,
                'accuracy_analysis': accuracy_stats
            },
            'drone_telemetry': {
                'gps_coordinates': self.get_gps_coordinates_precise(),
                'altitude_m': self.altitude,
                'camera_specifications': {
                    'fov_degrees': self.camera_fov,
                    'ground_coverage_m': 2 * self.altitude * math.tan(math.radians(self.camera_fov) / 2)
                }
            },
            'detailed_frame_results': self.frame_results,
            'all_detections_chronological': self.all_detections
        }

        with open(results_filename, 'w') as f:
            json.dump(comprehensive_data, f, indent=2)

        if self.all_detections:
            coords_filename = os.path.join(self.desktop_path, f"rescue_coordinates_{video_name}_{timestamp}.csv")
            with open(coords_filename, 'w', newline='') as f:
                f.write("Frame,Timestamp,Human_ID,Latitude,Longitude,Distance_m,Accuracy_m,Confidence\n")

                for frame_num, frame_data in self.frame_results.items():
                    if 'detections' in frame_data:
                        for detection in frame_data['detections']:
                            f.write(f"{frame_num},{frame_data['timestamp']:.2f},{detection['id']},"
                                    f"{detection['latitude']:.8f},{detection['longitude']:.8f},"
                                    f"{detection['distance_from_center_m']:.2f},"
                                    f"{detection['accuracy_estimate_m']:.2f},{detection['confidence']:.3f}\n")

        print(f"📄 Comprehensive flood rescue analysis saved to desktop:")
        print(f"   📊 Main report: {results_filename}")
        if self.all_detections:
            print(f"   📍 Rescue coordinates: {coords_filename}")

    def print_performance_summary(self, total_time: float, processed_frames: int):
        print(f"\n{'=' * 80}")
        print(f"🚁 FLOOD RESCUE MISSION COMPLETE - PERFORMANCE SUMMARY")
        print(f"{'=' * 80}")

        avg_fps = sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0
        avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0

        print(f"⚡ PERFORMANCE METRICS:")
        print(f"   Total processing time: {total_time:.2f}s")
        print(f"   Frames processed: {processed_frames}")
        print(f"   Average FPS achieved: {avg_fps:.2f} (Target: {self.target_fps})")
        print(
            f"   Performance rating: {'✅ EXCELLENT' if avg_fps >= self.target_fps else '⚠️ GOOD' if avg_fps >= self.target_fps * 0.8 else '❌ NEEDS OPTIMIZATION'}")
        print(f"   Average frame processing: {avg_processing_time * 1000:.1f}ms")
        print(f"   Raspberry Pi 5 optimization: {'✅ ACTIVE' if avg_fps >= 15 else '⚠️ NEEDS TUNING'}")

        if self.processing_times:
            print(f"   Fastest frame: {min(self.processing_times) * 1000:.1f}ms")
            print(f"   Slowest frame: {max(self.processing_times) * 1000:.1f}ms")

        print(f"\n🎯 DETECTION SUMMARY:")
        total_humans = len(self.all_detections)
        frames_with_detections = len([f for f in self.frame_results.values() if f['detections']])

        print(f"   Total humans detected: {total_humans}")
        print(f"   Frames with detections: {frames_with_detections}/{processed_frames}")
        if processed_frames > 0:
            print(f"   Detection rate: {(frames_with_detections / processed_frames * 100):.1f}%")

        if self.all_detections:
            distances = [d['distance_from_center_m'] for d in self.all_detections]
            accuracies = [d['accuracy_estimate_m'] for d in self.all_detections]
            confidences = [d['confidence'] for d in self.all_detections]

            print(f"\n📏 SPATIAL ANALYSIS:")
            print(f"   Closest human detected: {min(distances):.1f}m")
            print(f"   Average distance: {sum(distances) / len(distances):.1f}m")
            print(f"   Humans within 10m: {len([d for d in distances if d <= 10.0])}")
            print(f"   Humans within 20m: {len([d for d in distances if d <= 20.0])}")
            print(f"   Average positioning accuracy: ±{sum(accuracies) / len(accuracies):.1f}m")
            print(f"   Best accuracy achieved: ±{min(accuracies):.1f}m")

            print(f"\n📊 CONFIDENCE ANALYSIS:")
            print(f"   Average confidence: {sum(confidences) / len(confidences):.3f}")
            print(f"   High confidence detections (>0.7): {len([c for c in confidences if c > 0.7])}")
            print(f"   Very high confidence (>0.9): {len([c for c in confidences if c > 0.9])}")

        print(f"\n🚁 DRONE STATUS:")
        drone_lat, drone_lon, gps_acc = self.get_gps_coordinates_precise()
        print(f"   GPS Position: {drone_lat:.6f}, {drone_lon:.6f}")
        print(f"   GPS Accuracy: ±{gps_acc:.1f}m")
        print(f"   Operating Altitude: {self.altitude}m")
        print(f"   Camera FOV: {self.camera_fov}°")
        ground_coverage = 2 * self.altitude * math.tan(math.radians(self.camera_fov) / 2)
        print(
            f"   Ground Coverage: {ground_coverage:.1f}m × {ground_coverage * (self.frame_height / self.frame_width):.1f}m")

        print(f"\n💾 Output files saved to: {self.desktop_path}")

    def cleanup_video(self, video_writer=None):
        if self.video_cap:
            self.video_cap.release()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        print("✅ Video processing cleanup completed")


def main():
    parser = argparse.ArgumentParser(description='High-Precision Flood Rescue Drone Detection System')
    parser.add_argument('--model', '-m', required=True, help='Path to YOLOv11 ONNX model')
    parser.add_argument('--video', '-v', required=True, help='Path to video file')
    parser.add_argument('--altitude', '-a', type=float, default=8.0, help='Precise drone altitude in meters')
    parser.add_argument('--fov', '-f', type=float, default=84.0, help='Precise camera FOV in degrees')
    parser.add_argument('--confidence', type=float, default=0.1,
                        help='Confidence threshold (default: 0.1 for rescue ops)')
    parser.add_argument('--nms', type=float, default=0.1, help='NMS threshold (default: 0.1 for rescue ops)')
    parser.add_argument('--drone-lat', type=float, default=18.5204, help='Manual drone latitude')
    parser.add_argument('--drone-lon', type=float, default=73.8567, help='Manual drone longitude')
    parser.add_argument('--target-fps', type=float, default=20.0, help='Target FPS for RPi 5 (default: 20)')
    parser.add_argument('--desktop-path', default=r"C:\Users\Namit\Desktop\rew_ncnn",
                        help='Desktop output path')

    parser.add_argument('--no-display', action='store_true', help='Disable video display (better performance)')
    parser.add_argument('--save-results', action='store_true', help='Save comprehensive detection results')
    parser.add_argument('--save-video', action='store_true', help='Save annotated video to desktop')
    parser.add_argument('--debug', action='store_true', help='Enable detailed debug output for every frame')

    args = parser.parse_args()

    print(f"🚁 FLOOD RESCUE DRONE DETECTION SYSTEM")
    print(f"🎯 Optimized for Raspberry Pi 5 | Target FPS: {args.target_fps}")
    print(f"💾 Output directory: {args.desktop_path}")

    try:
        detector = HighPrecisionDroneDetectionSystem(
            model_path=args.model,
            altitude=args.altitude,
            camera_fov=args.fov,
            confidence_threshold=args.confidence,
            nms_threshold=args.nms,
            desktop_path=args.desktop_path
        )

        detector.drone_gps = (args.drone_lat, args.drone_lon)
        detector.target_fps = args.target_fps

        print(f"🎬 Starting high-performance video analysis...")
        print(f"📍 Drone GPS: {args.drone_lat}, {args.drone_lon}")
        print(f"🏔️  Altitude: {args.altitude}m | 📐 FOV: {args.fov}°")

        results = detector.process_video_high_performance(
            video_path=args.video,
            save_results=args.save_results,
            save_video=args.save_video,
            display=not args.no_display,
            debug=args.debug
        )

        print(f"\n🎉 MISSION ACCOMPLISHED!")
        if results:
            print(f"📊 Total detections logged: {len(results)}")
            distances = [r['distance_from_center_m'] for r in results]
            print(f"🎯 Closest human: {min(distances):.1f}m from drone center")
            print(f"📍 All coordinates saved to desktop for rescue coordination")
        else:
            print("📊 No humans were detected during the mission.")

    except Exception as e:
        print(f"❌ CRITICAL ERROR: {e}")
        print(f"🚨 Check system resources and model file")
        sys.exit(1)


if __name__ == "__main__":
    main()
