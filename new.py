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
from typing import List, Dict, Tuple, Optional
import sys
from collections import deque

# GPS imports
try:
    import serial

    GPS_SERIAL_AVAILABLE = True
except ImportError:
    GPS_SERIAL_AVAILABLE = False
    print("⚠️  Serial GPS module not available. Using mock GPS.")

# GPIO imports
try:
    from gpiozero import LED

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("⚠️  GPIO not available. Using mock GPIO.")

# Legacy GPS support
try:
    import gpsd

    GPS_DAEMON_AVAILABLE = True
except ImportError:
    GPS_DAEMON_AVAILABLE = False


class MockLED:
    """Mock LED class for systems without GPIO"""

    def __init__(self, pin):
        self.pin = pin
        self.is_on = False
        print(f"🔧 Mock LED initialized on pin {pin}")

    def on(self):
        self.is_on = True
        print("💡 Mock LED: ON")

    def off(self):
        self.is_on = False
        print("💡 Mock LED: OFF")


class TrackedHuman:
    """Class to represent a tracked human across frames"""

    def __init__(self, detection: Dict, frame_number: int, track_id: int):
        self.track_id = track_id
        self.first_seen_frame = frame_number
        self.last_seen_frame = frame_number
        self.total_frames_seen = 1
        self.positions = [detection['center']]
        self.gps_positions = [(detection.get('latitude', 0), detection.get('longitude', 0))]
        self.confidences = [detection['confidence']]
        self.bbox_sizes = [detection['bbox']['width'] * detection['bbox']['height']]
        self.is_active = True
        self.frames_missing = 0

        # Store the best detection (highest confidence)
        self.best_detection = detection.copy()

    def update(self, detection: Dict, frame_number: int):
        """Update tracked human with new detection"""
        self.last_seen_frame = frame_number
        self.total_frames_seen += 1
        self.positions.append(detection['center'])
        self.gps_positions.append((detection.get('latitude', 0), detection.get('longitude', 0)))
        self.confidences.append(detection['confidence'])
        self.bbox_sizes.append(detection['bbox']['width'] * detection['bbox']['height'])
        self.frames_missing = 0
        self.is_active = True

        # Update best detection if this one has higher confidence
        if detection['confidence'] > self.best_detection['confidence']:
            self.best_detection = detection.copy()
            self.best_detection['track_id'] = self.track_id

    def mark_missing(self):
        """Mark this track as missing for current frame"""
        self.frames_missing += 1
        if self.frames_missing > 30:  # Consider lost after 30 frames (~1.5 seconds at 20fps)
            self.is_active = False

    def get_predicted_position(self) -> Dict:
        """Get predicted position based on movement history"""
        if len(self.positions) < 2:
            return self.positions[-1]

        # Simple linear prediction based on last two positions
        last_pos = self.positions[-1]
        prev_pos = self.positions[-2]

        dx = last_pos['x'] - prev_pos['x']
        dy = last_pos['y'] - prev_pos['y']

        return {
            'x': last_pos['x'] + dx,
            'y': last_pos['y'] + dy
        }

    def get_average_position(self) -> Dict:
        """Get average position across all frames"""
        avg_x = sum(pos['x'] for pos in self.positions) / len(self.positions)
        avg_y = sum(pos['y'] for pos in self.positions) / len(self.positions)
        return {'x': int(avg_x), 'y': int(avg_y)}

    def get_statistics(self) -> Dict:
        """Get statistics for this tracked human"""
        return {
            'track_id': self.track_id,
            'first_seen_frame': self.first_seen_frame,
            'last_seen_frame': self.last_seen_frame,
            'total_frames_seen': self.total_frames_seen,
            'frames_tracked': self.last_seen_frame - self.first_seen_frame + 1,
            'detection_rate': self.total_frames_seen / (self.last_seen_frame - self.first_seen_frame + 1),
            'average_confidence': sum(self.confidences) / len(self.confidences),
            'max_confidence': max(self.confidences),
            'average_bbox_size': sum(self.bbox_sizes) / len(self.bbox_sizes),
            'movement_distance': self.calculate_movement_distance(),
            'best_detection': self.best_detection
        }

    def calculate_movement_distance(self) -> float:
        """Calculate total movement distance in pixels"""
        if len(self.positions) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(1, len(self.positions)):
            prev_pos = self.positions[i - 1]
            curr_pos = self.positions[i]
            distance = math.sqrt((curr_pos['x'] - prev_pos['x']) ** 2 + (curr_pos['y'] - prev_pos['y']) ** 2)
            total_distance += distance

        return total_distance


class HumanTracker:
    """Enhanced human tracking system to prevent duplicate counting"""

    def __init__(self, max_distance_threshold: float = 100.0, max_missing_frames: int = 30):
        self.tracked_humans = {}  # Dictionary of track_id -> TrackedHuman
        self.next_track_id = 1
        self.max_distance_threshold = max_distance_threshold
        self.max_missing_frames = max_missing_frames
        self.frame_number = 0

    def calculate_distance(self, pos1: Dict, pos2: Dict) -> float:
        """Calculate Euclidean distance between two positions"""
        return math.sqrt((pos1['x'] - pos2['x']) ** 2 + (pos1['y'] - pos2['y']) ** 2)

    def calculate_iou(self, bbox1: Dict, bbox2: Dict) -> float:
        """Calculate Intersection over Union (IoU) between two bounding boxes"""
        x1_min, y1_min = bbox1['x'], bbox1['y']
        x1_max, y1_max = x1_min + bbox1['width'], y1_min + bbox1['height']

        x2_min, y2_min = bbox2['x'], bbox2['y']
        x2_max, y2_max = x2_min + bbox2['width'], y2_min + bbox2['height']

        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)

        if inter_x_min >= inter_x_max or inter_y_min >= inter_y_max:
            return 0.0

        intersection = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)

        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)
        union = area1 + area2 - intersection

        return intersection / union if union > 0 else 0.0

    def match_detections_to_tracks(self, detections: List[Dict]) -> List[Tuple[int, int]]:
        """Match current detections to existing tracks using Hungarian algorithm approximation"""
        if not detections or not self.tracked_humans:
            return []

        active_tracks = {tid: track for tid, track in self.tracked_humans.items() if track.is_active}
        if not active_tracks:
            return []

        matches = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(active_tracks.keys())

        for det_idx, detection in enumerate(detections):
            if det_idx not in unmatched_detections:
                continue

            best_match_track = None
            best_score = float('inf')

            for track_id in unmatched_tracks:
                track = active_tracks[track_id]
                predicted_pos = track.get_predicted_position()

                distance = self.calculate_distance(detection['center'], predicted_pos)

                if hasattr(track, 'best_detection') and 'bbox' in track.best_detection:
                    iou = self.calculate_iou(detection['bbox'], track.best_detection['bbox'])
                    score = distance * (1.0 - iou * 0.5)
                else:
                    score = distance

                if score < best_score and distance < self.max_distance_threshold:
                    best_score = score
                    best_match_track = track_id

            if best_match_track is not None:
                matches.append((det_idx, best_match_track))
                unmatched_detections.remove(det_idx)
                unmatched_tracks.remove(best_match_track)

        return matches

    def update(self, detections: List[Dict], frame_number: int) -> List[Dict]:
        """Update tracker with new detections and return tracked detections"""
        self.frame_number = frame_number

        matches = self.match_detections_to_tracks(detections)

        matched_detection_indices = set()
        matched_track_ids = set()

        for det_idx, track_id in matches:
            self.tracked_humans[track_id].update(detections[det_idx], frame_number)
            matched_detection_indices.add(det_idx)
            matched_track_ids.add(track_id)

        for det_idx, detection in enumerate(detections):
            if det_idx not in matched_detection_indices:
                new_track = TrackedHuman(detection, frame_number, self.next_track_id)
                self.tracked_humans[self.next_track_id] = new_track
                matched_track_ids.add(self.next_track_id)
                self.next_track_id += 1

        for track_id, track in self.tracked_humans.items():
            if track_id not in matched_track_ids and track.is_active:
                track.mark_missing()

        tracked_detections = []
        for det_idx, detection in enumerate(detections):
            track_id = None
            for match_det_idx, match_track_id in matches:
                if match_det_idx == det_idx:
                    track_id = match_track_id
                    break

            if track_id is None:
                for tid, track in self.tracked_humans.items():
                    if track.first_seen_frame == frame_number and track.total_frames_seen == 1:
                        if (track.positions[0]['x'] == detection['center']['x'] and
                                track.positions[0]['y'] == detection['center']['y']):
                            track_id = tid
                            break

            if track_id is not None:
                enhanced_detection = detection.copy()
                enhanced_detection['track_id'] = track_id
                enhanced_detection['first_seen_frame'] = self.tracked_humans[track_id].first_seen_frame
                enhanced_detection['total_frames_seen'] = self.tracked_humans[track_id].total_frames_seen
                tracked_detections.append(enhanced_detection)

        return tracked_detections

    def get_unique_humans_count(self) -> int:
        """Get count of unique humans ever detected"""
        return len(self.tracked_humans)

    def get_active_humans_count(self) -> int:
        """Get count of currently active/visible humans"""
        return len([track for track in self.tracked_humans.values() if track.is_active])

    def get_tracking_statistics(self) -> Dict:
        """Get comprehensive tracking statistics"""
        total_tracks = len(self.tracked_humans)
        active_tracks = len([track for track in self.tracked_humans.values() if track.is_active])

        if total_tracks == 0:
            return {
                'total_unique_humans': 0,
                'currently_active': 0,
                'average_track_length': 0,
                'longest_track': 0,
                'best_tracks': []
            }

        track_lengths = [track.total_frames_seen for track in self.tracked_humans.values()]
        track_confidences = [track.get_statistics()['average_confidence'] for track in self.tracked_humans.values()]

        best_tracks = []
        for track in self.tracked_humans.values():
            stats = track.get_statistics()
            if stats['total_frames_seen'] >= 5:
                best_tracks.append(stats)

        best_tracks.sort(key=lambda x: (x['total_frames_seen'], x['average_confidence']), reverse=True)

        return {
            'total_unique_humans': total_tracks,
            'currently_active': active_tracks,
            'average_track_length': sum(track_lengths) / len(track_lengths),
            'longest_track': max(track_lengths),
            'shortest_track': min(track_lengths),
            'average_confidence': sum(track_confidences) / len(track_confidences),
            'best_tracks': best_tracks[:10]
        }


class GPSHandler:
    """Handles both serial GPS and daemon GPS with fallback to mock"""

    def __init__(self, serial_port="/dev/serial0", baudrate=9600):
        self.serial_connection = None
        self.daemon_connected = False
        self.mock_mode = False
        self.last_coords = (0.0, 0.0)
        self.setup_gps(serial_port, baudrate)

    def setup_gps(self, serial_port, baudrate):
        """Setup GPS connection with multiple fallback options"""
        if GPS_SERIAL_AVAILABLE:
            try:
                self.serial_connection = serial.Serial(serial_port, baudrate=baudrate, timeout=1)
                print(f"✅ Serial GPS connected on {serial_port}")
                return
            except Exception as e:
                print(f"⚠️  Serial GPS connection failed: {e}")

        if GPS_DAEMON_AVAILABLE:
            try:
                import gpsd
                gpsd.connect()
                self.daemon_connected = True
                print("✅ GPS daemon connected")
                return
            except Exception as e:
                print(f"⚠️  GPS daemon connection failed: {e}")

        self.mock_mode = True
        print("🔧 Using mock GPS coordinates")

    def parse_gga(self, sentence):
        """Parse NMEA GGA sentence"""
        parts = sentence.split(',')
        if len(parts) < 15:
            return None

        fix_quality = parts[6]
        if fix_quality not in ("1", "2"):
            return None

        lat_raw = parts[2]
        lat_dir = parts[3]
        lon_raw = parts[4]
        lon_dir = parts[5]

        if not lat_raw or not lon_raw:
            return None

        lat_deg = float(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lat = lat_deg + (lat_min / 60.0)
        if lat_dir == "S":
            lat = -lat

        lon_deg = float(lon_raw[:3])
        lon_min = float(lon_raw[3:])
        lon = lon_deg + (lon_min / 60.0)
        if lon_dir == "W":
            lon = -lon

        return lat, lon

    def get_coordinates(self, fallback_coords=(18.5204, 73.8567)) -> Tuple[float, float, float]:
        """Get GPS coordinates with accuracy estimate"""
        if self.serial_connection:
            try:
                line = self.serial_connection.readline().decode(errors="ignore").strip()
                if line.startswith("$GPGGA"):
                    coords = self.parse_gga(line)
                    if coords:
                        self.last_coords = coords
                        return (*coords, 3.0)
            except Exception as e:
                print(f"⚠️  Serial GPS read error: {e}")

        if self.daemon_connected:
            try:
                import gpsd
                packet = gpsd.get_current()
                if packet.lat != 0 and packet.lon != 0:
                    coords = (packet.lat, packet.lon)
                    self.last_coords = coords
                    accuracy = getattr(packet, 'error', {}).get('x', 5.0)
                    return (*coords, accuracy)
            except Exception as e:
                print(f"⚠️  Daemon GPS read error: {e}")

        if self.mock_mode:
            base_lat, base_lon = fallback_coords
            time_factor = time.time() * 0.001
            lat_offset = math.sin(time_factor) * 0.00001
            lon_offset = math.cos(time_factor) * 0.00001
            mock_coords = (base_lat + lat_offset, base_lon + lon_offset)
            return (*mock_coords, 5.0)

        return (*fallback_coords, 10.0)


class HighPrecisionDroneDetectionSystem:
    def __init__(self,
                 model_path: str,
                 altitude: float = 8.0,
                 camera_fov: float = 84.0,
                 confidence_threshold: float = 0.3,
                 nms_threshold: float = 0.3,
                 desktop_path: str = "C:\\Users\\Namit\\Desktop\\rew_ncnn",
                 gpio_pin: int = 17):
        self.model_path = model_path
        self.altitude = altitude
        self.camera_fov = camera_fov
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.desktop_path = desktop_path

        os.makedirs(desktop_path, exist_ok=True)

        self.tracker = HumanTracker(max_distance_threshold=80.0, max_missing_frames=30)
        self.gps_handler = GPSHandler()

        self.gpio_pin = gpio_pin
        if GPIO_AVAILABLE:
            try:
                self.led = LED(gpio_pin)
                self.gpio_mock = False
                print(f"✅ GPIO LED initialized on pin {gpio_pin}")
            except Exception as e:
                print(f"⚠️  GPIO initialization failed: {e}")
                self.led = MockLED(gpio_pin)
                self.gpio_mock = True
        else:
            self.led = MockLED(gpio_pin)
            self.gpio_mock = True

        self.led_state = False
        self.humans_detected_current_frame = False

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
        self.processing_times = deque(maxlen=100)
        self.fps_history = deque(maxlen=30)
        self.target_fps = 20.0
        self.frame_skip_adaptive = False
        self.distance_corrections = []
        self.gps_precision_history = []

    def update_led_status(self, humans_detected: bool):
        """Update LED status based on human detection"""
        if humans_detected != self.led_state:
            if humans_detected:
                self.led.on()
                self.led_state = True
            else:
                self.led.off()
                self.led_state = False

    def get_gps_coordinates_precise(self) -> Tuple[float, float, float]:
        """Get precise GPS coordinates using the integrated GPS handler"""
        return self.gps_handler.get_coordinates(self.drone_gps)

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

            if 'track_id' in detection:
                result['track_id'] = detection['track_id']
                result['first_seen_frame'] = detection['first_seen_frame']
                result['total_frames_seen'] = detection['total_frames_seen']

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
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
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
            track_id = result.get('track_id', 'N/A')
            frames_seen = result.get('total_frames_seen', 1)

            if frames_seen >= 10:
                color = (0, 255, 0)
            elif frames_seen >= 5:
                color = (0, 255, 255)
            else:
                color = (0, 165, 255)
            if distance > 20.0:
                color = (0, 0, 255)

            thickness = min(5, max(2, frames_seen // 3))
            cv2.rectangle(frame, (bbox['x'], bbox['y']),
                          (bbox['x'] + bbox['width'], bbox['y'] + bbox['height']),
                          color, thickness)
            cv2.circle(frame, (x, y), 12, color, -1)
            cv2.circle(frame, (x, y), 15, (255, 255, 255), 2)
            cv2.putText(frame, f"#{track_id}", (x - 10, y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.line(frame, (frame_center_x, frame_center_y), (x, y), color, 2)

            if distance > 5.0:
                steps = int(distance / 5.0)
                for step in range(1, steps + 1):
                    marker_x = int(frame_center_x + (x - frame_center_x) * (step * 5.0 / distance))
                    marker_y = int(frame_center_y + (y - frame_center_y) * (step * 5.0 / distance))
                    cv2.circle(frame, (marker_x, marker_y), 4, (255, 255, 255), -1)

            info_lines = [
                f"TRACK #{track_id}",
                f"FRAMES: {frames_seen}",
                f"DIST: {distance:.1f}m (±{accuracy:.1f}m)",
                f"GPS: {result['latitude']:.6f}",
                f"     {result['longitude']:.6f}",
                f"CONF: {result['confidence']:.3f}"
            ]

            for i, line in enumerate(info_lines):
                y_offset = y - 80 + (i * 18)
                text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0]
                cv2.rectangle(frame, (x + 15, y_offset - 12),
                              (x + 20 + text_size[0], y_offset + 6), (0, 0, 0), -1)
                text_color = (255, 255, 255) if i != 0 else color
                cv2.putText(frame, line, (x + 18, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1)

        tracking_stats = self.tracker.get_tracking_statistics()
        status_bg_height = 280
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (650, status_bg_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

        drone_lat, drone_lon, gps_acc = self.get_gps_coordinates_precise()
        gps_status = "REAL GPS" if not self.gps_handler.mock_mode else "MOCK GPS"
        gps_color = (0, 255, 0) if not self.gps_handler.mock_mode else (0, 255, 255)
        gpio_status = f"GPIO-{self.gpio_pin}" if not self.gpio_mock else f"MOCK-{self.gpio_pin}"
        gpio_color = (0, 255, 0) if not self.gpio_mock else (0, 255, 255)
        led_status = "LED-ON" if self.led_state else "LED-OFF"
        led_color = (0, 255, 0) if self.led_state else (0, 0, 255)

        status_info = [
            f"FLOOD RESCUE DETECTION SYSTEM - WITH TRACKING",
            f"Frame: {frame_number}/{self.total_frames} | Time: {timestamp:.1f}s",
            f"FPS: {current_fps:.1f} | Current Detections: {len(results)}",
            f"🎯 UNIQUE HUMANS: {tracking_stats['total_unique_humans']} | Active: {tracking_stats['currently_active']}",
            f"📊 Avg Track Length: {tracking_stats['average_track_length']:.1f} frames",
            f"Altitude: {self.altitude}m | FOV: {self.camera_fov}°",
            f"GPS: {drone_lat:.6f}, {drone_lon:.6f} (±{gps_acc:.1f}m)",
            f"Confidence: {self.confidence_threshold} | NMS: {self.nms_threshold}",
            f"Target FPS: {self.target_fps} | Processing: RPi 5 Optimized"
        ]

        for i, text in enumerate(status_info):
            y_pos = 25 + (i * 24)
            font_scale = 0.8 if i == 0 else 0.6
            color = (0, 255, 0) if i == 3 else (0, 255, 255) if i == 0 else (255, 255, 255)
            thickness = 2 if i == 0 else 1
            cv2.putText(frame, text, (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

        status_y_start = H - 80
        cv2.putText(frame, gps_status, (W - 150, status_y_start),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, gps_color, 2)
        cv2.putText(frame, gpio_status, (W - 150, status_y_start + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, gpio_color, 2)
        cv2.putText(frame, led_status, (W - 150, status_y_start + 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, led_color, 2)

        return frame

    def process_frame_optimized(self, frame: np.ndarray, frame_number: int) -> List[Dict]:
        input_tensor = self.preprocess_frame_optimized(frame)
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
        detections = self.postprocess_detections_optimized(outputs, frame.shape)
        tracked_detections = self.tracker.update(detections, frame_number)
        results = self.calculate_precise_gps_coordinates(tracked_detections, frame.shape)
        self.humans_detected_current_frame = len(results) > 0
        self.update_led_status(self.humans_detected_current_frame)
        return results

    def print_inference_results(self, results: List[Dict], frame_number: int,
                                timestamp: float, processing_time: float, current_fps: float):
        tracking_stats = self.tracker.get_tracking_statistics()
        print(f"\n{'=' * 80}")
        print(f"🚁 FRAME {frame_number} | TIME: {timestamp:.2f}s | FPS: {current_fps:.1f}")
        print(f"⚡ Processing: {processing_time * 1000:.1f}ms | Target FPS: {self.target_fps}")
        gps_status = "REAL" if not self.gps_handler.mock_mode else "MOCK"
        gpio_status = "REAL" if not self.gpio_mock else "MOCK"
        led_status = "ON" if self.led_state else "OFF"
        print(f"🔧 Hardware: GPS-{gps_status} | GPIO-{gpio_status} | LED-{led_status}")
        print(
            f"🎯 TRACKING: {tracking_stats['total_unique_humans']} UNIQUE HUMANS | {tracking_stats['currently_active']} ACTIVE")
        print(f"📊 Average track length: {tracking_stats['average_track_length']:.1f} frames")

        if results:
            print(f"🎯 CURRENT FRAME: {len(results)} DETECTION(S) - FLOOD RESCUE COORDINATES:")
            print(
                f"{'TRK':<4} {'FRSN':<5} {'DISTANCE':<12} {'GPS LATITUDE':<12} {'GPS LONGITUDE':<13} {'ACCURACY':<10} {'CONFIDENCE':<10}")
            print(f"{'-' * 4} {'-' * 5} {'-' * 12} {'-' * 12} {'-' * 13} {'-' * 10} {'-' * 10}")

            for result in results:
                track_id = result.get('track_id', 'N/A')
                frames_seen = result.get('total_frames_seen', 1)
                distance_str = f"{result['distance_from_center_m']:.1f}m"
                lat_str = f"{result['latitude']:.6f}"
                lon_str = f"{result['longitude']:.6f}"
                accuracy_str = f"±{result['accuracy_estimate_m']:.1f}m"
                confidence_str = f"{result['confidence']:.3f}"
                print(
                    f"{track_id:<4} {frames_seen:<5} {distance_str:<12} {lat_str:<12} {lon_str:<13} {accuracy_str:<10} {confidence_str:<10}")

            distances = [r['distance_from_center_m'] for r in results]
            accuracies = [r['accuracy_estimate_m'] for r in results]
            confidences = [r['confidence'] for r in results]
            print(f"\n📊 CURRENT FRAME SUMMARY:")
            print(f"   Closest human: {min(distances):.1f}m")
            print(f"   Average distance: {sum(distances) / len(distances):.1f}m")
            print(f"   Average accuracy: ±{sum(accuracies) / len(accuracies):.1f}m")
            print(f"   Average confidence: {sum(confidences) / len(confidences):.3f}")
            print(f"   💡 LED Status: {'🟢 ALERT ON' if self.led_state else '🔴 ALERT OFF'}")
        else:
            print("🔍 NO HUMANS DETECTED IN CURRENT FRAME")

        drone_lat, drone_lon, gps_acc = self.get_gps_coordinates_precise()
        print(f"📍 Drone GPS: {drone_lat:.6f}, {drone_lon:.6f} (±{gps_acc:.1f}m)")

        if frame_number % 100 == 0 and tracking_stats['best_tracks']:
            print(f"\n🏆 TOP TRACKED HUMANS:")
            for i, track in enumerate(tracking_stats['best_tracks'][:3]):
                print(
                    f"   #{track['track_id']}: {track['total_frames_seen']} frames, conf {track['average_confidence']:.3f}")

    def process_video_high_performance(self,
                                       video_path: str,
                                       save_results: bool = False,
                                       save_video: bool = False,
                                       display: bool = True,
                                       debug: bool = False) -> List[Dict]:
        print(f"🎬 HIGH-PERFORMANCE PROCESSING WITH TRACKING: {video_path}")
        print(f"🎯 Target FPS: {self.target_fps} | Optimized for Raspberry Pi 5")
        self.setup_video_optimized(video_path)

        video_writer = None
        if save_video:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_name = os.path.splitext(os.path.basename(video_path))[0]
            output_video_path = os.path.join(self.desktop_path, f"flood_rescue_tracked_{video_name}_{timestamp}.mp4")
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
                results = self.process_frame_optimized(frame, frame_number)
                processing_time = time.time() - frame_start_time
                self.processing_times.append(processing_time)
                current_fps = 1.0 / processing_time if processing_time > 0 else 0
                self.fps_history.append(current_fps)

                tracking_stats = self.tracker.get_tracking_statistics()
                frame_data = {
                    'frame_number': frame_number,
                    'timestamp': timestamp,
                    'detections': results,
                    'processing_time': processing_time,
                    'fps': current_fps,
                    'led_status': self.led_state,
                    'tracking_stats': tracking_stats,
                    'hardware_status': {
                        'gps_real': not self.gps_handler.mock_mode,
                        'gpio_real': not self.gpio_mock
                    }
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
                        cv2.imshow('FLOOD RESCUE - Human Detection with Tracking', display_frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            print("\n🛑 Stopping video processing...")
                            break
                        elif key == ord('s'):
                            self.save_current_detections(frame_number, results, tracking_stats)
                        elif key == ord('l'):
                            self.update_led_status(not self.led_state)
                            print(f"💡 Manual LED toggle: {'ON' if self.led_state else 'OFF'}")
                        elif key == ord('t'):
                            print(f"\n📊 TRACKING STATS: {tracking_stats}")
                    if save_video and video_writer:
                        video_writer.write(annotated_frame)

        except KeyboardInterrupt:
            print("\n🛑 Emergency stop - Video processing interrupted")
        finally:
            self.led.off()
            self.cleanup_video(video_writer)

        total_time = time.time() - start_time
        self.print_performance_summary(total_time, frame_number)
        if save_results:
            self.save_comprehensive_results(video_path)

        return self.all_detections

    def save_current_detections(self, frame_number: int, results: List[Dict], tracking_stats: Dict):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.desktop_path,
                                f"emergency_detections_tracked_frame_{frame_number}_{timestamp}.json")
        save_data = {
            'emergency_save': True,
            'frame_number': frame_number,
            'timestamp': timestamp,
            'current_detections': results,
            'tracking_statistics': tracking_stats,
            'drone_info': {
                'gps': self.get_gps_coordinates_precise(),
                'altitude': self.altitude,
                'camera_fov': self.camera_fov
            },
            'hardware_status': {
                'gps_real': not self.gps_handler.mock_mode,
                'gpio_real': not self.gpio_mock,
                'led_state': self.led_state
            }
        }
        with open(filename, 'w') as f:
            json.dump(save_data, f, indent=2)
        print(f"🚨 EMERGENCY SAVE: Frame {frame_number} detections and tracking data saved to desktop")

    def save_comprehensive_results(self, video_path: str):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_name = os.path.splitext(os.path.basename(video_path))[0]
        results_filename = os.path.join(self.desktop_path,
                                        f"flood_rescue_tracked_analysis_{video_name}_{timestamp}.json")
        final_tracking_stats = self.tracker.get_tracking_statistics()
        total_frame_detections = len(self.all_detections)
        unique_humans = final_tracking_stats['total_unique_humans']
        frames_with_humans = len([f for f in self.frame_results.values() if f['detections']])
        avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
        avg_fps = sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0
        frames_with_led_on = len([f for f in self.frame_results.values() if f.get('led_status', False)])
        detection_reduction = ((
                                           total_frame_detections - unique_humans) / total_frame_detections * 100) if total_frame_detections > 0 else 0

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
                'mission_status': 'COMPLETED' if unique_humans > 0 else 'NO_HUMANS_FOUND',
                'tracking_enabled': True
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
            'tracking_configuration': {
                'max_distance_threshold': self.tracker.max_distance_threshold,
                'max_missing_frames': self.tracker.max_missing_frames,
                'tracking_algorithm': 'Enhanced Multi-Object Tracking with IoU'
            },
            'hardware_configuration': {
                'gps_mode': 'REAL' if not self.gps_handler.mock_mode else 'MOCK',
                'gpio_mode': 'REAL' if not self.gpio_mock else 'MOCK',
                'gpio_pin': self.gpio_pin,
                'led_activations': frames_with_led_on,
                'led_activation_rate': frames_with_led_on / len(self.frame_results) if self.frame_results else 0
            },
            'performance_metrics': {
                'target_fps': self.target_fps,
                'average_fps_achieved': avg_fps,
                'average_processing_time_ms': avg_processing_time * 1000,
                'performance_rating': 'EXCELLENT' if avg_fps >= self.target_fps else 'GOOD' if avg_fps >= self.target_fps * 0.8 else 'NEEDS_OPTIMIZATION'
            },
            'rescue_statistics': {
                'total_frame_detections': total_frame_detections,
                'unique_humans_identified': unique_humans,
                'detection_reduction_percentage': detection_reduction,
                'frames_with_detections': frames_with_humans,
                'detection_rate': frames_with_humans / len(self.frame_results) if self.frame_results else 0,
                'gps_coverage_area': gps_bounds,
                'distance_analysis': distance_stats,
                'accuracy_analysis': accuracy_stats
            },
            'tracking_analysis': final_tracking_stats,
            'drone_telemetry': {
                'gps_coordinates': self.get_gps_coordinates_precise(),
                'altitude_m': self.altitude,
                'camera_specifications': {
                    'fov_degrees': self.camera_fov,
                    'ground_coverage_m': 2 * self.altitude * math.tan(math.radians(self.camera_fov) / 2)
                }
            },
            'detailed_frame_results': self.frame_results,
            'unique_human_tracks': {tid: track.get_statistics() for tid, track in self.tracker.tracked_humans.items()}
        }

        with open(results_filename, 'w') as f:
            json.dump(comprehensive_data, f, indent=2)

        if unique_humans > 0:
            coords_filename = os.path.join(self.desktop_path, f"unique_humans_coordinates_{video_name}_{timestamp}.csv")
            with open(coords_filename, 'w', newline='') as f:
                f.write(
                    "Track_ID,First_Frame,Last_Frame,Total_Frames_Seen,Avg_Confidence,Best_Latitude,Best_Longitude,Best_Distance_m,Best_Accuracy_m\n")
                for track_id, track in self.tracker.tracked_humans.items():
                    stats = track.get_statistics()
                    best_det = stats['best_detection']
                    f.write(f"{track_id},{stats['first_seen_frame']},{stats['last_seen_frame']},"
                            f"{stats['total_frames_seen']},{stats['average_confidence']:.3f},"
                            f"{best_det.get('latitude', 0):.8f},{best_det.get('longitude', 0):.8f},"
                            f"{best_det.get('distance_from_center_m', 0):.2f},"
                            f"{best_det.get('accuracy_estimate_m', 0):.2f}\n")

        print(f"📄 Comprehensive flood rescue analysis with tracking saved to desktop:")
        print(f"   📊 Main report: {results_filename}")
        if unique_humans > 0:
            print(f"   📍 Unique human coordinates: {coords_filename}")

    def print_performance_summary(self, total_time: float, processed_frames: int):
        tracking_stats = self.tracker.get_tracking_statistics()
        total_frame_detections = len(self.all_detections)
        unique_humans = tracking_stats['total_unique_humans']
        print(f"\n{'=' * 80}")
        print(f"🚁 FLOOD RESCUE MISSION COMPLETE - TRACKING-ENHANCED SUMMARY")
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
        print(f"\n🔧 HARDWARE STATUS:")
        gps_status = "REAL GPS" if not self.gps_handler.mock_mode else "MOCK GPS"
        gpio_status = f"REAL GPIO (Pin {self.gpio_pin})" if not self.gpio_mock else f"MOCK GPIO (Pin {self.gpio_pin})"
        print(f"   GPS Mode: {gps_status}")
        print(f"   GPIO Mode: {gpio_status}")
        frames_with_led = len([f for f in self.frame_results.values() if f.get('led_status', False)])
        print(f"   LED Activations: {frames_with_led}/{processed_frames} frames")
        if processed_frames > 0:
            print(f"   LED Activation Rate: {(frames_with_led / processed_frames * 100):.1f}%")
        print(f"\n🎯 DETECTION SUMMARY:")
        print(f"   Total frame detections: {total_frame_detections}")
        print(f"   Unique humans identified: {unique_humans}")
        detection_reduction = ((
                                           total_frame_detections - unique_humans) / total_frame_detections * 100) if total_frame_detections > 0 else 0
        print(f"   Ghosting reduction: {detection_reduction:.1f}%")
        frames_with_detections = len([f for f in self.frame_results.values() if f['detections']])
        print(f"   Frames with detections: {frames_with_detections}/{processed_frames}")
        if processed_frames > 0:
            print(f"   Detection rate: {(frames_with_detections / processed_frames * 100):.1f}%")
        if unique_humans > 0:
            distances = [track.get_statistics()['best_detection']['distance_from_center_m'] for track in
                         self.tracker.tracked_humans.values()]
            accuracies = [track.get_statistics()['best_detection']['accuracy_estimate_m'] for track in
                          self.tracker.tracked_humans.values()]
            confidences = [track.get_statistics()['average_confidence'] for track in
                           self.tracker.tracked_humans.values()]
            print(f"\n📏 SPATIAL ANALYSIS (UNIQUE HUMANS):")
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
        try:
            self.led.off()
            print("💡 LED turned off during cleanup")
        except:
            pass
        if self.video_cap:
            self.video_cap.release()
        if video_writer:
            video_writer.release()
        cv2.destroyAllWindows()
        print("✅ Video processing cleanup completed")

    def __del__(self):
        try:
            if hasattr(self, 'led'):
                self.led.off()
        except:
            pass


def main():
    parser = argparse.ArgumentParser(
        description='Enhanced High-Precision Flood Rescue Drone Detection System with Tracking')
    parser.add_argument('--model', '-m', required=True, help='Path to YOLOv11 ONNX model')
    parser.add_argument('--video', '-v', required=True, help='Path to video file')
    parser.add_argument('--altitude', '-a', type=float, default=8.0, help='Precise drone altitude in meters')
    parser.add_argument('--fov', '-f', type=float, default=84.0, help='Precise camera FOV in degrees')
    parser.add_argument('--confidence', type=float, default=0.1, help='Confidence threshold')
    parser.add_argument('--nms', type=float, default=0.1, help='NMS threshold')
    parser.add_argument('--drone-lat', type=float, default=18.5204, help='Manual drone latitude')
    parser.add_argument('--drone-lon', type=float, default=73.8567, help='Manual drone longitude')
    parser.add_argument('--target-fps', type=float, default=20.0, help='Target FPS for RPi 5')
    parser.add_argument('--desktop-path', default=r"C:\Users\Namit\Desktop\rew_ncnn",
                        help='Desktop output path')
    parser.add_argument('--gpio-pin', type=int, default=17, help='GPIO pin for LED')
    parser.add_argument('--no-display', action='store_true', help='Disable video display')
    parser.add_argument('--save-results', action='store_true', help='Save comprehensive detection results')
    parser.add_argument('--save-video', action='store_true', help='Save annotated video to desktop')
    parser.add_argument('--debug', action='store_true', help='Enable detailed debug output')

    args = parser.parse_args()
    print(f"🚁 ENHANCED FLOOD RESCUE DRONE DETECTION SYSTEM WITH TRACKING")
    print(f"🎯 Optimized for Raspberry Pi 5 | Target FPS: {args.target_fps}")
    print(f"🔧 GPIO Pin: {args.gpio_pin} | Hardware Integration: GPS + GPIO")
    print(f"💾 Output directory: {args.desktop_path}")

    try:
        detector = HighPrecisionDroneDetectionSystem(
            model_path=args.model,
            altitude=args.altitude,
            camera_fov=args.fov,
            confidence_threshold=args.confidence,
            nms_threshold=args.nms,
            desktop_path=args.desktop_path,
            gpio_pin=args.gpio_pin
        )
        detector.drone_gps = (args.drone_lat, args.drone_lon)
        detector.target_fps = args.target_fps
        print(f"🎬 Starting enhanced high-performance video analysis...")
        print(f"📍 Drone GPS: {args.drone_lat}, {args.drone_lon}")
        print(f"🏔️  Altitude: {args.altitude}m | 📐 FOV: {args.fov}°")
        print(f"💡 LED Alert System: {'Real GPIO' if GPIO_AVAILABLE else 'Mock GPIO'}")
        print(f"🛰️  GPS System: {'Real GPS' if GPS_SERIAL_AVAILABLE or GPS_DAEMON_AVAILABLE else 'Mock GPS'}")
        print(f"⌨️  Controls: 'q' = quit, 's' = save current detections, 'l' = toggle LED, 't' = show tracking stats")
        results = detector.process_video_high_performance(
            video_path=args.video,
            save_results=args.save_results,
            save_video=args.save_video,
            display=not args.no_display,
            debug=args.debug
        )
        tracking_stats = detector.tracker.get_tracking_statistics()
        print(f"\n🎉 MISSION ACCOMPLISHED!")
        if tracking_stats['total_unique_humans'] > 0:
            unique_humans = tracking_stats['total_unique_humans']
            distances = [track.get_statistics()['best_detection']['distance_from_center_m'] for track in
                         detector.tracker.tracked_humans.values()]
            print(f"📊 Unique humans identified: {unique_humans}")
            print(f"🎯 Closest human: {min(distances):.1f}m from drone center")
            print(f"📍 All unique coordinates saved to desktop for rescue coordination")
            print(f"💡 LED activations correlated with human detections")
        else:
            print("📊 No humans were detected during the mission.")
            print("💡 LED remained off throughout the mission.")
    except Exception as e:
        print(f"❌ CRITICAL ERROR: {e}")
        print(f"🚨 Check system resources, model file, and hardware connections")
        sys.exit(1)


if __name__ == "__main__":
    main()
