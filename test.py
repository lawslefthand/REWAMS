import cv2
import numpy as np
from typing import List, Dict

class DroneOverlay:
    def __init__(self, altitude=10, fov=130, confidence=0.4, target_fps=20):
        self.altitude = altitude
        self.fov = fov
        self.confidence_threshold = confidence
        self.target_fps = target_fps
        self.total_frames = 0  # Optional, can be updated elsewhere

    def draw_enhanced_detections(
        self, frame: np.ndarray, results: List[Dict],
        frame_number: int = 0, timestamp: float = 0.0,
        current_fps: float = 0.0
    ) -> np.ndarray:

        H, W = frame.shape[:2]
        frame_center_x, frame_center_y = W // 2, H // 2

        # Draw center crosshair
        cv2.line(frame, (frame_center_x - 20, frame_center_y), (frame_center_x + 20, frame_center_y), (0, 255, 255), 2)
        cv2.line(frame, (frame_center_x, frame_center_y - 20), (frame_center_x, frame_center_y + 20), (0, 255, 255), 2)
        cv2.circle(frame, (frame_center_x, frame_center_y), 4, (0, 255, 255), -1)
        cv2.putText(frame, "DRONE CENTER", (frame_center_x + 25, frame_center_y - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Draw detections
        for result in results:
            x, y = result['pixel_coordinates']['x'], result['pixel_coordinates']['y']
            bbox = result['bbox']
            distance = result['distance_from_center_m']
            accuracy = result['accuracy_estimate_m']

            # Color coding by distance
            if distance < 5.0:
                color = (0, 255, 0)     # green
            elif distance < 15.0:
                color = (0, 255, 255)   # yellow
            else:
                color = (0, 0, 255)     # red

            # Bounding box
            cv2.rectangle(frame, (bbox['x'], bbox['y']),
                          (bbox['x'] + bbox['width'], bbox['y'] + bbox['height']),
                          color, 2)

            # Target dot
            cv2.circle(frame, (x, y), 10, color, -1)
            cv2.circle(frame, (x, y), 13, (255, 255, 255), 1)

            # Line from drone center to detection
            cv2.line(frame, (frame_center_x, frame_center_y), (x, y), color, 2)

            # Optional step markers every 5m
            if distance > 5.0:
                steps = int(distance / 5.0)
                for step in range(1, steps + 1):
                    marker_x = int(frame_center_x + (x - frame_center_x) * (step * 5.0 / distance))
                    marker_y = int(frame_center_y + (y - frame_center_y) * (step * 5.0 / distance))
                    cv2.circle(frame, (marker_x, marker_y), 3, (255, 255, 255), -1)

            # Info text near detection
            info_lines = [
                f"HUMAN {result['id']}",
                f"DIST: {distance:.1f}m (+/-{accuracy:.1f}m)",
                f"GPS: {result['latitude']:.6f}",
                f"     {result['longitude']:.6f}",
                f"CONF: {result['confidence']:.2f}"
            ]

            for i, line in enumerate(info_lines):
                y_offset = y - 60 + (i * 18)
                text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0]
                cv2.rectangle(frame, (x + 15, y_offset - 14),
                              (x + 20 + text_size[0], y_offset + 4), (0, 0, 0), -1)
                cv2.putText(frame, line, (x + 18, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        # Transparent black status box (smaller now)
        status_bg_height = 80
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (300, status_bg_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Status text
        status_info = [
            f"FPS: {current_fps:.1f} | Humans: {len(results)}",
            f"Altitude: {self.altitude}m | FOV: {self.fov} deg",
            f"Confidence: {self.confidence_threshold} | Target FPS: {self.target_fps}"
        ]

        for i, text in enumerate(status_info):
            y_pos = 22 + (i * 20)
            font_scale = 0.6
            color = (0, 255, 255) if i == 0 else (255, 255, 255)
            cv2.putText(frame, text, (10, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, 1)

        return frame
