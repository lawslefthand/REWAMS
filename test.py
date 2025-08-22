import cv2
import numpy as np
import onnxruntime as ort

# --- Load YOLO ONNX model ---
MODEL_PATH = "best.onnx"   # change this to your YOLO model path
session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])

# Get model input details
input_name = session.get_inputs()[0].name
input_shape = session.get_inputs()[0].shape
input_height, input_width = input_shape[2], input_shape[3]

# Open webcam (use video path instead if needed)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get frame dimensions
    H, W = frame.shape[:2]
    frame_center = (W // 2, H // 2)

    # Preprocess image for YOLO
    img = cv2.resize(frame, (input_width, input_height))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
    img = np.expand_dims(img, axis=0)

    # Run inference
    outputs = session.run(None, {input_name: img})
    detections = outputs[0][0]  # depends on model, assume YOLO-like [x,y,w,h,conf,class]

    for det in detections:
        conf = det[4]
        if conf < 0.4:  # confidence threshold
            continue

        # Get bbox in original frame size
        x, y, w, h = det[0:4]
        x = int(x * W / input_width)
        y = int(y * H / input_height)
        w = int(w * W / input_width)
        h = int(h * H / input_height)

        x1, y1 = x - w // 2, y - h // 2
        x2, y2 = x + w // 2, y + h // 2

        # Box center
        box_center = (x, y)

        # Distance from frame center (in pixels)
        dx = box_center[0] - frame_center[0]
        dy = box_center[1] - frame_center[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, box_center, 4, (0, 0, 255), -1)

        # Draw line to frame center
        cv2.line(frame, frame_center, box_center, (255, 0, 0), 2)

        # Show distance
        cv2.putText(frame, f"Dist: {distance:.1f}px", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # Draw frame center crosshair
    cv2.drawMarker(frame, frame_center, (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

    cv2.imshow("YOLO Detections", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
