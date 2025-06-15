import cv2
import numpy as np
import mediapipe as mp
import tensorflow as tf
import serial
import time

# ========== CONFIGURATION ==========
TESTING = True  # Set to True to disable serial communication for testing

# Use the correct port for your system if not testing
esp32_port = 'COM3'  # Example for Windows, change as needed
baud_rate = 115200

width = 640
height = 360

iris_radius = 48
eye_radius = 80

# ========== LOAD MODELS AND IMAGES ==========
interpreter = tf.lite.Interpreter(model_path='iris_gesture_model.tflite')
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5)

# Define the eye and iris landmark indices
right_eye_indices = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246]
left_eye_indices = [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398]
left_iris_indices = [474, 475, 476, 477, 473]
right_iris_indices = [469, 470, 471, 472, 468]
eye_iris_indices = left_eye_indices + left_iris_indices + right_eye_indices + right_iris_indices
all_eye_indices = eye_iris_indices

# Load the iris and eye images with alpha channel
iris_img = cv2.imread('iris_image.png', cv2.IMREAD_UNCHANGED)
iris_img = cv2.resize(iris_img, (2 * iris_radius, 2 * iris_radius))
eye_img = cv2.imread('eye_image.png', cv2.IMREAD_UNCHANGED)
eye_img = cv2.resize(eye_img, (2 * eye_radius, 2 * eye_radius))

# Center of the eyes
eye_centers = [(width // 3, height // 2), (2 * width // 3, height // 2)]

# ========== LABELS AND BALL STATE ==========
labels = ['up', 'down', 'right', 'left', 'center', 'both close', 'left close', 'right close']
ball_positions = [eye_centers[0], eye_centers[1]]
ball_speed = 20

# ========== FUNCTIONS ==========

def extract_and_plot_eye_indices(frame, face_landmarks):
    all_eye_points = np.array([[int(face_landmarks.landmark[i].x * frame.shape[1]),
                                int(face_landmarks.landmark[i].y * frame.shape[0])]
                               for i in all_eye_indices])
    ex, ey, ew, eh = cv2.boundingRect(all_eye_points)
    margin = 5
    ex = max(0, ex - margin)
    ey = max(0, ey - margin)
    ew = min(frame.shape[1], ew + 2 * margin)
    eh = min(frame.shape[0], eh + 2 * margin)
    eye_region = frame[ey:ey + eh, ex:ex + ew]
    new_height = int(frame.shape[1] * eh / ew)
    eye_enlarged = cv2.resize(eye_region, (frame.shape[1], new_height), interpolation=cv2.INTER_CUBIC)
    enlarged_frame = np.zeros((new_height, frame.shape[1], 3), dtype=np.uint8)
    enlarged_frame[:new_height, :frame.shape[1]] = eye_enlarged
    points = []
    for i in all_eye_indices:
        nx = int((face_landmarks.landmark[i].x * frame.shape[1] - ex) * frame.shape[1] / ew)
        ny = int((face_landmarks.landmark[i].y * frame.shape[0] - ey) * new_height / eh)
        points.append((nx, ny))
    points = np.array(points, dtype=np.int32)
    (r_cx, r_cy), r_radius = cv2.minEnclosingCircle(points[16:20])
    (l_cx, l_cy), l_radius = cv2.minEnclosingCircle(points[37:41])
    center_right = np.array([r_cx, r_cy], dtype=np.int32)
    center_left = np.array([l_cx, l_cy], dtype=np.int32)
    cv2.polylines(enlarged_frame, [points[:15]], isClosed=True, color=(0, 255, 255), thickness=2)
    cv2.circle(enlarged_frame, center_right, int(r_radius), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(enlarged_frame, points[20], 3, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.polylines(enlarged_frame, [points[21:36]], isClosed=True, color=(0, 255, 255), thickness=2)
    cv2.circle(enlarged_frame, center_left, int(l_radius), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(enlarged_frame, points[41], 3, (255, 255, 255), 2, cv2.LINE_AA)
    return enlarged_frame

def normalize_landmarks(landmarks, indices):
    left_eye_center = np.mean([[landmarks[i].x, landmarks[i].y] for i in left_eye_indices], axis=0)
    right_eye_center = np.mean([[landmarks[i].x, landmarks[i].y] for i in right_eye_indices], axis=0)
    mid_point = (left_eye_center + right_eye_center) / 2.0
    normalized = np.array([[landmarks[i].x - mid_point[0], landmarks[i].y - mid_point[1]] for i in indices])
    return normalized.flatten()

def move_ball(gesture):
    global ball_positions
    if gesture == "close":
        return
    for i in range(len(ball_positions)):
        ball_x, ball_y = ball_positions[i]
        if gesture == "center":
            ball_positions[i] = eye_centers[i]
            continue
        if gesture == "up":
            ball_y -= ball_speed
        elif gesture == "down":
            ball_y += ball_speed
        elif gesture == "left" or gesture == "left close":
            ball_x += ball_speed
        elif gesture == "right" or gesture == "right close":
            ball_x -= ball_speed
        dx = ball_x - eye_centers[i][0]
        dy = ball_y - eye_centers[i][1]
        distance = np.sqrt(dx ** 2 + dy ** 2)
        if distance > (eye_radius - iris_radius):
            angle = np.arctan2(dy, dx)
            ball_x = int(eye_centers[i][0] + (eye_radius - iris_radius) * np.cos(angle))
            ball_y = int(eye_centers[i][1] + (eye_radius - iris_radius) * np.sin(angle))
        ball_positions[i] = (ball_x, ball_y)

def overlay_image_alpha(img, img_overlay, pos, alpha_mask):
    x, y = pos
    y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return
    img_crop = img[y1:y2, x1:x2]
    img_overlay_crop = img_overlay[y1o:y2o, x1o:x2o]
    alpha = alpha_mask[y1o:y2o, x1o:x2o, np.newaxis]
    alpha_inv = 1.0 - alpha
    img_crop[:] = alpha * img_overlay_crop + alpha_inv * img_crop

def draw_half_closed_eyelid(img, center, radius):
    overlay = img.copy()
    cv2.ellipse(overlay, center, (radius, radius // 2), 0, 0, 180, (0, 0, 0), -1)
    alpha = 0.4
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

# ========== MAIN LOOP VARIABLES ==========
gesture_name = ""
system_on = False
eye_closed_duration = 0
eye_state_start_time = None
eye_closed_threshold = 4  # seconds

# ========== MAIN LOOP ==========
try:
    ser = None
    if not TESTING:
        try:
            ser = serial.Serial(esp32_port, baud_rate, timeout=1)
            print(f"Connected to {esp32_port} at {baud_rate} baud")
            time.sleep(0.2)
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            ser = None
    else:
        print("TESTING MODE: Serial communication disabled.")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Camera not found. Exiting.")
        exit(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = face_mesh.process(rgb_frame)

        if result.multi_face_landmarks:
            for face_landmarks in result.multi_face_landmarks:
                enlarged_frame = extract_and_plot_eye_indices(frame, face_landmarks)
                cv2.imshow('Enlarged Eyes', enlarged_frame)

                normalized_landmarks = normalize_landmarks(face_landmarks.landmark, eye_iris_indices)
                input_data = np.array(normalized_landmarks, dtype=np.float32).reshape(1, -1)
                interpreter.set_tensor(input_details[0]['index'], input_data)
                interpreter.invoke()
                output_data = interpreter.get_tensor(output_details[0]['index'])
                predicted_label = np.argmax(output_data[0])
                gesture_name = labels[predicted_label]

                # Handle turn on/off mechanism
                if gesture_name == "right close" or gesture_name == "left close":
                    if eye_state_start_time is None:
                        eye_state_start_time = time.time()
                    eye_closed_duration = time.time() - eye_state_start_time
                    if eye_closed_duration > eye_closed_threshold:
                        system_on = not system_on
                        eye_state_start_time = None
                        print(f"System {'ON' if system_on else 'OFF'}")
                        if not system_on and ser is not None:
                            try:
                                ser.write("system_off\n".encode())
                                print("Sent gesture: system_off")
                            except serial.SerialException as e:
                                print(f"Serial communication error while sending system_off: {e}")
                else:
                    eye_state_start_time = None
                    eye_closed_duration = 0

                # Only process gestures if the system is ON
                if system_on:
                    if ser is not None:
                        try:
                            ser.write(f"{gesture_name}\n".encode())
                            print(f"Sent gesture: {gesture_name}")
                            response = ser.readline().decode('utf-8', errors='ignore').strip()
                            if response:
                                print(f"ESP32 Response: {response}")
                        except serial.SerialException as e:
                            print(f"Serial communication error while sending gesture: {e}")
                    else:
                        print(f"(TESTING) Would send gesture: {gesture_name}")

                # Display the predicted gesture and system status
                cv2.putText(frame, f'System: {"ON" if system_on else "OFF"}', (50, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if system_on else (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(frame, f'Gesture: {gesture_name}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 0, 0), 2, cv2.LINE_AA)
                if eye_closed_duration > 0:
                    cv2.putText(frame, f'Eyes Closed: {eye_closed_duration:.2f}s', (50, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 125, 255), 2, cv2.LINE_AA)

        cv2.imshow('Iris Gesture Detection', frame)

        # Draw the eye window with iris/eyelid overlays
        eye_window = np.zeros((height, width, 3), np.uint8) * 255
        for i, center in enumerate(eye_centers):
            overlay_image_alpha(eye_window, eye_img[:, :, :3],
                                (center[0] - eye_radius, center[1] - eye_radius),
                                eye_img[:, :, 3] / 255.0)
            if i == 0 and gesture_name in ["both close", "right close"]:
                draw_half_closed_eyelid(eye_window, eye_centers[0], eye_radius)
            elif i == 1 and gesture_name in ["both close", "left close"]:
                draw_half_closed_eyelid(eye_window, eye_centers[1], eye_radius)
            else:
                overlay_image_alpha(eye_window, iris_img[:, :, :3],
                                    (ball_positions[i][0] - iris_radius, ball_positions[i][1] - iris_radius),
                                    iris_img[:, :, 3] / 255.0)
        cv2.putText(eye_window, f'Gesture: {gesture_name}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 125, 255), 2, cv2.LINE_AA)
        cv2.putText(eye_window, f'Eyes Closed: {eye_closed_duration:.2f}s', (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('Eyeball Movement', eye_window)

        # Move the ball according to gesture
        move_ball(gesture_name)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

except serial.SerialException as e:
    print(f"Serial communication error: {e}")
finally:
    if 'ser' in locals() and ser is not None and hasattr(ser, 'is_open') and ser.is_open:
        ser.close()
        print("Serial connection closed.")