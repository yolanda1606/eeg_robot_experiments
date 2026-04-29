import cv2
import socket
import csv
import time
import threading
import datetime
import os
import numpy as np
import pyrealsense2 as rs  
from deepface import DeepFace

# --- 1. SETUP NETWORK & VARIABLES ---
UDP_IP_LISTEN = "127.0.0.1"
UDP_PORT_LISTEN = 5005

latest_trigger = 0 
trigger_lock = threading.Lock()

# --- 2. UDP LISTENER THREAD ---
def udp_listener():
    global latest_trigger
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP_LISTEN, UDP_PORT_LISTEN))
    print(f"Listening for triggers on {UDP_IP_LISTEN}:{UDP_PORT_LISTEN}...")
    
    while True:
        data, addr = sock.recvfrom(1024)
        if data:
            with trigger_lock:
                latest_trigger = int(data.decode('utf-8'))
                print(f"Trigger {latest_trigger} received at {time.time()}!")

listener_thread = threading.Thread(target=udp_listener, daemon=True)
listener_thread.start()

# --- 3. FACE DETECTION AI THREAD ---
frame_for_detection = None
detection_lock = threading.Lock()

detected_emotion = "no face"
detected_box = None
box_lock = threading.Lock()

def face_detector_thread():
    global frame_for_detection, detected_emotion, detected_box
    print("AI Detection Thread Started...")
    while True:
        with detection_lock:
            # We are back to grabbing the clean RGB frame!
            current_frame = frame_for_detection.copy() if frame_for_detection is not None else None
        
        if current_frame is not None:
            try:
                # Using MTCNN: It is highly accurate and ignores robot arms.
                # The lag won't affect the video because it is in this background thread.
                result = DeepFace.analyze(
                    current_frame, 
                    actions=['emotion'], 
                    enforce_detection=True,
                    # detector_backend='mtcnn' 
                    detector_backend='opencv' 
                )
                
                face_data = result[0]
                region = face_data['region']
                w, h = region['w'], region['h']

                # Lowered the size threshold slightly to be more forgiving if you lean back
                if w >= 80 and h >= 80:
                    with box_lock:
                        detected_box = (region['x'], region['y'], w, h)
                        detected_emotion = face_data['dominant_emotion'] 
                else:
                    raise ValueError("Face too small")

            except ValueError:
                with box_lock:
                    detected_box = None
                    detected_emotion = "no face" 
            
            time.sleep(0.01)
        else:
            time.sleep(0.1)

ai_thread = threading.Thread(target=face_detector_thread, daemon=True)
ai_thread.start()

# --- 4. REALSENSE CAMERA SETUP ---
print("Waking up Intel RealSense camera (Single RGB Stream)...")
pipeline = rs.pipeline()
config = rs.config()

frame_width = 1280
frame_height = 720
fps = 30

# Request ONLY the clean RGB Color feed
config.enable_stream(rs.stream.color, frame_width, frame_height, rs.format.bgr8, fps)

try:
    pipeline.start(config)
    print("SUCCESS: Connected to RealSense RGB stream!")
except Exception as e:
    print(f"FATAL: Could not start RealSense camera. Error: {e}")
    exit()

# --- 5. SETUP DIRECTORIES, VIDEO WRITER & CSV ---
now = datetime.datetime.now()
date_folder = now.strftime("%Y-%m-%d")  
time_stamp = now.strftime("%H-%M-%S")   

save_dir = os.path.join("data", date_folder)
os.makedirs(save_dir, exist_ok=True)

video_filename = os.path.join(save_dir, f"raw_experiment_video_{time_stamp}.avi")
csv_filename = os.path.join(save_dir, f"experiment_video_log_{time_stamp}.csv")

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_video = cv2.VideoWriter(video_filename, fourcc, fps, (frame_width, frame_height))

csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Timestamp', 'Frame_Count', 'Trigger', 'Emotion'])

print(f"\nSaving data to: {save_dir}")
print("Vision Node Started. Recording Video and Log...")
print("-> To quit: Click on the VIDEO WINDOW and press ESC.")
print("-> Or, click this TERMINAL and press Ctrl+C.")

frame_count = 0

try:
    # --- 6. MAIN LOOP ---
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame: 
            continue
            
        color_image = np.asanyarray(color_frame.get_data())
        
        # Send the CLEAN RGB frame to the AI thread
        with detection_lock:
            frame_for_detection = color_image
        
        frame_count += 1
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        # Save the clean RGB frame to your video file
        out_video.write(color_image)

        with trigger_lock:
            trigger_to_log = latest_trigger
            latest_trigger = 0 

        with box_lock:
            current_box = detected_box
            current_emotion = detected_emotion

        # 4. Draw UI on the clean RGB frame
        if current_box is not None:
            x, y, w, h = current_box
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (255, 0, 0), 3)
            cv2.putText(color_image, f"EMOTION: {current_emotion.upper()}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(color_image, "NO FACE DETECTED", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if trigger_to_log != 0:
            cv2.putText(color_image, f"TRIGGER: {trigger_to_log}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)

        csv_writer.writerow([current_time, frame_count, trigger_to_log, current_emotion])
        
        cv2.imshow('Sisyphus Nervous System: Eyes', color_image)
        
        if cv2.waitKey(1) & 0xFF == 27: 
            print("\nESC pressed in video window. Exiting...")
            break

except KeyboardInterrupt:
    print("\nCtrl+C detected in terminal! Forcing a safe shutdown...")

finally:
    # --- 7. CLEAN UP ---
    print("Cleaning up resources...")
    pipeline.stop()  
    out_video.release()
    csv_file.close()
    cv2.destroyAllWindows()
    print(f"Data saved successfully to {save_dir}.")