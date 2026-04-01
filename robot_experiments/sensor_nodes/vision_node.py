import cv2
import socket
from deepface import DeepFace

# --- 1. SETUP NETWORK BRIDGE ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- 2. AUTO-FIND CAMERA ---
def get_working_camera():
    print("Hunting for the RealSense RGB camera...")
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"SUCCESS: Connected to camera at index {i}")
                return cap
        cap.release()
    return None

cap = get_working_camera()
if cap is None:
    print("FATAL: Could not find any video stream. Is the RealSense plugged in?")
    exit()

print(f"\nVision Node Started. Broadcasting emotions to Port {UDP_PORT}...")
print("Press ESC to quit.")

# --- 3. MAIN LOOP ---
while True:
    ret, frame = cap.read()
    if not ret: 
        continue

    try:
        # Analyze the frame (enforce_detection=True makes it honest)
        result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=True)
        face_data = result[0]
        dominant_emotion = face_data['dominant_emotion']
        
        # Extract skeleton/box coordinates
        region = face_data['region']
        x, y = region['x'], region['y']
        w, h = region['w'], region['h']

        # Broadcast the emotion to the C++ Robot!
        sock.sendto(dominant_emotion.encode(), (UDP_IP, UDP_PORT))

        # Draw UI
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
        cv2.putText(frame, f"SENDING: {dominant_emotion.upper()}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    except ValueError:
        # If no face is detected, tell the robot to relax (neutral)
        sock.sendto(b"neutral", (UDP_IP, UDP_PORT))
        cv2.putText(frame, "NO FACE - SENDING: NEUTRAL", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show the window
    cv2.imshow('Sisyphus Nervous System: Eyes', frame)
    
    if cv2.waitKey(1) & 0xFF == 27: 
        break

# Clean up
cap.release()
cv2.destroyAllWindows()