#!/bin/bash

# Define our paths
ROBOT_DIR="/home/sysgen/Projects/Yolanda/Thesis/robot_experiments"
EXP_DIR="/home/sysgen/Projects/Yolanda/eeg_emotion_experiment"

# Hide TensorFlow warnings
export TF_CPP_MIN_LOG_LEVEL=2 

echo "==================================================="
echo "      🧠 EEG EMOTION IMAGE EXPERIMENT 🖼️          "
echo "==================================================="

# The Safe Shutdown Trap
trap 'trap - SIGINT; echo -e "\n🚨 EMERGENCY STOP! Halting Experiment and Camera..."; pkill -2 -f "emotion_eeg_exp.py"; kill -15 $VISION_PID 2>/dev/null; wait $VISION_PID; exit 1' SIGINT

echo "🚀 Starting Data Collection..."

# 1. Start the central Vision Node (Uses System Python)
echo "📷 Starting Vision Node..."
python3 $ROBOT_DIR/sensor_nodes/vision_node.py --file_suffix "Emotion_Images" &
VISION_PID=$!

# Give the camera 5 seconds to warm up
sleep 5 

# 2. Run your Python Image Experiment (Uses Virtual Environment)
echo "🖼️ Executing Image Presentation..."
cd "$EXP_DIR" || { echo "❌ Could not find directory $EXP_DIR"; kill -15 $VISION_PID; exit 1; }

# --- NEW: Activate the Virtual Environment ---
# ⚠️ If your folder is named 'env' or '.venv', change it here:
source venv/bin/activate 

# Start your experiment in the background and grab its PID
python3 emotion_eeg_exp.py &
EXP_PID=$!

# 3. Wait for the image experiment to finish
wait $EXP_PID

# 4. Clean up when finished
echo "✅ Image experiment complete! Saving video files..."
kill -15 $VISION_PID 2>/dev/null
wait $VISION_PID

echo "🎉 Emotion experiment saved successfully!"