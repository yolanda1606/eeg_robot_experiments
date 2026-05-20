#!/bin/bash

BASE_DIR="/home/sysgen/Projects/Yolanda/Thesis/robot_experiments"

# Hide annoying TensorFlow/DeepFace GPU warnings
export TF_CPP_MIN_LOG_LEVEL=2 

echo "================================================"
echo "      🧠 EEG + ROBOT EXPERIMENT LAUNCHER 🤖      "
echo "================================================"

echo "Select the experiment to run:"
echo "  1) Pick and Place"
echo "  2) Shape Sorter (Observation)"
echo "  3) Shape Sorter (Interactive/HRI)"
echo "  4) Sisyphus"
echo "  5) Stack"
echo -n "Choice (1-5): "
read EXP_CHOICE

FAULT_MODE=0
NEEDS_FAULT_PROMPT=false

case $EXP_CHOICE in
    1)
        SUFFIX="PnP"
        PROJ_DIR="$BASE_DIR/pick_and_place_cpp"
        # ⚠️ VERIFY THIS NAME matches the file in your build folder!
        EXEC_NAME="pick_place" 
        ;;
    2)
        SUFFIX="SS_obs"
        PROJ_DIR="$BASE_DIR/shape_sorter_cpp"
        EXEC_NAME="shape_sorter"   
        NEEDS_FAULT_PROMPT=true
        ;;
    3)
        SUFFIX="SS_int"
        PROJ_DIR="$BASE_DIR/shape_sorter_hri"
        EXEC_NAME="shape_sorter"   
        ;;
    4)
        SUFFIX="Sisy"
        PROJ_DIR="$BASE_DIR/sisyphus_cpp_project"
        EXEC_NAME="sisyphus_task"       
        ;;
    5)
        SUFFIX="Stack"
        PROJ_DIR="$BASE_DIR/stack_cpp"
        EXEC_NAME="stack"    
        NEEDS_FAULT_PROMPT=true
        ;;
    *)
        echo "❌ Invalid choice. Exiting."
        exit 1
        ;;
esac

if [ "$NEEDS_FAULT_PROMPT" = true ]; then
    echo ""
    echo "Mode Selection for $SUFFIX:"
    echo "  0) Fault-Free Run (Control)"
    echo "  1) Faulty Run (Surprise)"
    echo -n "Choice (0 or 1): "
    read FAULT_MODE
    
    if [ "$FAULT_MODE" -eq 1 ]; then
        SUFFIX="${SUFFIX}f"
    fi
fi

# --- THE NEW CTRL+C TRAP ---
# If Ctrl+C is pressed, remove the trap (prevents looping), stop the robot immediately, and stop the camera.
trap 'trap - SIGINT; echo -e "\n🚨 EMERGENCY STOP! Halting Robot and Camera..."; kill -2 $ROBOT_PID 2>/dev/null; kill -2 $VISION_PID 2>/dev/null; wait $VISION_PID; exit 1' SIGINT

echo ""
echo "🚀 Starting Data Collection for: $SUFFIX"

# Start Vision Node
echo "📷 Starting Vision Node..."
python3 $BASE_DIR/sensor_nodes/vision_node.py --file_suffix "$SUFFIX" &
VISION_PID=$!

sleep 5 

# Run the Robot Code in the BACKGROUND so the bash script can monitor it
echo "🤖 Executing C++ Robot Node..."
cd "$PROJ_DIR/build" || { echo "❌ Could not find build directory in $PROJ_DIR"; kill -2 $VISION_PID; exit 1; }

if [ "$NEEDS_FAULT_PROMPT" = true ]; then
    echo "$FAULT_MODE" | ./$EXEC_NAME &
else
    ./$EXEC_NAME &
fi
ROBOT_PID=$! # Capture the Robot's Process ID

# Wait here until the Robot code finishes naturally
wait $ROBOT_PID

# Once the robot finishes (and we get past the 'wait'), safely shut down the camera
echo "✅ Robot path complete! Saving video files..."
kill -2 $VISION_PID 2>/dev/null
wait $VISION_PID

echo "🎉 Experiment sequence saved successfully!"