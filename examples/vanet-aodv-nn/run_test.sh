#!/bin/bash

# VANET AODV Neural Network Simulation Test Script
# Enhanced with OFDM errors, realistic urban channels, and 7-input neural network
#
# This script runs:
# 1. Baseline AODV (standard, no NN) for comparison
# 2. Enhanced AODV with Neural Network link quality estimation
#
# Modes:
# - AUTO: Automatically runs both simulations
# - MANUAL: Provides commands to run manually
# - ENHANCED-ONLY: Runs only the enhanced version with NN
# - BASELINE-ONLY: Runs only the baseline for comparison

echo "========================================"
echo " VANET AODV COMPARISON FRAMEWORK"
echo " Baseline vs Neural Network Enhanced"
echo "========================================"

# Clean up old processes
echo "Cleaning up old processes..."
pkill -f vanet-aodv-gym 2>/dev/null
pkill -f link_quality_agent 2>/dev/null
sleep 2

# Check port
echo "Checking port 5555..."
if lsof -i :5555 > /dev/null 2>&1; then
    echo "WARNING: Port 5555 is in use!"
    echo "Killing process on port 5555..."
    lsof -t -i :5555 | xargs kill -9 2>/dev/null
    sleep 2
fi

echo "✓ Port 5555 is free"
echo ""

# Configuration
NUM_VEHICLES=10
SIM_TIME=50
VEHICLE_SPEED=20
PORT=5555
ENABLE_NETANIM=0  # Disable for faster simulation

echo "========================================"
echo " SIMULATION CONFIGURATION"
echo "========================================"
echo "Vehicles:         $NUM_VEHICLES"
echo "Simulation Time:  ${SIM_TIME}s"
echo "Max Speed:        ${VEHICLE_SPEED} m/s"
echo "OpenGym Port:     $PORT"
echo "NetAnim:          $([ $ENABLE_NETANIM -eq 1 ] && echo 'Enabled' || echo 'Disabled')"
echo ""
echo "Features:"
echo "  ✓ OFDM Preamble & Payload Error Tracking"
echo "  ✓ Realistic Urban Channel (ThreeLogDistance + Shadowing)"
echo "  ✓ 7-Input Neural Network"
echo "  ✓ SUMO Mobility Support"
echo "========================================"
echo ""

# Ask user for execution mode
echo ""
echo "SELECT SIMULATION MODE:"
echo "========================================"
echo "  1) AUTO - Run BOTH Baseline + Enhanced"
echo "  2) AUTO - Enhanced ONLY (with NN)"
echo "  3) AUTO - Baseline ONLY (for comparison)"
echo "  4) MANUAL - Show commands only"
echo "========================================"
echo ""
read -p "Enter choice [1-4]: " MODE

if [ "$MODE" = "1" ]; then
    # AUTO MODE - RUN BOTH BASELINE AND ENHANCED
    echo ""
    echo "========================================"
    echo " AUTO MODE: BASELINE + ENHANCED"
    echo "========================================"
    echo ""

    # ====================
    # STEP 1: Run Baseline AODV (No NN)
    # ====================
    echo "STEP 1/2: Running BASELINE AODV (Standard, No NN)..."
    echo "----------------------------------------"
    cd /home/uzair/Desktop/Hamida/ns-3

    ./build/scratch/ns3.40-vanet-aodv-baseline-default \
        --numVehicles=$NUM_VEHICLES \
        --simTime=$SIM_TIME \
        --vehicleSpeed=$VEHICLE_SPEED \
        --enableNetAnim=$ENABLE_NETANIM

    BASELINE_RESULT=$?
    if [ $BASELINE_RESULT -eq 0 ]; then
        echo "✓ Baseline simulation completed successfully"
        echo "  Results saved to: baseline-aodv-results.txt"
    else
        echo "✗ Baseline simulation failed with error code: $BASELINE_RESULT"
        exit 1
    fi

    echo ""
    echo "Waiting 5 seconds before starting enhanced simulation..."
    sleep 5
    echo ""

    # ====================
    # STEP 2: Run Enhanced AODV (With NN)
    # ====================
    echo "STEP 2/2: Running ENHANCED AODV (Neural Network)..."
    echo "----------------------------------------"

    # Start NS-3 in background
    ./build/contrib/opengym/examples/ns3.40-vanet-aodv-gym-default \
        --numVehicles=$NUM_VEHICLES \
        --simTime=$SIM_TIME \
        --vehicleSpeed=$VEHICLE_SPEED \
        --openGymPort=$PORT \
        --enableNetAnim=$ENABLE_NETANIM &

    NS3_PID=$!
    echo "NS-3 Enhanced started (PID: $NS3_PID)"
    echo "Waiting 3 seconds for NS-3 to initialize..."
    sleep 3

    # Start Python agent
    echo "Starting Python Neural Network agent..."
    cd /home/uzair/Desktop/Hamida/ns-3/contrib/opengym/examples/vanet-aodv-nn

    if [ -d "../../ns3gym-venv" ]; then
        source ../../ns3gym-venv/bin/activate
    fi

    python3 link_quality_agent.py --port=$PORT --steps=2000

    # Wait for NS-3 to finish
    echo ""
    echo "Python agent finished. Waiting for NS-3 to complete..."
    wait $NS3_PID

    echo ""
    echo "========================================"
    echo " BOTH SIMULATIONS COMPLETE"
    echo "========================================"
    echo ""
    echo "RESULTS COMPARISON:"
    echo "----------------------------------------"
    echo "Baseline (Standard AODV):"
    echo "  File: baseline-aodv-results.txt"
    if [ -f "/home/uzair/Desktop/Hamida/ns-3/baseline-aodv-results.txt" ]; then
        echo "  ✓ Results file exists"
        grep "Overall PDR:" /home/uzair/Desktop/Hamida/ns-3/baseline-aodv-results.txt 2>/dev/null || echo "  (Check file for PDR)"
    fi
    echo ""
    echo "Enhanced (with Neural Network):"
    echo "  CSV Dataset: vanet_link_dataset_*.csv"
    echo "  PyTorch Model: link_quality_model_*.pth"
    echo "  Logs: Check NS-3 output above"
    echo ""
    echo "Compare the PDR, throughput, and delay metrics!"
    echo "========================================"
    echo ""

elif [ "$MODE" = "2" ]; then
    # AUTO MODE - ENHANCED ONLY
    echo ""
    echo "========================================"
    echo " AUTO MODE: ENHANCED ONLY"
    echo "========================================"

    # Start NS-3 in background
    echo "Starting NS-3 Enhanced simulation..."
    cd /home/uzair/Desktop/Hamida/ns-3

    ./build/contrib/opengym/examples/ns3.40-vanet-aodv-gym-default \
        --numVehicles=$NUM_VEHICLES \
        --simTime=$SIM_TIME \
        --vehicleSpeed=$VEHICLE_SPEED \
        --openGymPort=$PORT \
        --enableNetAnim=$ENABLE_NETANIM &

    NS3_PID=$!
    echo "NS-3 started (PID: $NS3_PID)"
    echo "Waiting 3 seconds for NS-3 to initialize..."
    sleep 3

    # Start Python agent
    echo ""
    echo "Starting Python agent..."
    cd /home/uzair/Desktop/Hamida/ns-3/contrib/opengym/examples/vanet-aodv-nn

    if [ -d "../../ns3gym-venv" ]; then
        source ../../ns3gym-venv/bin/activate
    fi

    python3 link_quality_agent.py --port=$PORT --steps=2000

    # Wait for NS-3 to finish
    echo ""
    echo "Python agent finished. Waiting for NS-3 to complete..."
    wait $NS3_PID

    echo ""
    echo "========================================"
    echo " ENHANCED SIMULATION COMPLETE"
    echo "========================================"
    echo ""
    echo "Results:"
    echo "  - CSV Dataset: vanet_link_dataset_*.csv"
    echo "  - PyTorch Model: link_quality_model_*.pth"
    if [ $ENABLE_NETANIM -eq 1 ]; then
        echo "  - NetAnim: /home/uzair/Desktop/Hamida/ns-3/vanet-aodv-gym.xml"
    fi
    echo ""

elif [ "$MODE" = "3" ]; then
    # AUTO MODE - BASELINE ONLY
    echo ""
    echo "========================================"
    echo " AUTO MODE: BASELINE ONLY"
    echo "========================================"
    echo ""
    echo "Running standard AODV (no NN) for comparison..."
    cd /home/uzair/Desktop/Hamida/ns-3

    ./build/scratch/ns3.40-vanet-aodv-baseline-default \
        --numVehicles=$NUM_VEHICLES \
        --simTime=$SIM_TIME \
        --vehicleSpeed=$VEHICLE_SPEED \
        --enableNetAnim=$ENABLE_NETANIM

    echo ""
    echo "========================================"
    echo " BASELINE SIMULATION COMPLETE"
    echo "========================================"
    echo ""
    echo "Results saved to: baseline-aodv-results.txt"
    echo ""
    if [ -f "baseline-aodv-results.txt" ]; then
        echo "Quick Summary:"
        grep -A 7 "OVERALL STATISTICS:" baseline-aodv-results.txt
    fi
    echo ""

elif [ "$MODE" = "4" ]; then
    # MANUAL MODE - INSTRUCTIONS ONLY
    echo ""
    echo "========================================"
    echo " MANUAL MODE - INSTRUCTIONS"
    echo "========================================"
    echo ""
    echo "=========================================="
    echo " OPTION A: Run BASELINE (for comparison)"
    echo "=========================================="
    echo ""
    echo "Single Terminal:"
    echo "cd /home/uzair/Desktop/Hamida/ns-3"
    echo ""
    echo "./build/scratch/ns3.40-vanet-aodv-baseline-default \\"
    echo "    --numVehicles=$NUM_VEHICLES \\"
    echo "    --simTime=$SIM_TIME \\"
    echo "    --vehicleSpeed=$VEHICLE_SPEED \\"
    echo "    --enableNetAnim=$ENABLE_NETANIM"
    echo ""
    echo "Results: baseline-aodv-results.txt"
    echo ""
    echo "=========================================="
    echo " OPTION B: Run ENHANCED (with Neural Network)"
    echo "=========================================="
    echo ""
    echo "Terminal 1 (NS-3 Simulation):"
    echo "----------------------------------------"
    echo "cd /home/uzair/Desktop/Hamida/ns-3"
    echo ""
    echo "./build/contrib/opengym/examples/ns3.40-vanet-aodv-gym-default \\"
    echo "    --numVehicles=$NUM_VEHICLES \\"
    echo "    --simTime=$SIM_TIME \\"
    echo "    --vehicleSpeed=$VEHICLE_SPEED \\"
    echo "    --openGymPort=$PORT \\"
    echo "    --enableNetAnim=$ENABLE_NETANIM"
    echo ""
    echo "Wait for: 'Waiting for Python agent to connect on port $PORT...'"
    echo ""
    echo "Terminal 2 (Python Agent) - Start AFTER NS-3:"
    echo "----------------------------------------"
    echo "cd /home/uzair/Desktop/Hamida/ns-3/contrib/opengym/examples/vanet-aodv-nn"
    echo "source ../../ns3gym-venv/bin/activate"
    echo ""
    echo "python3 link_quality_agent.py --port=$PORT --steps=2000"
    echo ""
    echo "=========================================="
    echo " OPTION C: Run BOTH for Comparison"
    echo "=========================================="
    echo "Run Option A first, then Option B"
    echo "Compare baseline-aodv-results.txt with enhanced results"
    echo ""
    echo "============================================"
    echo ""
    echo "OPTIONAL: With SUMO Mobility (Manhattan)"
    echo "============================================"
    echo "./build/contrib/opengym/examples/ns3.40-vanet-aodv-gym-default \\"
    echo "    --numVehicles=20 \\"
    echo "    --simTime=100 \\"
    echo "    --sumoTraceFile=/path/to/manhattan.tcl \\"
    echo "    --openGymPort=$PORT \\"
    echo "    --enableNetAnim=0"
    echo ""
    echo "============================================"
    echo ""
    echo "Available Parameters (Both Simulations):"
    echo "----------------------------------------"
    echo "  --numVehicles=X      Number of vehicles (default: 10)"
    echo "  --numRSUs=X          Number of RSUs (default: 4)"
    echo "  --simTime=X          Simulation time in seconds (default: 100)"
    echo "  --vehicleSpeed=X     Max vehicle speed m/s (default: 20)"
    echo "  --openGymPort=X      OpenGym port (default: 5555, Enhanced only)"
    echo "  --enableNetAnim=0/1  NetAnim visualization (default: 1)"
    echo "  --enableMobility=0/1 Enable mobility (default: 1)"
    echo "  --sumoTraceFile=PATH SUMO mobility trace file (Enhanced only)"
    echo "  --verbose=1          Enable verbose logging"
    echo ""
    echo "Expected Results Comparison:"
    echo "----------------------------------------"
    echo "Baseline AODV (IEEE 802.11p):"
    echo "  - PDR: 75-85%"
    echo "  - Technology: Standard DSRC"
    echo "  - Routing: Pure AODV"
    echo ""
    echo "Enhanced AODV (with NN):"
    echo "  - PDR: 80-90% (OPTIMIZED)"
    echo "  - Technology: OPTIMIZED 802.11p"
    echo "  - Routing: AODV + Neural Network link quality"
    echo "  - Features: ML-based route selection"
    echo ""
    echo "========================================"
    echo ""

else
    echo ""
    echo "Invalid choice. Please run script again and select 1-4."
    exit 1
fi

echo "Press Ctrl+C to stop simulation"
echo "========================================"
