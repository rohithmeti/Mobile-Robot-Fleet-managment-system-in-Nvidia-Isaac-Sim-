#!/usr/bin/env python3
"""
RMF Diagnostics Tool - WITH FREQUENCY (Hz) MONITORING
Captures topic data, frequencies, and pub/sub info for key RMF topics.
Run: python3 test.py
Stop: Ctrl+C  ->  saves to ~/rmf_diagnostics_<timestamp>.txt
"""

import subprocess
import threading
import time
import os
import signal
import sys
import re
from datetime import datetime

OUTPUT_FILE = os.path.expanduser(f"~/rmf_diagnostics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")

# Base RMF Topics
TOPICS = [
    "/rmf_task/bid_notice",
    "/rmf_task/bid_response",
    "/rmf_task/dispatch_request",
    "/rmf_task/dispatch_ack",
    "/rmf_traffic/heartbeat",
    "/rmf_traffic/negotiation_notice",
    "/rmf_traffic/negotiation_proposal",
    "/rmf_traffic/negotiation_conclusion",
    "/rmf_traffic/itinerary_set",
    "/rmf_traffic/itinerary_reached",
    "/rmf_traffic/itinerary_clear",
    "/dispatch_states",
    "/fleet_states",
    "/nav_graphs",
    "/lane_states",
    "/task_api_requests",
    "/task_api_responses",
    "/task_summaries",
]

# Add AMR-specific topics (1 through 5)
for i in range(1, 6):
    TOPICS.append(f"/iw_hub_{i}/speed_limit")
    TOPICS.append(f"/iw_hub_{i}/amcl_pose")
    TOPICS.append(f"/iw_hub_{i}/initialpose")

collected_data = {t: [] for t in TOPICS}
collected_hz = {t: "Not measured" for t in TOPICS}

lock = threading.Lock()
stop_event = threading.Event()

def measure_hz(topic):
    """Measures the publish frequency of a topic for 3 seconds."""
    try:
        # Run ros2 topic hz for exactly 3 seconds
        cmd = ["timeout", "3", "ros2", "topic", "hz", topic]
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        output = result.stdout
        
        # Extract the average rates from the output
        rates = re.findall(r"average rate:\s*([0-9.]+)", output)
        with lock:
            if rates:
                # Take the last measured average rate
                collected_hz[topic] = f"{rates[-1]} Hz"
            elif "no new messages" in output or "not published" in output:
                collected_hz[topic] = "0 Hz (No messages published)"
            else:
                collected_hz[topic] = "0 Hz / Unknown"
    except Exception as e:
        with lock:
            collected_hz[topic] = f"Error measuring Hz"

def echo_topic(topic):
    """Echos the topic to capture data."""
    try:
        proc = subprocess.Popen(["ros2", "topic", "echo", topic], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        current_msg = []
        while not stop_event.is_set():
            line = proc.stdout.readline()
            if not line:
                break
                
            if line.strip() == "---":
                with lock:
                    collected_data[topic].append("\n".join(current_msg))
                current_msg = []
            else:
                current_msg.append(line.strip())
        
        proc.terminate()
        proc.wait()
        
    except Exception as e:
        pass

def print_status():
    while not stop_event.is_set():
        with lock:
            # Print a clean status bar
            sys.stdout.write("\r[Capturing...] " + " | ".join([f"{t.split('/')[-1]}: {len(collected_data[t])}" for t in TOPICS[-5:]]) + " ...")
            sys.stdout.flush()
        time.sleep(1.0)

def main():
    print("RMF Diagnostics Tool - WITH FREQUENCY (Hz) MONITORING")
    print(f"Output: {OUTPUT_FILE}")
    print("Press Ctrl+C to stop and save report.\n")
    
    # 1. Measure frequencies first (Parallel execution)
    print("Step 1: Measuring topic frequencies (Please wait ~3 seconds)...")
    hz_threads = []
    for topic in TOPICS:
        t = threading.Thread(target=measure_hz, args=(topic,))
        t.start()
        hz_threads.append(t)
        
    for t in hz_threads:
        t.join()
        
    print("Frequency measurement complete.\n")
    
    # 2. Start capturing data
    print("Step 2: Capturing topic data... (Ctrl+C to finish)")
    echo_threads = []
    for topic in TOPICS:
        t = threading.Thread(target=echo_topic, args=(topic,))
        t.start()
        echo_threads.append(t)

    # Status printer
    status_thread = threading.Thread(target=print_status)
    status_thread.start()

    def handle_sigint(sig, frame):
        print("\n\nStopping capture...")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
        
    # Ensure all processes are dead
    for t in echo_threads:
        t.join(timeout=1.0)

    # Save to file
    print(f"\nSaving diagnostics to {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, "w") as f:
        f.write("================================================================================\n")
        f.write("RMF DIAGNOSTICS REPORT\n")
        f.write(f"Generated: {datetime.now().isoformat()}\n")
        f.write("================================================================================\n\n")
        
        f.write("================================================================================\n")
        f.write("SECTION 1: TOPIC PUBLISH RATES (Hz)\n")
        f.write("================================================================================\n")
        for t in TOPICS:
            f.write(f"{t: <45} : {collected_hz[t]}\n")
        f.write("\n")

        f.write("================================================================================\n")
        f.write("SECTION 2: TOPIC DATA\n")
        f.write("================================================================================\n")
        for t in TOPICS:
            msgs = collected_data[t]
            f.write(f"────────────────────────────────────────────────────────────\n")
            f.write(f"TOPIC: {t}  ({len(msgs)} messages)\n")
            f.write(f"────────────────────────────────────────────────────────────\n")
            if msgs:
                f.write(f"LATEST MESSAGE:\n{msgs[-1]}\n\n")
            else:
                f.write("  [No messages received — topic may not be publishing]\n\n")

    print("Done.")

if __name__ == '__main__':
    main()
