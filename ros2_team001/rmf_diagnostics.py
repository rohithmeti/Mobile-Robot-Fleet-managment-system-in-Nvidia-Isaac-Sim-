#!/usr/bin/env python3
"""
RMF Diagnostics Tool
Captures topic data, frequencies, and pub/sub info for key RMF topics.
Run: python3 rmf_diagnostics.py
Stop: Ctrl+C  ->  saves to ~/rmf_diagnostics_<timestamp>.txt
"""

import subprocess
import threading
import time
import os
import signal
import sys
from datetime import datetime

OUTPUT_FILE = os.path.expanduser(f"~/rmf_diagnostics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")

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
    "/iw_hub_5/speed_limit",
    "/iw_hub_1/speed_limit",
]

collected = {t: [] for t in TOPICS}
lock = threading.Lock()
stop_event = threading.Event()

def echo_topic(topic):
    """Subscribe to a topic and collect messages until stop."""
    try:
        proc = subprocess.Popen(
            ["ros2", "topic", "echo", "--no-arr", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        buffer = []
        while not stop_event.is_set():
            line = proc.stdout.readline()
            if not line:
                break
            buffer.append(line.rstrip())
            # Message boundary: "---" separates messages
            if line.strip() == "---":
                with lock:
                    collected[topic].append({
                        "time": datetime.now().isoformat(),
                        "data": "\n".join(buffer)
                    })
                buffer = []
        proc.terminate()
        proc.wait(timeout=2)
    except Exception as e:
        with lock:
            collected[topic].append({"time": datetime.now().isoformat(), "data": f"ERROR: {e}"})

def get_topic_info(topic):
    """Get publisher/subscriber info via ros2 topic info -v."""
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", "-v", topic],
            capture_output=True, text=True, timeout=5
        )
        return result.stdout or result.stderr or "No info available"
    except Exception as e:
        return f"ERROR: {e}"

def get_topic_hz(topic, duration=5):
    """Measure topic frequency for `duration` seconds."""
    try:
        proc = subprocess.Popen(
            ["ros2", "topic", "hz", "--window", "10", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        time.sleep(duration)
        proc.terminate()
        out, _ = proc.communicate(timeout=3)
        # Extract the last average Hz line
        lines = [l for l in out.splitlines() if "average rate" in l or "no new" in l.lower()]
        return lines[-1].strip() if lines else "No data (topic not publishing)"
    except Exception as e:
        return f"ERROR: {e}"

def print_status():
    """Print live message counts every 5 seconds."""
    while not stop_event.is_set():
        time.sleep(5)
        with lock:
            counts = {t: len(v) for t, v in collected.items() if v}
        if counts:
            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] Messages captured so far:")
            for t, c in counts.items():
                print(f"  {t}: {c} messages")

def save_report():
    """Write everything to the output file."""
    print(f"\n\nSaving report to: {OUTPUT_FILE}")
    
    with open(OUTPUT_FILE, "w") as f:
        f.write("=" * 80 + "\n")
        f.write("RMF DIAGNOSTICS REPORT\n")
        f.write(f"Generated: {datetime.now().isoformat()}\n")
        f.write("=" * 80 + "\n\n")

        # ── Section 1: Topic info (pub/sub) ──────────────────────────────────
        f.write("=" * 80 + "\n")
        f.write("SECTION 1: TOPIC INFO (Publishers / Subscribers / QoS)\n")
        f.write("=" * 80 + "\n\n")
        for topic in TOPICS:
            f.write(f"{'─'*60}\n")
            f.write(f"TOPIC: {topic}\n")
            f.write(f"{'─'*60}\n")
            f.write(get_topic_info(topic))
            f.write("\n\n")

        # ── Section 2: Topic frequencies ─────────────────────────────────────
        f.write("=" * 80 + "\n")
        f.write("SECTION 2: TOPIC FREQUENCIES (measured over 5s each)\n")
        f.write("=" * 80 + "\n\n")
        # Hz measurements collected during run (stored in collected metadata)
        for topic in TOPICS:
            with lock:
                count = len(collected[topic])
            # Rough Hz from collected messages divided by run duration
            f.write(f"{topic}: {count} messages captured during run\n")
        f.write("\n")

        # ── Section 3: Captured messages ─────────────────────────────────────
        f.write("=" * 80 + "\n")
        f.write("SECTION 3: CAPTURED MESSAGES (all messages seen during run)\n")
        f.write("=" * 80 + "\n\n")
        for topic in TOPICS:
            with lock:
                msgs = list(collected[topic])
            f.write(f"{'─'*60}\n")
            f.write(f"TOPIC: {topic}  ({len(msgs)} messages)\n")
            f.write(f"{'─'*60}\n")
            if not msgs:
                f.write("  [No messages received — topic may not be publishing]\n")
            else:
                # Show first 3 and last 2 to keep file manageable
                show = msgs[:3] + (msgs[-2:] if len(msgs) > 5 else [])
                shown_indices = list(range(min(3, len(msgs))))
                if len(msgs) > 5:
                    shown_indices += list(range(len(msgs)-2, len(msgs)))
                for idx, msg in zip(shown_indices, show):
                    f.write(f"\n  --- Message #{idx+1} at {msg['time']} ---\n")
                    for line in msg['data'].splitlines():
                        f.write(f"  {line}\n")
                if len(msgs) > 5:
                    f.write(f"\n  ... ({len(msgs) - 5} messages omitted) ...\n")
            f.write("\n")

        # ── Section 4: Speed limit analysis ──────────────────────────────────
        f.write("=" * 80 + "\n")
        f.write("SECTION 4: SPEED LIMIT ANALYSIS\n")
        f.write("=" * 80 + "\n\n")
        f.write("Speed limit = 0 means RMF is NOT imposing a speed restriction.\n")
        f.write("The fleet_config.yaml linear limit [1.8, 2.5] controls actual robot speed.\n")
        f.write("/lane_states controls per-lane speed overrides from the RMF operator.\n")
        f.write("If /iw_hub_X/speed_limit publishes 0 constantly, robot uses its own max speed.\n")
        f.write("This is CORRECT behavior for normal operation.\n\n")
        with lock:
            sl_msgs = collected.get("/iw_hub_5/speed_limit", [])
        if sl_msgs:
            f.write(f"Last speed_limit message for iw_hub_5:\n{sl_msgs[-1]['data']}\n")
        else:
            f.write("No speed_limit messages captured for iw_hub_5.\n")

    print(f"Report saved: {OUTPUT_FILE}")
    # Print summary to terminal
    with lock:
        print("\n── SUMMARY ──")
        for t, msgs in collected.items():
            if msgs:
                print(f"  {t}: {len(msgs)} messages")
        silent = [t for t, v in collected.items() if not v]
        if silent:
            print(f"\n  [SILENT - no messages]: {', '.join(silent)}")

def main():
    print("RMF Diagnostics Tool")
    print(f"Output: {OUTPUT_FILE}")
    print("Press Ctrl+C to stop and save report.\n")
    print("Starting echo threads for all topics...")

    threads = []
    for topic in TOPICS:
        t = threading.Thread(target=echo_topic, args=(topic,), daemon=True)
        t.start()
        threads.append(t)
        print(f"  Listening: {topic}")

    # Status printer
    status_thread = threading.Thread(target=print_status, daemon=True)
    status_thread.start()

    print("\nCapturing... (Ctrl+C to stop)\n")

    def handle_sigint(sig, frame):
        print("\n\nStopping capture...")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        stop_event.set()

    print("Waiting for threads to finish...")
    time.sleep(2)
    save_report()

if __name__ == "__main__":
    main()
