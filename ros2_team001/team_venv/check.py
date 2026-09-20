import subprocess
import re

def get_topic_qos():
    # 1. Get the list of all active topics
    try:
        topic_list_raw = subprocess.check_output(["ros2", "topic", "list"]).decode("utf-8")
        topics = [t.strip() for t in topic_list_raw.split('\n') if t.strip()]
    except Exception as e:
        print(f"Error getting topic list: {e}")
        return

    print(f"{'TOPIC NAME':<60} | {'RELIABILITY':<15} | {'DURABILITY':<15}")
    print("-" * 95)

    for topic in topics:
        try:
            # 2. Run topic info -v for each topic
            info_raw = subprocess.check_output(["ros2", "topic", "info", "-v", topic]).decode("utf-8")
            
            # 3. Use regex to find the Publisher's QoS Profile (not the subscriber)
            # We look for the first QoS profile block under a Publisher
            reliability = "Unknown"
            durability = "Unknown"

            reliability_match = re.search(r"Reliability: (.*)", info_raw)
            durability_match = re.search(r"Durability: (.*)", info_raw)

            if reliability_match:
                reliability = reliability_match.group(1).strip()
            if durability_match:
                durability = durability_match.group(1).strip()

            print(f"{topic:<60} | {reliability:<15} | {durability:<15}")
        except Exception:
            continue

if __name__ == "__main__":
    get_topic_qos()
