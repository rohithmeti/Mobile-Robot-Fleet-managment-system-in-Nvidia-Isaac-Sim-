import importlib.util
import sys

# Standard Python/PIP libraries required for your adapter and API
pip_deps = [
    'yaml', 
    'nudged', 
    'requests', 
    'fastapi', 
    'uvicorn', 
    'pydantic'
]

# Core ROS 2 and RMF packages
ros_deps = [
    'rclpy',
    'geometry_msgs',
    'nav2_msgs',
    'rmf_adapter',
    'rmf_fleet_msgs',
    'rmf_task_msgs',
    'rmf_building_map_msgs'
]

def check_deps(deps, category):
    print(f"\n--- Checking {category} ---")
    missing = []
    for dep in deps:
        try:
            spec = importlib.util.find_spec(dep)
            if spec is None:
                print(f"[MISSING] {dep}")
                missing.append(dep)
            else:
                print(f"[OK]      {dep}")
        except ModuleNotFoundError:
            print(f"[MISSING] {dep}")
            missing.append(dep)
        except Exception as e:
            print(f"[ERROR]   {dep} (Failed to check: {e})")
            missing.append(dep)
    return missing

print(f"Using Python executable: {sys.executable}")
missing_pip = check_deps(pip_deps, "Python/PIP Dependencies")
missing_ros = check_deps(ros_deps, "ROS 2 / RMF Dependencies")

print("\n=== SUMMARY ===")
if not missing_pip and not missing_ros:
    print("SUCCESS: All required modules are detectable in the current path.")
else:
    print("FAILURE: You have missing dependencies.")
    print("If ROS 2 packages are missing, you must 'colcon build' them and 'source install/setup.bash'.")
    print("If PIP packages are missing, 'pip install' them inside your virtual environment.")
