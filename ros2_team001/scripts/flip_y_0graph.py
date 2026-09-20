#!/usr/bin/env python3
"""
flip_y_nav_graph.py
Run this EVERY TIME after building_map_generator to fix Y coordinates.
Usage: python3 flip_y_nav_graph.py <path_to_0.yaml>
"""
import sys
import yaml

def flip_y(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    count = 0#!/usr/bin/env python3
"""
flip_y_nav_graph.py
Run this EVERY TIME after building_map_generator to fix Y coordinates.
Usage: python3 flip_y_nav_graph.py <path_to_0.yaml>
"""
import sys
import yaml

def flip_y(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    count = 0
    for level in data['levels'].values():
        for v in level.get('vertices', []):
            v[1] = -v[1]
            count += 1

    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    print(f"Flipped Y on {count} vertices in: {path}")
    print("\nKey waypoints after flip:")
    for level in data['levels'].values():
        for v in level.get('vertices', []):
            name = v[2].get('name', '') if isinstance(v[2], dict) else ''
            if name:
                print(f"  {name}: [{v[0]:.4f}, {v[1]:.4f}]")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 flip_y_nav_graph.py <path_to_0.yaml>")
        sys.exit(1)
    flip_y(sys.argv[1])
    for level in data['levels'].values():
        for v in level.get('vertices', []):
            v[1] = -v[1]
            count += 1

    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    print(f"Flipped Y on {count} vertices in: {path}")
    print("\nKey waypoints after flip:")
    for level in data['levels'].values():
        for v in level.get('vertices', []):
            name = v[2].get('name', '') if isinstance(v[2], dict) else ''
            if name:
                print(f"  {name}: [{v[0]:.4f}, {v[1]:.4f}]")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 flip_y_nav_graph.py <path_to_0.yaml>")
        sys.exit(1)
    flip_y(sys.argv[1])
