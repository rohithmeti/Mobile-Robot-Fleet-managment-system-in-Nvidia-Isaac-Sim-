import yaml
import math
import sys
import os

def get_pgm_dimensions(pgm_path):
    """Parses the PGM header to find the width and height of the image."""
    with open(pgm_path, 'rb') as f:
        tokens = []
        # Read file chunk by chunk to find header info without loading whole image
        while len(tokens) < 3:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if not line or line.startswith('#'):
                continue
            tokens.extend(line.split())
        
        if tokens[0] not in ['P2', 'P5']:
            raise ValueError("Not a valid PGM file. Missing P2 or P5 header.")
        
        width = int(tokens[1])
        height = int(tokens[2])
        return width, height

def process_map_data(yaml_path, pgm_path):
    # 1. Parse the YAML file
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    
    resolution = float(map_data['resolution'])
    origin_x = float(map_data['origin'][0])
    origin_y = float(map_data['origin'][1])
    yaw = float(map_data['origin'][2])
    
    # 2. Get Image Dimensions
    width, height = get_pgm_dimensions(pgm_path)
    
    # 3. Calculate Pixels per Meter
    pixels_per_meter = 1.0 / resolution
    
    # 4. Calculate RMF Origin (Top-Left)
    rmf_origin_x = origin_x - (height * resolution) * math.sin(yaw)
    rmf_origin_y = origin_y + (height * resolution) * math.cos(yaw)
    
    # 5. Output the results
    print("-" * 40)
    print(f"File Data:")
    print(f"  Image Size:      {width}x{height} pixels")
    print(f"  ROS Resolution:  {resolution} m/px")
    print(f"  ROS Origin:      [{origin_x}, {origin_y}, {yaw}]")
    print("-" * 40)
    print(f"Calculated Results:")
    print(f"  Scale:           {pixels_per_meter:.2f} pixels/meter")
    print(f"  RMF Origin X:    {rmf_origin_x:.4f}")
    print(f"  RMF Origin Y:    {rmf_origin_y:.4f}")
    print(f"  RMF Yaw:         {yaw:.4f}")
    print("-" * 40)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python rmf_map_converter.py <path_to_yaml> <path_to_pgm>")
        sys.exit(1)
        
    yaml_file = sys.argv[1]
    pgm_file = sys.argv[2]
    
    if not os.path.exists(yaml_file):
        print(f"Error: YAML file '{yaml_file}' not found.")
        sys.exit(1)
    if not os.path.exists(pgm_file):
        print(f"Error: PGM file '{pgm_file}' not found.")
        sys.exit(1)
        
    process_map_data(yaml_file, pgm_file)
