import json
import yaml
import os
import math

# ==============================================================================
# BLOCK 1: CONFIGURATION
# ==============================================================================
INPUT_FILE = "warehousermf.site.json"
OUTPUT_FILE = "warehousermf.building.yaml"

def convert_and_flip():
    print(f"Reading {INPUT_FILE}...")
    with open(INPUT_FILE) as f:
        site = json.load(f)

    level = site["levels"]["1"]
    anchors = level.get("anchors", {})
    lanes = site["navigation"]["guided"]["lanes"]
    locations = site["navigation"]["guided"]["locations"]

    # Map anchors to their location properties
    anchor_to_loc = {}
    for loc in locations.values():
        anchor_to_loc[str(loc["anchor"])] = loc

# ==============================================================================
# BLOCK 2: FLOORPLAN METADATA
# ==============================================================================
    floorplan_list = []
    ppm = 20.0  # fallback only, overwritten below if a drawing is found
    if "drawings" in level:
        for did, ddata in level["drawings"].items():
            props = ddata["properties"]

            source_file = props.get("source", {}).get("Local", "default_map.png")
            filename = os.path.basename(source_file)

            trans = props.get("pose", {}).get("trans", [0.0, 0.0, 0.0])
            yaw = props.get("pose", {}).get("rot", {}).get("yaw", {}).get("deg", 0.0)
            ppm = props.get("pixels_per_meter", ppm)

            floorplan_list.append({
                "filename": filename,
                "x_offset": trans[0],
                "y_offset": trans[1],
                "yaw": yaw,
                "scale": 1.0 / ppm if ppm > 0 else 0.05
            })

    print(f"Using pixels_per_meter = {ppm} for vertex scaling")

# ==============================================================================
# BLOCK 3: VERTICES, METER->PIXEL SCALING & Y-AXIS INVERSION
# ==============================================================================
    vertices = []
    anchor_idx = {}
    flipped_count = 0

    print("Converting vertices (meters -> pixels) and flipping Y-axis...")
    for i, (aid, adata) in enumerate(anchors.items()):
        x, y = adata["Translate2D"]

        x_px = x * ppm
        y_px = -y * ppm  
        flipped_count += 1

        name = ""
        props = {}
        if aid in anchor_to_loc:
            loc = anchor_to_loc[aid]
            name = loc.get("name", "")
            tags = loc.get("tags", [])

            if "Charger" in tags:
                props["is_charger"] = [4, True]
            if "ParkingSpot" in tags:
                props["is_parking_spot"] = [4, True]

            if name or "HoldingPoint" in tags:
                props["is_holding_point"] = [4, True]

        vertices.append([x_px, y_px, 0.0, name, props])
        anchor_idx[aid] = i

    print(f"Scaled and flipped {flipped_count} vertices.")

# ==============================================================================
# BLOCK 4: LANES & FLOORS
# ==============================================================================
    lane_list = []
    for lid, ldata in lanes.items():
        a, b = ldata["anchors"]
        lane_list.append([
            anchor_idx[str(a)],
            anchor_idx[str(b)],
            {
                "graph_idx": [2, 0],
                "bidirectional": [4, False],
                "orientation": [1, ""],
                "speed_limit": [3, 0.0]
            }
        ])

    floor_list = []
    if "floors" in level:
        for fid, fdata in level["floors"].items():
            floor_anchors = fdata["anchors"]
            vertex_indices = [anchor_idx[str(a)] for a in floor_anchors]
            floor_list.append({
                "parameters": {
                    "texture_name": [1, "blue_linoleum"],
                    "texture_rotation": [3, 0.0],
                    "texture_scale": [3, 1.0]
                },
                "vertices": vertex_indices
            })

# ==============================================================================
# BLOCK 4.5: MEASUREMENT INJECTION (CRITICAL FOR SCALE FIX)
# ==============================================================================
    measurements = []
    if len(vertices) >= 5:
        idx_a = 0
        idx_b = 4 
        dx_px = vertices[idx_a][0] - vertices[idx_b][0]
        dy_px = vertices[idx_a][1] - vertices[idx_b][1]
        pixel_dist = math.hypot(dx_px, dy_px)
        
        # Convert the pixel distance back to meters for the RMF measurement
        physical_dist = pixel_dist / ppm  
        measurements.append([idx_a, idx_b, {"distance": [2, physical_dist]}])
        print(f"Injected mathematical measurement: {physical_dist:.4f} meters")

# ==============================================================================
# BLOCK 5: EXPORT
# ==============================================================================
    building = {
        "name": "warehousermf",
        "levels": {
            "L1": {
                "elevation": 0.0,
                "floorplans": floorplan_list,
                "vertices": vertices,
                "lanes": lane_list,
                "walls": [],
                "measurements": measurements, # FIXED: Array is no longer empty
                "models": [],
                "floors": floor_list,
                "doors": []
            }
        }
    }

    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(building, f, default_flow_style=None)

    print(f"Success! Wrote {OUTPUT_FILE}.")

if __name__ == '__main__':
    convert_and_flip()
