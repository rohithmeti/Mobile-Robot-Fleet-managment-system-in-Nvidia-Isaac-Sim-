import os
from pathlib import Path
import datetime

def generate_directory_tree(target_path, output_filename):
    # Expand the '~' to the absolute home directory path
    base_dir = Path(target_path).expanduser().resolve()
    
    if not base_dir.exists() or not base_dir.is_dir():
        print(f"❌ Error: The directory '{base_dir}' does not exist.")
        return

    print(f"🔍 Scanning directory: {base_dir}...")
    
    with open(output_filename, 'w', encoding='utf-8') as f:
        # Write a header
        f.write(f"Directory Structure Report\n")
        f.write(f"Target Directory: {base_dir}\n")
        f.write(f"Generated on: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 70 + "\n\n")

        # Traverse the directory
        for root, dirs, files in os.walk(base_dir):
            # Sort directories and files for alphabetical output
            dirs.sort()
            files.sort()
            
            # Calculate the indentation level based on depth
            level = root.replace(str(base_dir), '').count(os.sep)
            indent = '│   ' * level
            
            folder_name = os.path.basename(root)
            if level == 0:
                f.write(f"📁 {folder_name}/\n")
            else:
                f.write(f"{indent}├── 📁 {folder_name}/\n")
            
            # Sub-indentation for files inside the current folder
            file_indent = '│   ' * (level + 1)
            
            for file in files:
                file_path = Path(root) / file
                # Extract file extension (type)
                file_ext = file_path.suffix if file_path.suffix else "(No extension)"
                
                # Write file details
                f.write(f"{file_indent}├── 📄 {file}  |  Type: {file_ext}\n")

    print(f"✅ Successfully saved folder structure to: {Path(output_filename).resolve()}")

if __name__ == "__main__":
    # Define the target directory and the output text file name
    target_directory = "~/ros2_team001/src"
    output_file = "src_folder_structure.txt"
    
    generate_directory_tree(target_directory, output_file)
