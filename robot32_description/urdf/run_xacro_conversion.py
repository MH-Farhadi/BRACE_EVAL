import xacro
import os
import sys

# --- Configuration ---
input_xacro_filename = "robot32_for_pybullet.xacro"  # Changed
output_urdf_filename = "robot32_for_pybullet.urdf"  # Changed
# --- End Configuration ---

script_directory = os.path.dirname(os.path.realpath(__file__))
xacro_file_full_path = os.path.join(script_directory, input_xacro_filename)
urdf_file_full_path = os.path.join(script_directory, output_urdf_filename)

print(f"Input XACRO file: {xacro_file_full_path}")
print(f"Output URDF file: {urdf_file_full_path}")

if not os.path.exists(xacro_file_full_path):
    print(f"ERROR: Input XACRO file '{xacro_file_full_path}' not found.")
    sys.exit(1)

try:
    print("Processing XACRO to URDF...")
    doc = xacro.process_file(xacro_file_full_path)
    urdf_xml_string = doc.toprettyxml(indent='  ')

    with open(urdf_file_full_path, 'w') as f:
        f.write(urdf_xml_string)

    print(f"Successfully converted '{input_xacro_filename}' to '{output_urdf_filename}'")

except Exception as e:
    print(f"An error occurred during XACRO processing: {e}")
    if hasattr(e, 'args') and e.args:
        for arg in e.args:
            print(arg)
    sys.exit(1)