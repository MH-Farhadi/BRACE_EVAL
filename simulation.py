import pybullet as p
import pybullet_data
import time
import os
import math # Needed for pi

# --- Simulation Setup ---
try:
    physicsClient = p.connect(p.GUI)
    print("Connected to PyBullet GUI.")
except p.error as e:
    print(f"Failed to connect to PyBullet GUI, attempting DIRECT connection: {e}")
    try:
        physicsClient = p.connect(p.DIRECT)
        print("Connected to PyBullet DIRECT (no GUI). Sliders will not be visible.")
    except p.error as e_direct:
        print(f"Failed to connect to PyBullet DIRECT as well: {e_direct}")
        print("Ensure PyBullet is installed correctly.")
        exit()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# --- Robot Loading ---
# >>> IMPORTANT: THIS PATH SHOULD POINT TO THE GENERATED 'robot32.urdf' FILE <<<
# AFTER YOU HAVE CONVERTED IT FROM XACRO AND MODIFIED ITS MESH PATHS.
urdf_file_path = "C:/Users/coold/Downloads/RobotArmURDF/robot32_description/urdf/robot32_for_pybullet.urdf"

if not os.path.exists(urdf_file_path):
    print("-" * 50)
    print(f"ERROR: URDF file not found at '{urdf_file_path}'")
    print("Please ensure you have:")
    print("1. Converted your 'robot32.xacro' to 'robot32.urdf' using the 'xacro' command.")
    print("2. Updated the 'urdf_file_path' variable in this script to the correct absolute path of 'robot32.urdf'.")
    print("3. Modified mesh paths inside 'robot32.urdf' to be relative (e.g., '../meshes/mesh.stl').")
    print("-" * 50)
    if p.isConnected():
        p.disconnect()
    exit()

start_pos = [0, 0, 0.0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = -1

try:
    print(f"Attempting to load URDF: {urdf_file_path}")
    urdf_dir = os.path.dirname(urdf_file_path)
    p.setAdditionalSearchPath(urdf_dir)
    print(f"Added to PyBullet search path: {urdf_dir}")

    robotId = p.loadURDF(urdf_file_path,
                         start_pos,
                         start_orientation,
                         useFixedBase=True
                        )
    print(f"Robot loaded successfully with ID: {robotId}")

except Exception as e:
    print("="*30 + " ERROR LOADING URDF " + "="*30)
    print(f"Failed to load URDF: {e}")
    # ... (keep the detailed error messages from the previous script version) ...
    print("\nCommon troubleshooting steps AFTER XACRO CONVERSION:")
    print("1. Verify the 'urdf_file_path' points to your generated 'robot32.urdf'.")
    print("2. Double-check mesh paths inside your 'robot32.urdf' file:")
    print("   - They MUST be RELATIVE to the 'robot32.urdf' file's location (e.g., '../meshes/your_mesh.stl').")
    print("   - 'package://' paths will NOT work here.")
    print("3. Ensure mesh files (e.g., .stl) exist at the specified relative paths.")
    print("4. The generated 'robot32.urdf' might have syntax errors (less common if xacro conversion succeeded).")
    print("="*78)
    if p.isConnected():
        p.disconnect()
    exit()

# --- Joint Information and Control Setup ---
num_joints = p.getNumJoints(robotId)
print(f"Number of joints in robotId {robotId}: {num_joints}")

joint_sliders = []
active_joints_info = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_id = joint_info[0]
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]
    joint_lower_limit = joint_info[8]
    joint_upper_limit = joint_info[9]

    print(f"Joint Index: {i}, ID: {joint_id}, Name: {joint_name}, Type: {joint_type}, Limits: [{joint_lower_limit:.4f}, {joint_upper_limit:.4f}]")

    slider_min = joint_lower_limit
    slider_max = joint_upper_limit
    create_slider = False

    if joint_type == p.JOINT_REVOLUTE:
        if joint_lower_limit >= joint_upper_limit: # Likely a continuous joint or bad limits
            print(f"  -> Joint '{joint_name}' is Revolute with L>=U limits. Assuming continuous, setting slider range -pi to +pi.")
            slider_min = -math.pi
            slider_max = math.pi
            create_slider = True
        else: # Normal revolute joint with defined limits
            create_slider = True
    elif joint_type == p.JOINT_PRISMATIC:
        if joint_lower_limit < joint_upper_limit: # Normal prismatic joint
            create_slider = True
        else:
            print(f"  -> Skipping slider for Prismatic joint '{joint_name}' due to invalid limits (lower >= upper).")

    if create_slider:
        active_joints_info.append({
            'id': joint_id,
            'name': joint_name,
        })
        current_joint_state = p.getJointState(robotId, joint_id)
        current_joint_position = current_joint_state[0]
        # Ensure current position is within slider default range for continuous, if not, clamp it for slider init.
        # This is mostly for visual consistency of the slider's starting point.
        if joint_type == p.JOINT_REVOLUTE and joint_lower_limit >= joint_upper_limit:
            current_joint_position_for_slider = max(slider_min, min(slider_max, current_joint_position % (2*math.pi)))
            if current_joint_position_for_slider > math.pi : # map to -pi, pi
                 current_joint_position_for_slider -= 2*math.pi
        else:
            current_joint_position_for_slider = current_joint_position


        slider_id = p.addUserDebugParameter(joint_name,
                                            slider_min,
                                            slider_max,
                                            current_joint_position_for_slider)
        joint_sliders.append(slider_id)

if not active_joints_info:
    print("Warning: No movable (revolute or prismatic with valid limits) joints found to create sliders for.")

# --- Simulation Loop ---
try:
    print("\nSimulation started. Use sliders to control joints. Press Ctrl+C in the terminal to exit.")
    while p.isConnected():
        for i in range(len(joint_sliders)):
            slider_value = p.readUserDebugParameter(joint_sliders[i])
            joint_id_to_control = active_joints_info[i]['id']
            p.setJointMotorControl2(robotId,
                                    joint_id_to_control,
                                    p.POSITION_CONTROL,
                                    targetPosition=slider_value
                                    )
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    print("Simulation stopped by user (KeyboardInterrupt).")
except p.error as e:
    print(f"PyBullet error during simulation: {e}")
finally:
    if p.isConnected():
        print("Disconnecting from PyBullet.")
        p.disconnect()
    print("Simulation finished.")