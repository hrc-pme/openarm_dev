import pybullet as p
import pybullet_data
import time

# --- Configuration ---
# Path to your URDF file, this is the path inside the docker container
URDF_PATH = "/root/openarm/openarm_urdf/openarm_bimanual.urdf"


def main():
    """
    Main function to initialize PyBullet, load the URDF, and run the simulation.
    """
    print("🚀 Starting PyBullet URDF Test...")

    # 1. Connect to the physics server
    # p.DIRECT for no GUI, p.GUI for a graphical interface
    try:
        physics_client = p.connect(p.GUI)
        print("✅ Connected to PyBullet in GUI mode.")
    except p.error as e:
        print(f"❌ Error connecting to PyBullet: {e}")
        print("   Please ensure you have a display environment (e.g., X11) configured.")
        return

    # 2. Set up the simulation environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To find plane.urdf
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Load a ground plane
    plane_id = p.loadURDF("plane.urdf")
    print(f"Loaded ground plane with ID: {plane_id}")

    # Set camera position for a better view
    p.resetDebugVisualizerCamera(
        cameraDistance=2.5,
        cameraYaw=30,
        cameraPitch=-20,
        cameraTargetPosition=[0, 0, 0.5]
    )

    # 3. Load the URDF model
    print(f"Attempting to load URDF: {URDF_PATH}")
    try:
        robot_id = p.loadURDF(URDF_PATH, basePosition=[0, 0, 0.8], useFixedBase=True)
        print(f"✅ Successfully loaded URDF. Robot ID: {robot_id}")

        num_joints = p.getNumJoints(robot_id)
        print(f"   Number of joints: {num_joints}")
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            print(f"   - Joint {i}: {joint_info[1].decode('utf-8')} ({p.getJointState(robot_id, i)[0]:.2f})")

# 1. 先把底座 (Base, Index = -1) 塗成灰色
        p.changeVisualShape(robot_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1.0])
        
        # 2. 把所有關節 (Joints) 也全部塗成灰色
        for i in range(num_joints):
            p.changeVisualShape(robot_id, i, rgbaColor=[0.7, 0.7, 0.7, 1.0])

    except p.error as e:
        print(f"❌ Error loading URDF file: {e}")
        print("   Please check the following:")
        print("   - The URDF file path is correct.")
        print("   - All mesh files referenced in the URDF exist and their paths are correct relative to the URDF file.")
        p.disconnect()
        return

    # 4. Run the simulation indefinitely
    print("\nRunning simulation indefinitely (Press Ctrl+C in terminal to exit)...")
    try:
        while True:
            p.stepSimulation()
            time.sleep(1. / 240.)
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")


    # 5. Disconnect from the physics server
    p.disconnect()
    print("Disconnected from PyBullet. Test complete. 👋")


if __name__ == '__main__':
    main()
