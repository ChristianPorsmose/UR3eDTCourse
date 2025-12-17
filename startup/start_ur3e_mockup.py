"""
This module starts the executable in '../ur3e_mockup/' folder named 'ur3e_mockup'.
"""

import subprocess
import os


def start_robot_arm_mockup(ok_queue=None):
    """
    Starts the ur3e_mockup executable and keeps it running.
    Handles graceful shutdown via Ctrl+C (SIGINT).
    """
    # Get the path to the ur3e_mockup executable relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    executable_path = os.path.join(current_dir, "../ur3e_mockup/ur3e_mockup")

    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"Executable not found: {executable_path}")

    # Start the subprocess
    process = subprocess.Popen([executable_path])

    if ok_queue:
        ok_queue.put("started")

    try:
        # Keep the process running until interrupted
        process.wait()
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("\nShutting down robot arm mockup...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    finally:
        print("Robot arm mockup stopped.")


if __name__ == "__main__":
    start_robot_arm_mockup()
