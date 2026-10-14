import keyboard
import time
import socket
from swift import Swift, Slider
from spatialmath import SE3
import spatialgeometry as sg
import numpy as np

# Simulation harness: drives an IK loop inside a Swift browser sim and, when
# send_esp32 is set, streams the resulting joint angles to the firmware in
# src/main.cpp over UDP. Press 'c' to quit.
#
# Run it via ik_dls_rpy.py rather than importing this directly.

# Static IP, or swap for the mDNS hostname: "braccio-robot.local"
UDP_IP = "braccio-robot.local" 
UDP_PORT = 4210


def Run(robot, loop, dt, initial_target, send_esp32=False):
    """Step `robot` toward `initial_target` by calling `loop(target)` each tick."""

    X_home = initial_target
    X_target = initial_target.copy()
    print(X_target)

    env = Swift()
    env.launch(realtime=True)
    env.add(robot)

    env.add_ui(Slider( min=-0.15, max=0.1, step=0.01, value=0.0, desc="Target X (Pos)"), name="x")
    env.add_ui(Slider( min=-1.0, max=1.0, step=0.01, value=0.0, desc="Target Y (Pos)"), name="y")
    env.add_ui(Slider( min=-0.5, max=0.3, step=0.01, value=0.0, desc="Target Z (Pos)"), name="z")
    env.add_ui(Slider( min=-np.pi, max=np.pi, step=0.05, value=0.0, desc="Target Roll"), name="phi")
    env.add_ui(Slider( min=-np.pi, max=np.pi, step=0.05, value=0.0, desc="Target Pitch"), name="theta")
    env.add_ui(Slider( min=-np.pi, max=np.pi, step=0.05, value=0.0, desc="Target Yaw"), name="psi")

    # Target pose marker
    marker = sg.Axes(length=0.1, pose=SE3(X_target[0:3]) * SE3.RPY(X_target[3:6]))
    env.add(marker)

    print("Simulation started. Press 'c' at any time to quit.")
    try:
        if send_esp32:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
        while True:
            if keyboard.is_pressed('c'):
                print("'c' pressed, shutting down...")
                break

            # Sliders are offsets from the home pose
            X_target = [
                X_home[0] + env.values["x"], 
                X_home[1] +env.values["y"], 
                X_home[2] +env.values["z"], 
                X_home[3] +env.values["phi"], 
                X_home[4] +env.values["theta"], 
                X_home[5] +env.values["psi"]
            ]
            
            marker.T = (SE3(X_target[0:3]) * SE3.RPY(X_target[3:6])).A
            
            loop(X_target)
            env.step(dt)

            if send_esp32:
                # 3 decimals is plenty for a servo and keeps the packet small
                angles = [round(float(q), 3) for q in robot.q]
                msg = f"{angles[0]},{angles[1]},{angles[2]},{angles[3]},{angles[4]},{angles[5]}\n"
                sock.sendto(msg.encode('utf-8'), (UDP_IP, UDP_PORT))
    except Exception as e:
        print(f"generic error: {e}")
    except KeyboardInterrupt:
        print("Closed from the terminal (Ctrl+C).")
    finally:
        print("Shutting the simulator down...")
        try:
            # give Swift a second to drain the last frames
            time.sleep(1) 
            env.close()
        except RuntimeError:
            pass
        print("Environment closed cleanly.")

