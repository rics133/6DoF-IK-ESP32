import roboticstoolbox as rtb
import swift
import time

# 6-DoF industrial arm
robot = rtb.models.URDF.Puma560()

# Swift opens a browser window automatically
env = swift.Swift()
env.launch(realtime=True)

env.add(robot)

# Joint space test: sweep the base and shoulder
print(f"Starting robot angles: {robot.q}")

for _ in range(100):
    robot.q[0] += 0.02
    robot.q[1] -= 0.01
    
    env.step(0.05)
    time.sleep(0.05)
    
print("Test Completed.")