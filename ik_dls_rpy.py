import roboticstoolbox as rtb
import numpy as np
from main import Run

robot = rtb.models.URDF.Puma560()

# Comfortable starting pose
robot.q = [0, np.pi/4, -np.pi/4, 0, 0, 0]

# Target pose [x, y, z, roll, pitch, yaw]
X_target = np.array([0.6, 0.18, 0.65, 0.0, np.pi/2, 0.0])


dt = 0.05       # Control step
lmbda = 0.1     # DLS damping factor
I = np.eye(6)   # Required because we invert J J^T, which is 6x6 here



def loop(X_target):
    T_current = robot.fkine(robot.q)

    # State vector: position stacked with RPY angles
    X_current = np.hstack((T_current.t, T_current.rpy() ))

    error = X_target - X_current

    # Wrap the angular error to (-pi, pi] so the arm takes the short way round
    error[3:] = np.arctan2(np.sin(error[3:]), np.cos(error[3:]))

    # Analytic Jacobian, same RPY convention as the state vector
    Ja = robot.jacob0_analytical(robot.q, representation='rpy/xyz')

    # Damped least squares: (J J^T + lambda^2 I)^-1 J^T, avoids inverting J itself
    Ja_dls = Ja.T @ np.linalg.inv(Ja @ Ja.T + (lmbda**2) * I)

    dq = Ja_dls @ error
    robot.q += dq * dt


if __name__ == "__main__":
    Run(robot, loop=loop, dt=dt, initial_target=X_target)