from trajectory_smoothing import TrajectorySmoother
import numpy as np
def example_class():
    n = 5
    dof = 7
    max_velocity = np.zeros((dof),dtype=np.float32) + 2.0
    max_acceleration = max_velocity * 0.01
    max_deviation = 0.01

    trajectory_sm = TrajectorySmoother(dof, max_acceleration, max_velocity, max_deviation)
    traj = np.zeros((n,dof), dtype=np.float64)
    position = np.random.rand(n) * 0.1
    traj[:,0] = position
    traj[:,1] = position * -1.0

    out_pos, out_vel = trajectory_sm.smooth_interpolate(traj, dt=0.01)
    if out_pos is not None:
        import matplotlib.pyplot as plt

        plt.plot(out_pos[:,0])
        plt.tight_layout()
        plt.savefig('test.png')
        plt.close()
        print('saved')
    #print(out_pos[:,1])
    #print(out_vel[:,1])

example_class()