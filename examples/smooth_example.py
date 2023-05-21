from trajectory_smoothing import TrajectorySmoother
import numpy as np

def example_class():
    
    n = 20
    dof = 2
    max_velocity = np.zeros((dof),dtype=np.float32) + 1.5
    max_acceleration = np.zeros((dof),dtype=np.float32) + 0.1
    max_deviation = 0.05

    trajectory_sm = TrajectorySmoother(dof, max_acceleration, max_velocity, max_deviation)
    #trajectory_sm.test()
    #return
    traj = np.zeros((n,dof), dtype=np.float64)

    position = np.linspace(0, 1, int(n))
    traj[:position.shape[0],0] = position

    result = trajectory_sm.smooth_interpolate(traj, interpolation_dt=0.05, traj_dt=0.001)
    if result.success:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3,1, figsize=(4,10))
        for i in range(dof):
            ax[0].plot(result.position[:,i])
            ax[1].plot(result.velocity[:,i])
            ax[2].plot(result.acceleration[:,i])


        plt.tight_layout()
        plt.show()
if __name__ == "__main__":
    example_class()