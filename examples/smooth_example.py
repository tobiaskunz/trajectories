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

    result = trajectory_sm.smooth_interpolate(traj, interpolation_dt=0.02, traj_dt=0.001)
    if result.success:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(4,1, figsize=(4,10))
        t = [i*0.05 for i in range(result.length)]
        for i in range(dof):
            ax[0].plot(t,result.position[:,i])
            ax[1].plot(t,result.velocity[:,i])
            ax[2].plot(t,result.acceleration[:,i])
            ax[3].plot(t,result.jerk[:,i])
            
        ax[0].set_title("Joint Position")
        ax[1].set_title("Joint Velocity")
        ax[2].set_title("Joint Acceleration")
        ax[3].set_title("Joint Jerk")
        ax[3].set_xlabel("Time (s)")
        
        

        plt.tight_layout()
        plt.show()
if __name__ == "__main__":
    example_class()