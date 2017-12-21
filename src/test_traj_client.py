import rospy
from trajectory_smoothing.srv import GetSmoothTraj
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt

def plot_j_acc_profile(j_traj,idx):
    l=idx
    # get joint_positions:
    j_arr=[]
    vel_prof=[]
    acc_prof=[]
    for i in range(len(j_traj.points)):
        j_arr.append(j_traj.points[i].positions[l])
        vel_prof.append(j_traj.points[i].velocities[l])
        acc_prof.append(j_traj.points[i].accelerations[l])
        
    plt.plot(j_arr,linewidth=4,label='position')
    plt.plot(acc_prof,linewidth=4,label='acc')
    plt.plot(vel_prof,linewidth=4,label='velocity')
    plt.legend()

    plt.show()

if __name__=='__main__':
    rospy.init_node('smooth_traj_client')
    joint_names=['lbr4_j0','lbr4_j1','lbr4_j2','lbr4_j3','lbr4_j4','lbr4_j5','lbr4_j6']
    n=100
    j_init=np.zeros(7)
    j_final=np.ones(7)
    j_final[0]=-1.2
    j_wp=[]
    j_traj=JointTrajectory()

    # create waypoints:
    for i in range(n):
        j=j_init+(j_final-j_init)*float(i)/float(n)
        j_wp.append(j)

    
    # store in joint trajectory msg:
    for i in range(n):
        pt=JointTrajectoryPoint()
        pt.positions=j_wp[i]
        j_traj.points.append(pt)

    j_traj.joint_names=joint_names

    max_acc=np.ones(7)*0.1
    max_vel=np.ones(7)*0.2
    # call service for smoothing:
    rospy.wait_for_service('/get_smooth_trajectory')
    traj_call=rospy.ServiceProxy('/get_smooth_trajectory',GetSmoothTraj)
    resp=traj_call(j_traj,max_acc,max_vel,0.01,0.001)
    #print resp.smooth_traj
    plot_j_acc_profile(resp.smooth_traj,1)
    
