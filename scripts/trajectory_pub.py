# This class publishes the trajectory to rviz for visualization
#TODO: Implement for Allegro hand.
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory,RobotTrajectory
import roslib
import actionlib
#roslib.load_manifest('lbr4_interface')
#from joint_trajectory_action.msg import joint_trajectory_action
#from pr2_controllers_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
import numpy as np
import matplotlib.pyplot as plt


class trajectoryServer:
    def __init__(self,loop_rate,robot_name='lbr4',joint_names=['lbr4_j0','lbr4_j1','lbr4_j2','lbr4_j3','lbr4_j4','lbr4_j5','lbr4_j6'],topic_name='lbr4/joint_trajectory_viz',init_node=True,init_jtas_client=False):
        ## Initialize ros node
        if(init_node):
            rospy.init_node("joint_traj_viz")
        self.rate=rospy.Rate(loop_rate)
        
        tr_topic=topic_name
        self.pub_tr=rospy.Publisher(tr_topic,DisplayTrajectory,queue_size=1)
        self.joint_names=joint_names
        self.robot_name=robot_name
        if(init_jtas_client):
            self.client=actionlib.SimpleActionClient(topic_name,FollowJointTrajectoryAction)

    def plot_j_acc_profile(self,j_traj,j_ids):
        
        for l in j_ids:
            # get joint_positions:
            j_arr=[]
            for i in range(len(j_traj.points)):
                j_arr.append(j_traj.points[i].positions[l])
            
            # compute acc:
            pos_arr=j_arr
            acc_prof=[]
            vel_prof=[]
            for t in range(1,len(pos_arr)-1):
                acc=pos_arr[t-1]-2.0*pos_arr[t]+pos_arr[t+1]
                acc_prof.append(acc)
            
            for t in range(1,len(pos_arr)):
                vel=-pos_arr[t-1]+pos_arr[t]
                vel_prof.append(vel)
            i=0
            plt.subplot(311)
            plt.plot(pos_arr,linewidth=4,label=str(l))
            plt.xlabel('Timestep')
            plt.ylabel('Joint Position(rad)')
            
            plt.subplot(312)
            plt.plot(vel_prof,linewidth=4,label=str(l))
            plt.xlabel('Timestep')
            plt.ylabel('Joint Velocity')
            plt.subplot(313)
            plt.plot(acc_prof,linewidth=4,label=str(l))
            plt.xlabel('Timestep')
            plt.ylabel('Joint acceleration')
            
            plt.legend()

            plt.show()


    def send_traj(self,u_arr):
        # Formulate joint trajectory message:
        jt_msg=JointTrajectory()
        jt_msg.joint_names=self.joint_names
        for i in range(len(u_arr)):
            u=u_arr[i]
            jt_pt=JointTrajectoryPoint()
            jt_pt.positions=u
            jt_msg.points.append(jt_pt)

        robot_tr=RobotTrajectory()
        robot_tr.joint_trajectory=jt_msg
        disp_tr=DisplayTrajectory()
        disp_tr.trajectory.append(robot_tr)
        disp_tr.model_id=self.robot_name
        self.pub_tr.publish(disp_tr)
        i=0
        while( not rospy.is_shutdown() and i<10):
            i+=1
            self.rate.sleep()
    def get_jtraj(self,js):
        # Formulate joint trajectory message:
        jt_msg=JointTrajectory()
        jt_msg.joint_names=js.name
        #for i in range(len(js.position)):
        u=js.position
        jt_pt=JointTrajectoryPoint()
        jt_pt.positions=u
        jt_msg.points.append(jt_pt)
        return jt_msg
    
    def joint_traj_jtas(self,j_traj):
        self.client.wait_for_server()
        goal=FollowJointTrajectoryGoal()


        goal.trajectory=j_traj
        self.client.send_goal(goal)


    def viz_joint_traj(self,j_traj,base_link='base_link'):
        robot_tr=RobotTrajectory()
        j_traj.header.frame_id=base_link
        robot_tr.joint_trajectory=j_traj

        disp_tr=DisplayTrajectory()
        disp_tr.trajectory.append(robot_tr)
        disp_tr.model_id=self.robot_name
        
        i=0
        while( not rospy.is_shutdown() and i<10):
            i+=1
            self.pub_tr.publish(disp_tr)
           
            self.rate.sleep()
        
        
